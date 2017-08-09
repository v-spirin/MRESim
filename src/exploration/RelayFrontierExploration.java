/*
 *     Copyright 2010, 2015, 2017 Julian de Hoog (julian@dehoog.ca),
 *     Victor Spirin (victor.spirin@cs.ox.ac.uk),
 *     Christian Clausen (christian.clausen@uni-bremen.de
 *
 *     This file is part of MRESim 2.3, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our papers:
 *
 *     @inproceedings{deHoog2009,
 *         title = "Role-Based Autonomous Multi-Robot Exploration",
 *         author = "Julian de Hoog, Stephen Cameron and Arnoud Visser",
 *         year = "2009",
 *         booktitle =
 *     "International Conference on Advanced Cognitive Technologies and Applications (COGNITIVE)",
 *         location = "Athens, Greece",
 *         month = "November",
 *     }
 *
 *     @incollection{spirin2015mresim,
 *       title={MRESim, a Multi-robot Exploration Simulator for the Rescue Simulation League},
 *       author={Spirin, Victor and de Hoog, Julian and Visser, Arnoud and Cameron, Stephen},
 *       booktitle={RoboCup 2014: Robot World Cup XVIII},
 *       pages={106--117},
 *       year={2015},
 *       publisher={Springer}
 *     }
 *
 *     MRESim is free software: you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation, either version 3 of the License, or
 *     (at your option) any later version.
 *
 *     MRESim is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License along with MRESim.
 *     If not, see <http://www.gnu.org/licenses/>.
 */

package exploration;

import agents.RealAgent;
import agents.TeammateAgent;
import config.Constants;
import config.SimulatorConfig;
import environment.OccupancyGrid;
import environment.TopologicalMap;
import java.awt.Point;
import java.util.LinkedList;
import path.Path;

/**
 * Using relays in FrontierExploration
 *
 * @author Christian Clausen
 */
public class RelayFrontierExploration extends FrontierExploration {

    boolean useRelayStations;
    double dropChance;
    SimulatorConfig.relaytype relayType;
    private final OccupancyGrid occGrid;
    private Point currentGoal;
    private LinkedList<Point> keyPoints;

    public RelayFrontierExploration(RealAgent agent, SimulatorConfig simConfig,
            RealAgent baseStation) {
        super(agent, simConfig, baseStation);
        this.useRelayStations = simConfig.useComStations();
        this.dropChance = simConfig.getComStationDropChance();
        this.baseStation = baseStation;
        this.relayType = simConfig.getRelayAlgorithm();
        this.occGrid = agent.getOccupancyGrid();
        this.state = ExplorationState.Exploring;
        this.keyPoints = new LinkedList<>();
    }

    @Override
    public Point takeStep(int timeElapsed) {

        Point nextStep;
        /*if (useRelayStations && (Math.random() < dropChance)) {
            agent.dropComStation();
        }*/
        for (Point p : keyPoints) {
            if (agent.getLocation().distance(p) < Constants.KEY_POINT_RELAY_DISTANCE && noRelay(p)) {
                System.out.println("Near->replan");
                replan(timeElapsed);
            }
        }
        /* If base station is in range,
         * update timeLastDirectContactCS and lastContactAreaKnown and refill relayStations
         */
        if (agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()) {
            //When in com-range, maybe even in handover-range?
            if (agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInHandoverRange(agent)
                    && (agent.comStations.size() < agent.getComStationLimit())) {
                for (int i = 0; i < agent.getComStationLimit(); i++) {
                    agent.takeComStation(baseStation.giveComStation());
                }
            }
            agent.getStats().setTimeLastDirectContactCS(1);
            agent.getStats().setLastContactAreaKnown(agent.getStats().getAreaKnown());
        } else {
            agent.getStats().incrementLastDirectContactCS();
        }

        if (timeElapsed < Constants.INIT_CYCLES) {
            // CHECK 0: Take a couple of random steps to start (just to gather some sensor data)
            nextStep = RandomWalk.randomStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
        } else if (agent.getEnvError()) {
            // CHECK 1: if agent hasn't moved, then he may be stuck in front of a wall.
            // Taking a random step might help.
            agent.resetPathToBaseStation();
            nextStep = RandomWalk.randomStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setEnvError(false);
        } else if ((agent.getStats().getTimeSinceLastPlan() < Constants.REPLAN_INTERVAL)
                && agent.getPath() != null && agent.getPath().found && agent.getPath().getPoints().size() >= 2) {
            // CHECK 2: Agent isn't stuck, not yet time to replan.
            // Continue if we have points left in the previously planned path.
            nextStep = agent.getNextPathPoint();
        } else if (agent.getStats().getTimeSinceLastPlan() >= Constants.REPLAN_INTERVAL) {
            // CHECK 3: Agent isn't stuck. Is it time to replan?
            //Have we moved on to next time cycle?
            nextStep = replan(timeElapsed);
        } else {
            // CHECK 4: Agent isn't stuck, not yet time to replan, but we have no points left
            nextStep = replan(timeElapsed);
        }

        agent.getStats().incrementTimeSinceLastPlan();

        return nextStep;
    }

    @Override
    public Point replan(int timeElapsed) {
        Point goal;

        agent.getStats().setTimeSinceLastPlan(0);
        long realtimeStart = System.currentTimeMillis();

        switch (state) {
            case Exploring:
                goal = replan_explore(realtimeStart, timeElapsed);
                break;

            case SettingRelay:
                if (currentGoal == null) {
                    currentGoal = replan_frontier(realtimeStart, timeElapsed);
                    state = ExplorationState.Exploring;
                }
                if (agent.getLocation().distance(currentGoal) <= 1) {
                    agent.dropComStation();
                    state = ExplorationState.Exploring;
                }
                goal = currentGoal;
                //agent.setPath(tpath);
                break;
            default:
                goal = replan_frontier(realtimeStart, timeElapsed);
        }
        agent.setPath(new Path(occGrid, agent.getLocation(), goal, false, true));
        return goal;
    }

    private Point replan_explore(long realtimeStart, int timeElapsed) {
        Point goal;
        Path tpath;
        switch (relayType) {
            case KeyPoints:
                if (!agent.comStations.isEmpty()) {

                    TopologicalMap tmap = new TopologicalMap(occGrid);
                    tmap.generateSkeleton();
                    for (Point p : tmap.getJunctionPoints()) {
                        simulator.ExplorationImage.addErrorMarker(p, "", true);
                        if (agent.getLocation().distance(p) < Constants.KEY_POINT_RELAY_DISTANCE) {
                            if (noRelay(p) && noNearRelay(p)) {
                                currentGoal = p;
//                                tpath = new Path(occGrid, agent.getLocation(), p, false, true);
//                                agent.setPath(tpath);
                                state = ExplorationState.SettingRelay;
                                break;
                            }
                        }

                    }
                }
                //No ComStation to place
                goal = replan_frontier(realtimeStart, timeElapsed);

                break;
            case RangeBorder:
                //Todo
                goal = replan_frontier(realtimeStart, timeElapsed);
                break;
            case Random:
                //dropChance is drop/step, this is speed-long planning
                if (useRelayStations && (Math.random() < dropChance * agent.getSpeed())) {
                    agent.dropComStation();
                }
                goal = replan_frontier(realtimeStart, timeElapsed);
                break;
            case None:
            default:
                goal = replan_frontier(realtimeStart, timeElapsed);
        }
        return goal;
    }

    private Point replan_frontier(long realtimeStart, int timeElapsed) {
        Point nextStep;
        calculateFrontiers();

        //If no frontiers found, or reached exploration goal, return to ComStation
        if (((frontiers.isEmpty()) || (agent.getStats().getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))
                && timeElapsed > 100) {
            agent.setMissionComplete(true);
            agent.setPathToBaseStation();
            nextStep = agent.getNextPathPoint();
            while ((nextStep != null) && (nextStep.equals(agent.getLocation()))) {
                nextStep = agent.getNextPathPoint();
            }
            agent.getStats().setTimeSinceLastPlan(0);
            return nextStep;
        }

        long realtimeStart2 = System.currentTimeMillis();
        boolean foundFrontier = false;
        if (!agent.getSimConfig().keepAssigningRobotsToFrontiers()) {
            foundFrontier = (chooseFrontier(true, null) == null);

            //If could not find frontier, try to disregard other agents when planning
            if (!foundFrontier) {
                foundFrontier = (chooseFrontier(false, null) == null);
            }
        } else {
            LinkedList<Integer> assignedTeammates = new LinkedList<Integer>();
            for (int i = 0; (i < agent.getAllTeammates().size()) && !foundFrontier; i++) {
                assignedTeammates = chooseFrontier(true, assignedTeammates);
                if (assignedTeammates == null) {
                    foundFrontier = true;
                }
            }
        }

        //If no frontier could be assigned, then go back to base.
        if (!foundFrontier && timeElapsed > 100) {
            // mission complete
            agent.setMissionComplete(true);
            agent.setPathToBaseStation();
            nextStep = agent.getNextPathPoint();
            while ((nextStep != null) && (nextStep.equals(agent.getLocation()))) {
                nextStep = agent.getNextPathPoint();
            }
            agent.getStats().setTimeSinceLastPlan(0);
            return nextStep;
        }

        //If overlapping another agent, take random step
        for (TeammateAgent teammate : agent.getAllTeammates().values()) {
            if (agent.getLocation().equals(teammate.getLocation())) {
                nextStep = RandomWalk.randomStep(agent);
                agent.getStats().setTimeSinceLastPlan(0);
                return nextStep;
            }
        }

        // Note: Path to best frontier has already been set when calculating
        // utility, no need to recalculate
        // Check that we have a path, otherwise take random step
        if ((agent.getPath() == null)
                || agent.getPath().getPoints() == null
                || agent.getPath().getPoints().isEmpty()
                || agent.getPath().getPoints().size() == 1) {
            nextStep = RandomWalk.randomStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setEnvError(false);
            return nextStep;
        }
        // If we reach this point, we have a path.  Remove the first point
        // since this is the robot itself.
        agent.getPath().getPoints().remove(0);
        nextStep = agent.getNextPathPoint();
        return nextStep;
    }

}
