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

import agents.Agent;
import agents.RealAgent;
import agents.TeammateAgent;
import config.Constants;
import config.RobotConfig;
import config.SimulatorConfig;
import java.awt.Point;
import java.util.LinkedList;

/**
 *
 * @author Christian Clausen
 */
public class RelayFrontierExploration {

    // Returns new X, Y of RealAgent
    public static Point takeStep(RealAgent agent, int timeElapsed,
            SimulatorConfig.frontiertype frontierExpType, Agent baseStation, 
            boolean useRelayStations, double dropChance) {
        long realtimeStartAgentCycle = System.currentTimeMillis();

        Point nextStep;
        if (useRelayStations && (Math.random() < dropChance)) {
            agent.dropComStation();
        }

        //<editor-fold defaultstate="collapsed" desc="If base station is in range,">
        //update timeLastDirectContactCS and lastContactAreaKnown and refill relayStations
        if (agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInRange()) {
            //When in com-range, maybe even in handoverRange?
            if (agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInHandoverRange(agent) &&
                    (agent.comStations.size() < agent.getComStationLimit())) {
                //TODO One Comstation per step... needs to be enough by now
                agent.takeComStation(baseStation.giveComStation());
            }
            agent.getStats().setTimeLastDirectContactCS(1);
            agent.getStats().setLastContactAreaKnown(agent.getStats().getAreaKnown());
        } else {
            agent.getStats().incrementLastDirectContactCS();
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="CHECK 0: Take a couple of random steps to start (just to gather some sensor data).">
        if (timeElapsed < Constants.INIT_CYCLES) {
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
        } //</editor-fold>
        //<editor-fold defaultstate="collapsed" desc="CHECK 1: if agent hasn't moved, then he may be stuck in front of a wall.  Taking a random step might help.">
        else if (agent.getEnvError()) {
            agent.resetPathToBaseStation();
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setEnvError(false);
        } //</editor-fold>
        //<editor-fold defaultstate="collapsed" desc="CHECK 2: Agent isn't stuck, not yet time to replan. Continue if we have points left in the previously planned path.">
        else if ((agent.getStats().getTimeSinceLastPlan() < Constants.REPLAN_INTERVAL)
                && agent.getPath().found && agent.getPath().getPoints().size() >= 2) {
            nextStep = agent.getNextPathPoint();
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Agent " + agent + " continuing on path.");
            }
        } //</editor-fold>
        //<editor-fold defaultstate="collapsed" desc="CHECK 3: Agent isn't stuck. Is it time to replan? Have we moved on to next time cycle?">
        else if (agent.getStats().getTimeSinceLastPlan() >= Constants.REPLAN_INTERVAL) {
            nextStep = replan(agent, frontierExpType, timeElapsed);
        } //</editor-fold>
        //<editor-fold defaultstate="collapsed" desc="CHECK 4: Agent isn't stuck, not yet time to replan, but we have no points left">
        else {
            nextStep = replan(agent, frontierExpType, timeElapsed);
        }
        //</editor-fold>

        agent.getStats().incrementTimeSinceLastPlan();

        //agent.setLastTotalKnowledgeBelief(agent.getCurrentTotalKnowledgeBelief());
        //agent.setLastBaseKnowledgeBelief(agent.getCurrentBaseKnowledgeBelief());
        //agent.setLastNewInfo(agent.getNewInfo());
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent.toString() + "takeStep took " + 
                    (System.currentTimeMillis() - realtimeStartAgentCycle) + "ms.");
        }
        return nextStep;
    }

    public static Point replan(RealAgent agent, SimulatorConfig.frontiertype frontierExpType, 
            int timeElapsed) {
        Point nextStep;

        agent.getStats().setTimeSinceLastPlan(0);
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent.toString() + "starting replan");
        }
        long realtimeStart = System.currentTimeMillis();

        FrontierExploration.calculateFrontiers(agent, frontierExpType);

        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent.toString() + "calculateFrontiers took " + 
                    (System.currentTimeMillis() - realtimeStart) + "ms.");
        }

        //<editor-fold defaultstate="collapsed" desc="If no frontiers found, or reached exploration goal, return to ComStation">
        if (((agent.getFrontiers().isEmpty()) || (agent.getStats().getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))
                && timeElapsed > 100) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + " setting mission complete");
            }
            agent.setMissionComplete(true);
            agent.setPathToBaseStation();
            nextStep = agent.getNextPathPoint();
            while ((nextStep != null) && (nextStep.equals(agent.getLocation()))) {
                nextStep = agent.getNextPathPoint();
            }
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setCurrentGoal(agent.getTeammate(1).getLocation());
            return nextStep;
        }
        //</editor-fold>

        long realtimeStart2 = System.currentTimeMillis();
        boolean foundFrontier = false;

        if (!agent.getSimConfig().keepAssigningRobotsToFrontiers()) {
            foundFrontier = (FrontierExploration.chooseFrontier(agent, true, null) == null);
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent.toString() + "chooseFrontier took " + 
                        (System.currentTimeMillis() - realtimeStart2) + "ms.");
            }

            //<editor-fold defaultstate="collapsed" desc="If could not find frontier, try to disregard other agents when planning">
            if (!foundFrontier) {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(agent.toString() + 
                            " could not find frontier, trying to ignore other agents...");
                }
                foundFrontier = (FrontierExploration.chooseFrontier(agent, false, null) == null);
            }
            //</editor-fold>
        } else {
            LinkedList<Integer> assignedTeammates = new LinkedList<Integer>();
            for (int i = 0; (i < agent.getAllTeammates().size()) && !foundFrontier; i++) {
                assignedTeammates = FrontierExploration.chooseFrontier(agent, true, assignedTeammates);
                if (assignedTeammates == null) {
                    foundFrontier = true;
                }
            }
        }

        if ((agent.getRole() == RobotConfig.roletype.Relay) && 
                (agent.getState() == Agent.ExploreState.GoToChild)) {
            return RoleBasedExploration.takeStep_GoToChild(agent);
        }

        //<editor-fold defaultstate="collapsed" desc="If no frontier could be assigned, then go back to base.">
        if (!foundFrontier && timeElapsed > 100) {
            /*System.out.println(agent.toString() + " No frontier chosen, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
            agent.setTimeSinceLastPlan(0);
            agent.setCurrentGoal(nextStep);
            return nextStep;*/
            // mission complete
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent.toString() + " could not find frontier, proceeding to BaseStation (Mission Complete).");
            }
            agent.setMissionComplete(true);
            agent.setPathToBaseStation();
            nextStep = agent.getNextPathPoint();
            while ((nextStep != null) && (nextStep.equals(agent.getLocation()))) {
                nextStep = agent.getNextPathPoint();
            }
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setCurrentGoal(agent.getTeammate(1).getLocation());
            return nextStep;
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="If overlapping another agent, take random step">
        for (TeammateAgent teammate : agent.getAllTeammates().values()) {
            if (agent.getLocation().equals(teammate.getLocation())) {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(agent + " overlapping " + teammate + ", taking random step");
                }
                nextStep = RandomWalk.takeStep(agent);
                agent.getStats().setTimeSinceLastPlan(0);
                agent.setCurrentGoal(nextStep);
                return nextStep;
            }
        }
        //</editor-fold>

        // Note: Path to best frontier has already been set when calculating
        // utility, no need to recalculate
        // Check that we have a path, otherwise take random step
        if ((agent.getPath() == null)
                || agent.getPath().getPoints() == null
                || agent.getPath().getPoints().isEmpty()
                || agent.getPath().getPoints().size() == 1) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + " has no path, taking random step.");
            }
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setEnvError(false);
            return nextStep;
        }
        // If we reach this point, we have a path.  Remove the first point
        // since this is the robot itself.
        agent.getPath().getPoints().remove(0);
        nextStep = agent.getNextPathPoint();
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent.toString() + "replan took " + 
                    (System.currentTimeMillis() - realtimeStart) + "ms.");
        }
        return nextStep;
    }
   
}
