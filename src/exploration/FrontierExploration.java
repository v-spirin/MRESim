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
import config.SimConstants;
import config.SimulatorConfig;
import environment.ContourTracer;
import environment.Frontier;
import environment.OccupancyGrid;
import exploration.Frontier.FrontierUtility;
import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.PriorityQueue;
import path.Path;

/**
 * Calculate Frontiers, weight them by utility and decide which one to explore first.
 *
 * @author julh, Christian Clausen
 */
public class FrontierExploration extends BasicExploration implements Exploration {

    SimulatorConfig.frontiertype frontierExpType;
    RealAgent baseStation;
    int noReturnTimer;

    PriorityQueue<Frontier> frontiers;
    Frontier lastFrontier;          // Keep track of last frontier of interest
    //Frontiers that are impossible to reach, so should be discarded
    HashMap<Frontier, Boolean> badFrontiers;
    private double last_percentage_known = 0;
    private int no_change_counter = 0;

    /**
     * Normal Constructor
     *
     * @param agent
     * @param simConfig
     * @param baseStation
     */
    public FrontierExploration(RealAgent agent, SimulatorConfig simConfig, RealAgent baseStation) {
        super(agent, simConfig, Agent.ExplorationState.Initial);
        this.agent = agent;
        this.frontierExpType = simConfig.getFrontierAlgorithm();
        this.baseStation = baseStation;
        this.noReturnTimer = 0;
        this.frontiers = new PriorityQueue<>();
    }

    /**
     * Constructor for Explorations using a specific frontier-type
     *
     * @param agent
     * @param simConfig
     * @param baseStation
     * @param frontierType
     */
    protected FrontierExploration(RealAgent agent, SimulatorConfig simConfig, RealAgent baseStation, SimulatorConfig.frontiertype frontierType) {
        super(agent, simConfig, Agent.ExplorationState.Initial);
        this.agent = agent;
        this.frontierExpType = frontierType;
        this.baseStation = baseStation;
        this.noReturnTimer = 0;
        this.frontiers = new PriorityQueue<>();
    }

    @Override
    public Point takeStep(int timeElapsed) {
        Point nextStep;

        //Preprocessing
        if (agent.getEnvError()) {
            agent.setEnvError(false);
            agent.setExploreState(Agent.ExplorationState.EnvError);
        }
        if (agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()) {
            agent.getStats().setTimeLastDirectContactCS(1);
            agent.getStats().setLastContactAreaKnown(agent.getStats().getAreaKnown());
        } else {
            agent.getStats().incrementLastDirectContactCS();
        }

        //Statemachine
        switch (agent.getExploreState()) {
            case Initial:
                nextStep = RandomWalk.randomStep(agent);
                agent.getStats().setTimeSinceLastPlan(0);
                if (timeElapsed >= SimConstants.INIT_CYCLES - 1) {
                    agent.setExploreState(Agent.ExplorationState.Explore);
                }
                break;
            case Explore:
                if ((agent.getStats().getTimeSinceLastPlan() < SimConstants.REPLAN_INTERVAL)
                        && agent.getPath() != null && !agent.getPath().isFinished() && agent.getPath().found && agent.getPath().getPoints().size() >= 2) {
                    nextStep = agent.getPath().nextPoint();
                } else {
                    nextStep = replan(timeElapsed);
                }
                break;
            case ReturnToBase:
            case Finished:
            case SettingRelay:
            case EnvError:
                agent.setExploreState(Agent.ExplorationState.Explore);
            default:
                nextStep = RandomWalk.randomStep(agent);

        }

        //postprocessing
        agent.getStats().incrementTimeSinceLastPlan();
        if (agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()) {
            noReturnTimer = 0;
        } else {
            noReturnTimer++;
        }
        return nextStep;
    }

    @Override
    protected Point replan(int timeElapsed) {
        Point nextStep;

        agent.getStats().setTimeSinceLastPlan(0);

        if (frontierExpType.equals(SimulatorConfig.frontiertype.PeriodicReturn) && noReturnTimer > simConfig.PERIODIC_RETURN_PERIOD) {
            agent.setPathToBaseStation();
            nextStep = agent.getPath().nextPoint();
            return nextStep;
        }
        if (frontierExpType.equals(SimulatorConfig.frontiertype.UtilReturn)) {
            double infoRatio = (double) agent.getStats().getCurrentBaseKnowledgeBelief()
                    / (double) (agent.getStats().getCurrentBaseKnowledgeBelief() + agent.getStats().getNewInfo());
            if (infoRatio < simConfig.TARGET_INFO_RATIO) {
                agent.setPathToBaseStation();
                nextStep = agent.getPath().nextPoint();
                return nextStep;
            }
        }
        calculateFrontiers();
//        frontiers.forEach(f -> System.out.println(f));
        //If no frontiers found, or reached exploration goal, return to ComStation
        if (((frontiers.isEmpty()) || no_change_counter > 20 || (agent.getStats().getPercentageKnown() >= SimConstants.TERRITORY_PERCENT_EXPLORED_GOAL))) {
            agent.setMissionComplete(true);
            agent.setPathToBaseStation();
            nextStep = agent.getPath().nextPoint();
            while (agent.getPath().isFinished() && (nextStep != null) && (nextStep.equals(agent.getLocation()))) {
                nextStep = agent.getPath().nextPoint();
            }
            return nextStep;
        } else {
            if (last_percentage_known == agent.getStats().getPercentageKnown()) {
                no_change_counter++;
            } else {
                no_change_counter = 0;
            }
            last_percentage_known = agent.getStats().getPercentageKnown();
        }

        FrontierUtility best;
        best = chooseFrontier(true);
        //If could not find frontier, try to disregard other agents when planning
        if (best == null) {
            best = chooseFrontier(false);
        }

        if (best != null) {
            agent.setFrontier(best.getFrontier());
            agent.setPath(best.getPath());
        }

        //If no frontier could be assigned, then go back to base.">
        if (best == null) {
            // mission complete
            if (SimConstants.DEBUG_OUTPUT) {
                System.out.println(agent.toString() + " could not find frontier, proceeding to BaseStation (Mission Complete).");
            }
            agent.setMissionComplete(true);
            agent.setPathToBaseStation();
            nextStep = agent.getPath().nextPoint();
            while ((nextStep != null) && (nextStep.equals(agent.getLocation()))) {
                nextStep = agent.getPath().nextPoint();
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
        nextStep = agent.getPath().nextPoint();
        return nextStep;
    }

    /**
     * Filter frontiers to remove to small frontiers and frontiers not reachable from basestation
     * (don't know why). Unreachable frontiers well be saved in badFrontiers
     *
     * @param lastFrontier current goal-frontier to consider it in any case, otherwise agent might
     * oszillate
     * @param grid current agents occupancygrid to see if a frontier is still interresting because
     * next to unknown
     */
    private void frontiersOfInterest(Frontier lastFrontier, OccupancyGrid grid) {
        PriorityQueue<Frontier> list = new PriorityQueue<>();

        int counter = 0;
        for (Frontier currFrontier : frontiers) {
            if (counter >= SimConstants.MAX_NUM_FRONTIERS) {
                break;
            }
            // To avoid oscillation, add last frontier to list (just in case it
            // still is the best, but is not one of the closest)
            if (currFrontier == lastFrontier) {
                counter++;
                continue;
            }
            if (currFrontier.getArea() >= SimConstants.MIN_FRONTIER_SIZE
                    && currFrontier.hasUnknownBoundary(grid)) {//TODO Unneccessary!?!?
                //ignore frontiers not reachable from base //TODO WHY??? Woudn't path from agent be better?
                /*Path pathToFrontier = agent.calculatePath(agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID).getLocation(),
                        currFrontier.getCentre(), false);*/
                Path pathToFrontier = agent.calculatePath(agent.getLocation(), currFrontier.getCentre(), false);
                if (!pathToFrontier.found) {
                    agent.addBadFrontier(currFrontier);
                } else {
                    list.add(currFrontier);
                    counter++;
                }
                // no need to break here, as we still want to add last frontier
                // if we haven't iterated through it yet.
            }

        }

        frontiers = list;
    }

    /**
     * Calculates Euclidean distance from all known teammates and self to frontiers of interest
     */
    private PriorityQueue initializeUtilities(boolean considerOtherAgents) {
        PriorityQueue<FrontierUtility> utilities = new PriorityQueue<FrontierUtility>();

        // For each frontier of interest
        for (Frontier frontier : frontiers) {
            // Add own utilities
            utilities.add(new FrontierUtility(agent, frontier));
            // Add teammates' utilities
            for (TeammateAgent teammate : agent.getAllTeammates().values()) {
                if (considerOtherAgents
                        && teammate.isExplorer()
                        && teammate.getTimeSinceLastComm() < SimConstants.REMEMBER_TEAMMATE_FRONTIER_PERIOD) {

                    utilities.add(new FrontierUtility(teammate,
                            frontier));
                }
            }
        }

        return utilities;
    }

    protected FrontierUtility chooseFrontier(boolean considerOtherAgents) {
        // Step 1:  Create list of frontiers of interest
        frontiersOfInterest(agent.getFrontier(), agent.getOccupancyGrid());
        // Step 2:  Create utility estimates
        PriorityQueue<FrontierUtility> utilities = initializeUtilities(considerOtherAgents);
        // Step 3
        FrontierUtility best;

        ArrayList<FrontierUtility> badUtilities = new ArrayList<>();
        Iterator<FrontierUtility> util_iter = utilities.iterator();
        while (util_iter.hasNext()) {
            best = util_iter.next();
            if (badUtilities.contains(best)) {
                util_iter.remove();
                continue;
            }
            // Check if this is the only remaining frontier would be a little optimization

            // /it's not possible to plan a path to this frontier or we are already there, so eliminate it entirely
            if (best.getEcaxtUtility(agent) < 0) {
                badUtilities.add(best);
                for (FrontierUtility u : utilities) {
                    if (u.getFrontier() == best.getFrontier()) {
                        badUtilities.add(u);
                    }
                }
                if (best.getAgent() == agent) {
                    agent.addBadFrontier(best.getFrontier()); //only add bad frontiers if they are 'ours'
                }

            } else if (utilities.isEmpty() || (best.getUtility() >= utilities.peek().getUtility())) {
                //as the utility estimate is an optimistic heuristic if the best.utility is better as the peek-utility... go and get it
                if (!considerOtherAgents || best.getAgent() == agent) {
                    return best;
                } else {
                    // This robot is assigned to this frontier, so remove all remaining associated utilities (for the agent and for the frontier)
                    badUtilities.add(best);
                    for (FrontierUtility u : utilities) {
                        if (u.getAgent() == best.getAgent() || u.getFrontier() == best.getFrontier()) {
                            badUtilities.add(u);
                        }
                    }
                }
            }

        }
        utilities.removeAll(badUtilities);

        return utilities.peek();  // give the best Utility
    }

    /**
     * Calculates the frontiers and stores them in the member 'frontiers'
     */
    protected void calculateFrontiers() {
        // If recalculating frontiers, must set old frontiers dirty for image rendering
        frontiers.stream().forEach((f) -> {
            agent.addDirtyCells(f.getPolygonOutline());
        });

        // 1. Find all Contours
        LinkedList<LinkedList<Point>> contours = ContourTracer.findAllContours(agent.getOccupancyGrid());
        //System.out.println(contours.size());
        //ContourTracer.mergeContours(contours);
        //System.out.println(contours.size());

        frontiers = new PriorityQueue();
        Frontier currFrontier;

        int contourCounter = 0;
        int contoursSmall = 0;
        int contoursBad = 0;

        //2. Make the Contours to Frontiers
        for (LinkedList<Point> currContour : contours) {
            currFrontier = new Frontier(agent.getX(), agent.getY(), currContour);
            //badFrontiers is a list the Agent keeps with Frontiers he already marked as unreachable
            if (!agent.isBadFrontier(currFrontier)) {
                // only consider frontiers that are big enough and reachable
                if (currFrontier.getArea() >= SimConstants.MIN_FRONTIER_SIZE) {
                    {
                        frontiers.add(currFrontier);
                        contourCounter++;
                    }
                } else {
                    contoursSmall++;
                }
            } else {
                contoursBad++;
            }
        }

        if (SimConstants.DEBUG_OUTPUT) {
            System.out.println("retained " + contourCounter + " of them, disregarded due to size " + contoursSmall
                    + ", disregarded as bad " + contoursBad + ".");
        }
    }

    /**
     * Returns the frontiers for display purpose only.
     *
     * @return list of frontiers
     */
    public PriorityQueue<Frontier> getFrontiers() {
        return frontiers;
    }
}
