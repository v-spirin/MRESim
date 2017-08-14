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
import environment.ContourTracer;
import environment.Frontier;
import environment.OccupancyGrid;
import exploration.Frontier.FrontierUtility;
import exploration.rendezvous.RendezvousAgentData;
import java.awt.Point;
import java.util.HashMap;
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
        super(agent, simConfig, ExplorationState.Initial);
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
        super(agent, simConfig, ExplorationState.Initial);
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
            state = ExplorationState.EnvError;
        }
        if (agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()) {
            agent.getStats().setTimeLastDirectContactCS(1);
            agent.getStats().setLastContactAreaKnown(agent.getStats().getAreaKnown());
        } else {
            agent.getStats().incrementLastDirectContactCS();
        }

        //Statemachine
        switch (state) {
            case Initial:
                nextStep = RandomWalk.randomStep(agent);
                agent.getStats().setTimeSinceLastPlan(0);
                if (timeElapsed >= Constants.INIT_CYCLES - 1) {
                    state = ExplorationState.Exploring;
                }
                break;
            case Exploring:
                if ((agent.getStats().getTimeSinceLastPlan() < Constants.REPLAN_INTERVAL)
                        && agent.getPath() != null && agent.getPath().found && agent.getPath().getPoints().size() >= 2) {
                    nextStep = agent.getNextPathPoint();
                } else {
                    nextStep = replan(timeElapsed);
                }
                break;
            case BackToBase:
            case Finished:
            case SettingRelay:
            case EnvError:
                state = ExplorationState.Exploring;
            default:
                nextStep = RandomWalk.randomStep(agent);

        }

        //postprocessing
        agent.getStats().incrementTimeSinceLastPlan();
        if (agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).hasCommunicationLink()) {
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
            nextStep = agent.getNextPathPoint();
            return nextStep;
        }
        if (frontierExpType.equals(SimulatorConfig.frontiertype.UtilReturn)) {
            double infoRatio = (double) agent.getStats().getCurrentBaseKnowledgeBelief()
                    / (double) (agent.getStats().getCurrentBaseKnowledgeBelief() + agent.getStats().getNewInfo());

            System.out.println(agent.toString() + " in state Explore. infoRatio = "
                    + infoRatio + ", Target = " + simConfig.TARGET_INFO_RATIO + ". newInfo = " + agent.getStats().getNewInfo()
                    + ", baseInfo = " + agent.getStats().getCurrentBaseKnowledgeBelief());
            if (infoRatio < simConfig.TARGET_INFO_RATIO) {
                agent.setPathToBaseStation();
                nextStep = agent.getNextPathPoint();
                return nextStep;
            }
        }
        calculateFrontiers();

        //If no frontiers found, or reached exploration goal, return to ComStation
        if (((frontiers.isEmpty()) || no_change_counter > 20 || (agent.getStats().getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))) {
            agent.setMissionComplete(true);
            agent.setPathToBaseStation();
            nextStep = agent.getNextPathPoint();
            while ((nextStep != null) && (nextStep.equals(agent.getLocation()))) {
                nextStep = agent.getNextPathPoint();
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

        long realtimeStart = System.currentTimeMillis();
        boolean foundFrontier = false;

        if (!agent.getSimConfig().keepAssigningRobotsToFrontiers()) {
            foundFrontier = (chooseFrontier(true, null) == null);
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent.toString() + "chooseFrontier took " + (System.currentTimeMillis() - realtimeStart) + "ms.");
            }

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

        //If no frontier could be assigned, then go back to base.">
        if (!foundFrontier) {
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
            if (counter >= Constants.MAX_NUM_FRONTIERS) {
                break;
            }
            // To avoid oscillation, add last frontier to list (just in case it
            // still is the best, but is not one of the closest)
            if (currFrontier == lastFrontier) {
                counter++;
                continue;
            }
            if (currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE
                    && currFrontier.hasUnknownBoundary(grid)) {//TODO Unneccessary!?!?
                //ignore frontiers not reachable from base //TODO WHY??? Woudn't path from agent be better?
                Path pathToFrontier = agent.calculatePath(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation(),
                        currFrontier.getCentre(), false);
                if (!pathToFrontier.found) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(agent + "adding bad frontier " + currFrontier);
                    }
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

    private double utilityEstimate(Point agentLoc, Frontier frontier) {
        if (agentLoc.getX() == frontier.getCentre().x
                && agentLoc.getY() == frontier.getCentre().y) {
            return -1001;
        }
        return ((frontier.getArea() * 100000000) / Math.pow(agentLoc.distance(frontier.getCentre()), 4));
    }

    private void calculateUtilityExact(FrontierUtility ute) {
        if (agent.getLocation().getX() == ute.frontier.getCentre().x
                && agent.getLocation().getY() == ute.frontier.getCentre().y) {
            ute.utility = -1001;
            return;
        }
        Point start;
        boolean isMe = (ute.agentID == agent.getID());
        if (isMe) {
            start = agent.getLocation();
        } else {
            start = agent.getTeammate(ute.agentID).getLocation();
        }
        Path p;
        {
            p = agent.calculatePath(start, ute.frontier.getCentre(), false/*ute.frontier.getClosestPoint(start, agent.getOccupancyGrid())*/);
        }

        if (p.found) {
            ute.path = p;
            ute.utility = (ute.frontier.getArea() * 100000000) / Math.pow(p.getLength(), 4);
            if (Constants.AVOID_FRONTIERS_WE_CANNOT_REACH_IN_TIME && isMe) {
                //calculate how much time we have left if we are in Role-Based exp.
                Point RVDestination = null;
                int timeMeeting = Integer.MAX_VALUE;
                RendezvousAgentData rvd = agent.getRendezvousAgentData();
                Path frontierToRV = null;
                if (agent.getRole() == RobotConfig.roletype.Explorer) {
                    RVDestination = rvd.getParentRendezvous().getChildLocation();
                    if (RVDestination != null) {
                        frontierToRV = agent.calculatePath(ute.frontier.getCentre(), RVDestination, false);
                    }
                    timeMeeting = rvd.getParentRendezvous().getTimeMeeting();
                } else if (agent.getRole() == RobotConfig.roletype.Relay) {
                    RVDestination = rvd.getChildRendezvous().getParentLocation();
                    if (RVDestination != null) {
                        Path frontierToParent = agent.calculatePath(ute.frontier.getCentre(), RVDestination, false);
                        Path frontierToChild = agent.calculatePath(ute.frontier.getCentre(), rvd.getChildRendezvous().getChildLocation(), false);
                        if (frontierToParent.getLength() < frontierToChild.getLength() && frontierToParent.found) {
                            frontierToRV = frontierToParent;
                        } else if (frontierToParent.getLength() >= frontierToChild.getLength() && frontierToChild.found) {
                            frontierToRV = frontierToChild;
                        } else {
                            frontierToRV = frontierToParent;
                        }
                    }
                    timeMeeting = rvd.getChildRendezvous().getTimeMeeting();
                }
                if (RVDestination != null && frontierToRV != null) {
                    Path meToFrontier = p;

                    int timeToRV = (int) ((meToFrontier.getLength() + frontierToRV.getLength()) / Constants.DEFAULT_SPEED) + agent.getTimeElapsed();
                    //if time available is less than time to frontier, set utility to low value
                    if (timeToRV > timeMeeting) {
                        if (Constants.DEBUG_OUTPUT) {
                            System.out.println(agent + "Cannot explore frontier with centre " + ute.frontier.getCentre() + ", timeToRV is " + timeToRV + ", timeMeeting is " + timeMeeting + ", utility was " + ute.utility + ", setting utility to " + (ute.utility - 100000000));
                        }
                        ute.utility = ute.utility - 100000000;
                        if (agent.getRole() == RobotConfig.roletype.Relay) {
                            ute.utility = -1;
                        }
                    }
                }

            }
        } else {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Could not find path from " + start + " to " + ute.frontier.getCentre());
            }
            ute.utility = -1000;
        }

    }

    // Calculates Euclidean distance from all known teammates and self to frontiers of interest
    private PriorityQueue initializeUtilities(boolean considerOtherAgents, LinkedList<Integer> teammatesAssignedIDs) {
        if (teammatesAssignedIDs == null) {
            teammatesAssignedIDs = new LinkedList<Integer>();
        }
        PriorityQueue<FrontierUtility> utilities = new PriorityQueue<FrontierUtility>();

        int lastCommLimit = Constants.REMEMBER_TEAMMATE_FRONTIER_PERIOD;
        if (!considerOtherAgents) {
            lastCommLimit = -1;
        }

        // For each frontier of interest
        for (Frontier frontier : frontiers) {
            // Add own utilities
            utilities.add(new FrontierUtility(agent.getID(),
                    agent.getLocation(),
                    frontier,
                    utilityEstimate(agent.getLocation(), frontier),
                    null));
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(Constants.INDENT + "Own utility with ID "
                        + agent.getID() + " for frontier at "
                        + frontier.getCentre().x + ","
                        + frontier.getCentre().y + " is "
                        + (int) utilityEstimate(agent.getLocation(), frontier));
            }
            // Add teammates' utilities
            for (TeammateAgent teammate : agent.getAllTeammates().values()) {
                if (teammate.getID() != 1
                        && teammate.getID() != agent.getID()
                        && teammate.isExplorer()
                        && // THIS LINE ONLY IF non-explorer agents are to be ignored
                        teammate.getTimeSinceLastComm() < lastCommLimit
                        && !teammatesAssignedIDs.contains(teammate.getID())) {
                    utilities.add(new FrontierUtility(teammate.getID(),
                            teammate.getLocation(),
                            frontier,
                            utilityEstimate(teammate.getLocation(), frontier),
                            null));
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(Constants.INDENT + "Utility of robot with ID "
                                + teammate.getID() + " at location "
                                + "(" + (int) teammate.getLocation().getX() + ","
                                + (int) teammate.getLocation().getY()
                                + ") for frontier at "
                                + frontier.getCentre().x + ","
                                + frontier.getCentre().y + " is "
                                + (int) utilityEstimate(teammate.getLocation(), frontier));
                    }
                }
            }
        }

        return utilities;
    }

    /**
     * Calculates Euclidean distance from all known teammates and self to frontiers of interest
     *
     * @param p
     */
    private PriorityQueue initializeUtilities(Point p) {
        PriorityQueue<FrontierUtility> utilities = new PriorityQueue<FrontierUtility>();

        // For each frontier of interest
        frontiers.stream().forEach((frontier) -> {
            // Add own utilities
            utilities.add(new FrontierUtility(99,
                    p,
                    frontier,
                    utilityEstimate(p, frontier),
                    null));
        });

        return utilities;
    }

    protected LinkedList<Integer> chooseFrontier(boolean considerOtherAgents,
            LinkedList<Integer> teammatesAssignedIDs) {
        if (teammatesAssignedIDs == null) {
            teammatesAssignedIDs = new LinkedList<Integer>();
        }
        // Step 1:  Create list of frontiers of interest (closest ones)
        frontiersOfInterest(agent.getLastFrontier(), agent.getOccupancyGrid());
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontiers of interest: " + frontiers.size());
        }
        // Step 2:  Create priorityQueue of utility estimates (Euclidean distance)
        PriorityQueue<FrontierUtility> utilities = initializeUtilities(considerOtherAgents, teammatesAssignedIDs);
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontier utilities: " + utilities.size());
        }
        // Step 3
        FrontierUtility best;
        LinkedList<FrontierUtility> removal;
        boolean isLastFrontier = false;

        while (!utilities.isEmpty()) {    // && counter < 5) {
            best = utilities.poll();
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + best.toString());
            }
            // Check if this is the only remaining frontier
            isLastFrontier = true;
            if (!utilities.isEmpty()) {
                for (FrontierUtility u : utilities) {
                    if (u.frontier != best.frontier) {
                        isLastFrontier = false;
                        break;
                    }
                }
            }

            // If this is an estimate, calculate true utility
            if (best.path == null) {
                calculateUtilityExact(best);
            }

            if (best.path == null) {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(agent + " could not calculate exact utility: " + best + ", removing frontier: "
                            + best.frontier);
                }
                // it's not possible to plan a path to this frontier, so eliminate it entirely
                removal = new LinkedList<FrontierUtility>();
                for (FrontierUtility u : utilities) {
                    if (u.frontier == best.frontier) {
                        removal.add(u);
                    }
                }
                for (FrontierUtility r : removal) {
                    utilities.remove(r);
                }
                if (best.agentID == agent.getID()) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(agent + " adding bad frontier");
                    }
                    agent.addBadFrontier(best.frontier); //only add bad frontiers if they are 'ours'
                }

            } else if ((utilities.isEmpty()) || (best.utility >= utilities.peek().utility)) {
                if (best.agentID == agent.getID()) {
                    if ((agent.getRole() == RobotConfig.roletype.Relay) && (best.utility < 0)) {//cannot reach frontier in time
                        agent.setState(Agent.AgentState.GoToChild);
                        return null;
                    }
                    agent.setLastFrontier(best.frontier);
                    agent.setPath(best.path);
                    return null;
                } else {
                    // This robot assigned, so remove all remaining associated utilities
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(agent + "This robot assigned, so remove all remaining associated utilities");
                    }
                    removal = new LinkedList<FrontierUtility>();
                    for (FrontierUtility u : utilities) {
                        if (u.agentID == best.agentID
                                || u.frontier == best.frontier) {
                            removal.add(u);
                        }
                    }
                    for (FrontierUtility r : removal) {
                        utilities.remove(r);
                    }
                    teammatesAssignedIDs.add(best.agentID);
                }
            } else {
                utilities.add(best);
            }

        }

        return teammatesAssignedIDs;  // couldn't assign frontier - could be there are more robots than frontiers?
    }

    protected FrontierUtility chooseFrontier() {
        // Step 1:  Create list of frontiers of interest (closest ones)
        frontiersOfInterest(agent.getLastFrontier(), agent.getOccupancyGrid());
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontiers of interest: " + frontiers.size());
        }
        // Step 2:  Create priorityQueue of utility estimates (Euclidean distance)
        PriorityQueue<FrontierUtility> utilities = initializeUtilities(false, new LinkedList<Integer>());
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontier utilities: " + utilities.size());
        }
        // Step 3
        FrontierUtility best;
        LinkedList<FrontierUtility> removal;

        while (!utilities.isEmpty()) {
            best = utilities.poll();
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + best.toString());
            }
            // Check if this is the only remaining frontier
            if (!utilities.isEmpty()) {
                for (FrontierUtility u : utilities) {
                    if (u.frontier != best.frontier) {
                        break;
                    }
                }
            }

            // If this is an estimate, calculate true utility
            if (best.path == null) {
                calculateUtilityExact(best);
            }

            if (best.path == null) {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(agent + " could not calculate exact utility: " + best + ", removing frontier: "
                            + best.frontier);
                }
                // it's not possible to plan a path to this frontier, so eliminate it entirely
                removal = new LinkedList<FrontierUtility>();
                for (FrontierUtility u : utilities) {
                    if (u.frontier == best.frontier) {
                        removal.add(u);
                    }
                }
                for (FrontierUtility r : removal) {
                    utilities.remove(r);
                }
                if (best.agentID == agent.getID()) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(agent + " adding bad frontier");
                    }
                    agent.addBadFrontier(best.frontier); //only add bad frontiers if they are 'ours'
                }

            } else if ((utilities.isEmpty()) || (best.utility >= utilities.peek().utility)) {
                if (best.agentID == agent.getID()) {
                    return best;
                } else {
                    // This robot assigned, so remove all remaining associated utilities
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(agent + "This robot assigned, so remove all remaining associated utilities");
                    }
                    removal = new LinkedList<FrontierUtility>();
                    for (FrontierUtility u : utilities) {
                        if (u.agentID == best.agentID
                                || u.frontier == best.frontier) {
                            removal.add(u);
                        }
                    }
                    for (FrontierUtility r : removal) {
                        utilities.remove(r);
                    }
                }
            } else {
                utilities.add(best);
            }

        }

        return null;  // couldn't assign frontier
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
        LinkedList<LinkedList> contours = ContourTracer.findAllContours(agent.getOccupancyGrid());

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
                if (currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE) {
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

        if (Constants.DEBUG_OUTPUT) {
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
