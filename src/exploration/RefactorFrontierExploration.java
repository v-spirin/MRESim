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
import java.util.LinkedList;
import java.util.PriorityQueue;
import path.Path;

/**
 * Calculate Frontiers, weight them by utility and decide which one to explore first.
 *
 * @author julh, Christian Clausen
 */
public class RefactorFrontierExploration extends BasicExploration implements Exploration {

    SimulatorConfig.frontiertype frontierExpType;
    int PERIODIC_RETURN = 100;
    RealAgent baseStation;
    int noReturnTimer;
    Path path;

    public RefactorFrontierExploration(RealAgent agent, SimulatorConfig.frontiertype frontierExpType, RealAgent baseStation) {
        super(agent);
        this.agent = agent;
        this.frontierExpType = frontierExpType;
        this.baseStation = baseStation;
        this.noReturnTimer = 0;
    }

    @Override
    public Point takeStep(int timeElapsed) {
        if (frontierExpType == SimulatorConfig.frontiertype.UtilReturn) {
            throw new IllegalArgumentException("Frontier-UtilReturn is implemented in UtilExploration.java");
        }

        Point nextStep;

        if (agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInRange()) {
            agent.getStats().setTimeLastDirectContactCS(1);
            agent.getStats().setLastContactAreaKnown(agent.getStats().getAreaKnown());
        } else {
            agent.getStats().incrementLastDirectContactCS();
        }

        if (timeElapsed < Constants.INIT_CYCLES) {
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
        } else if (agent.getEnvError()) {
            agent.resetPathToBaseStation();
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setEnvError(false);
        } else if ((agent.getStats().getTimeSinceLastPlan() < Constants.REPLAN_INTERVAL)
                && agent.getPath().found && agent.getPath().getPoints().size() >= 2) {
            nextStep = agent.getNextPathPoint();
        } else if (agent.getStats().getTimeSinceLastPlan() >= Constants.REPLAN_INTERVAL) {
            nextStep = replan(timeElapsed);
        } else {
            nextStep = replan(timeElapsed);
        }

        agent.getStats().incrementTimeSinceLastPlan();

        return nextStep;
    }

    @Override
    protected Point replan(int timeElapsed) {
        Point nextStep;

        agent.getStats().setTimeSinceLastPlan(0);

        long realtimeStart = System.currentTimeMillis();

        calculateFrontiers(agent, frontierExpType);

        //If no frontiers found, or reached exploration goal, return to ComStation
        if (((agent.getFrontiers().isEmpty()) || (agent.getStats().getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))
                && timeElapsed > 100) {
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

        long realtimeStart2 = System.currentTimeMillis();
        boolean foundFrontier = false;

        if (!agent.getSimConfig().keepAssigningRobotsToFrontiers()) {
            foundFrontier = (chooseFrontier(true, null) == null);
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent.toString() + "chooseFrontier took " + (System.currentTimeMillis() - realtimeStart2) + "ms.");
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

        if ((agent.getRole() == RobotConfig.roletype.Relay) && (agent.getState() == Agent.ExploreState.GoToChild)) {
            return takeStep_GoToChild();
        }

        //If no frontier could be assigned, then go back to base.">
        if (!foundFrontier && timeElapsed > 100) {
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

        //If overlapping another agent, take random step
        for (TeammateAgent teammate : agent.getAllTeammates().values()) {
            if (agent.getLocation().equals(teammate.getLocation())) {
                nextStep = RandomWalk.takeStep(agent);
                agent.getStats().setTimeSinceLastPlan(0);
                agent.setCurrentGoal(nextStep);
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
            nextStep = RandomWalk.takeStep(agent);
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

// <editor-fold defaultstate="collapsed" desc="Choose best frontier">
    private LinkedList<Frontier> frontiersOfInterest(Frontier lastFrontier, PriorityQueue<Frontier> frontiers, OccupancyGrid grid) {
        PriorityQueue<Frontier> copy = new PriorityQueue();
        LinkedList<Frontier> list = new LinkedList();

        frontiers.stream().forEach((f) -> {
            copy.add(f.copy());
        });

        Frontier currFrontier;
        int counter = 0;
        for (int i = copy.size(); i > 0; i--) {
            currFrontier = copy.poll();

            // To avoid oscillation, add last frontier to list (just in case it
            // still is the best, but is not one of the closest)
            if (currFrontier == lastFrontier) {
                list.add(currFrontier);
                counter++;
                if (counter > Constants.MAX_NUM_FRONTIERS) {
                    break;
                }
            } else if (currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE
                    && currFrontier.hasUnknownBoundary(grid)
                    && counter < Constants.MAX_NUM_FRONTIERS) {
                //ignore frontiers not reachable from base
                Path pathToFrontier = agent.calculatePath(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation(),
                        currFrontier.getCentre(), false);
                if (/*(currFrontier.getArea() < Constants.MIN_FRONTIER_SIZE * 4) && */ //(grid.obstacleWithinDistance(currFrontier.getCentre().x, currFrontier.getCentre().y, 1)))
                        !pathToFrontier.found) {
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

        return list;
    }

    private static double utilityEstimate(Point agentLoc, Frontier frontier) {
        if (agentLoc.getX() == frontier.getCentre().x
                && agentLoc.getY() == frontier.getCentre().y) {
            return -1001;
        }
        return ((frontier.getArea() * 100000000) / Math.pow(agentLoc.distance(frontier.getCentre()), 4));
    }

    private static void calculateUtilityExact(RealAgent agent, FrontierUtility ute) {
        if (agent.getLocation().getX() == ute.frontier.getCentre().x
                && agent.getLocation().getY() == ute.frontier.getCentre().y) {
            ute.utility = -1001;
            return;
        }
        Point start;
        boolean isMe = (ute.ID == agent.getID());
        if (isMe) {
            start = agent.getLocation();
        } else {
            start = agent.getTeammate(ute.ID).getLocation();
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
    private static PriorityQueue initializeUtilities(RealAgent agent, LinkedList<Frontier> frontiers,
            boolean considerOtherAgents, LinkedList<Integer> teammatesAssignedIDs) {
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

    // Calculates Euclidean distance from all known teammates and self to frontiers of interest
    private static PriorityQueue initializeUtilities(Point p, PriorityQueue<Frontier> frontiers) {
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

    public Point takeStep_GoToChild() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //Check if we are in range of child. If yes, GetInfoFromChild
        if ((agent.getChildTeammate().isInRange()) /*&& !agent.getParentTeammate().isInRange()*/) {
            agent.setState(RealAgent.ExploreState.GetInfoFromChild);
            agent.setStateTimer(0);

            return takeStep_GetInfoFromChild();
        }

        //Assume that a path back to child has been calculated in previous state, recalculate every PATH_RECALC_CHILD_INTERVAL steps
        Path existingPath = agent.getPath();
        if (((agent.getStateTimer() % Constants.PATH_RECALC_CHILD_INTERVAL) == (Constants.PATH_RECALC_CHILD_INTERVAL - 1))) {
            if (existingPath != null) {
                agent.addDirtyCells(existingPath.getAllPathPixels());
            }

            Path _path = agent.getRendezvousStrategy().processGoToChildReplan();

            if (_path == null) {
                _path = agent.calculatePath(agent.getLocation(), rvd.getChildRendezvous().getParentLocation(), false);
            }
            //Could not find full path! Trying pure A*
            if (!_path.found) {
                System.out.println(agent.toString() + "!!!ERROR!  Could not find full path! Trying pure A*");
                _path = agent.calculatePath(agent.getLocation(), agent.getRendezvousAgentData().getChildRendezvous().getParentLocation(), true);
            }
            //Still couldn't find path, trying existing path or if that fails, taking random step
            if (!_path.found) {
                System.out.println(agent.toString() + "!!!ERROR!  Could not find full path!");
                if ((existingPath != null) && (existingPath.getPoints().size() > 2)) {
                    agent.setPath(existingPath);
                    agent.setCurrentGoal(existingPath.getGoalPoint());
                } else {
                    agent.setCurrentGoal(agent.getLocation());
                    return RandomWalk.takeStep(agent);
                }
            } else {
                agent.setPath(_path);
                agent.setCurrentGoal(rvd.getChildRendezvous().getParentLocation());
                // Must remove first point in path as this is robot's location.
                agent.getPath().getPoints().remove(0);
            }
        }

        if (agent.getPath().found && !agent.getPath().getPoints().isEmpty()) {
            return ((Point) agent.getPath().getPoints().remove(0));
        }

        // If we reach this point, we are not in range of the child and the
        // path is empty, so we must have reached rendezvous point.
        // Just check to make sure we are though and if not take random step.
        if ((agent.getLocation().distance(rvd.getChildRendezvous().getParentLocation()) > 2 * Constants.STEP_SIZE)
                && (agent.getLocation().distance(rvd.getChildRendezvous().getChildLocation()) > 2 * Constants.STEP_SIZE)) {
            System.out.println(agent.toString()
                    + "!!!ERROR! We should have reached child RV, but we are too far from it! Taking random step");
            return RandomWalk.takeStep(agent);
        }

        // If we reach this point, we're at the rendezvous point and waiting.
        agent.setState(RealAgent.ExploreState.WaitForChild);
        agent.setStateTimer(0);

        return agent.getLocation();
    }

    public Point takeStep_GetInfoFromChild() {
        //we've exchanged info, now return to parent (but wait for one timestep to learn new RV point)

        if (agent.getStateTimer() == 0) {
            agent.addDirtyCells(agent.getPath().getAllPathPixels());
            Path _path = agent.calculatePath(agent.getLocation(), agent.getRendezvousAgentData().getParentRendezvous().getChildLocation(), false);
            agent.setPath(_path);
            agent.setStateTimer(1);
            agent.setCurrentGoal(agent.getRendezvousAgentData().getParentRendezvous().getChildLocation());
            return agent.getLocation();
        } else {
            if (agent.getChildTeammate().getState() == Agent.ExploreState.GiveParentInfo) {
                agent.setTimeSinceGetChildInfo(0);
            }
            agent.setState(RealAgent.ExploreState.ReturnToParent);
            return agent.getLocation();
        }
    }

    public static double maxFrontierUtility(PriorityQueue<Frontier> frontiers, OccupancyGrid grid, Point start) {
        PriorityQueue<FrontierUtility> utilities = initializeUtilities(start, frontiers);

        return utilities.peek().utility;
    }

    public LinkedList<Integer> chooseFrontier(boolean considerOtherAgents,
            LinkedList<Integer> teammatesAssignedIDs) {
        if (teammatesAssignedIDs == null) {
            teammatesAssignedIDs = new LinkedList<Integer>();
        }
        // Step 1:  Create list of frontiers of interest (closest ones)
        LinkedList<Frontier> frontiers = frontiersOfInterest(agent.getLastFrontier(), agent.getFrontiers(), agent.getOccupancyGrid());
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontiers of interest: " + frontiers.size());
        }
        // Step 2:  Create priorityQueue of utility estimates (Euclidean distance)
        PriorityQueue<FrontierUtility> utilities = initializeUtilities(agent, frontiers, considerOtherAgents, teammatesAssignedIDs);
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontier utilities: " + utilities.size());
        }
        // Step 3
        FrontierUtility best;
        LinkedList<FrontierUtility> removal;
        boolean isLastFrontier = false;
        int counter = 0;

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
                calculateUtilityExact(agent, best);
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
                if (best.ID == agent.getID()) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(agent + " adding bad frontier");
                    }
                    agent.addBadFrontier(best.frontier); //only add bad frontiers if they are 'ours'
                }

            } else //System.out.println("UtilityExact: " + best.utility);
             if ((utilities.isEmpty()) || (best.utility >= utilities.peek().utility)) {
                    if (best.ID == agent.getID()) {
                        if ((agent.getRole() == RobotConfig.roletype.Relay) && (best.utility < 0)) {//cannot reach frontier in time
                            agent.setState(Agent.ExploreState.GoToChild);
                            return null;
                        }
                        agent.setLastFrontier(best.frontier);
                        agent.setCurrentGoal(best.frontier.getCentre());
                        if (agent.getPath() != null) {
                            agent.addDirtyCells(agent.getPath().getAllPathPixels());
                        }
                        agent.setPath(best.path);
                        return null;
                    } else {
                        // This robot assigned, so remove all remaining associated utilities
                        if (Constants.DEBUG_OUTPUT) {
                            System.out.println(agent + "This robot assigned, so remove all remaining associated utilities");
                        }
                        removal = new LinkedList<FrontierUtility>();
                        for (FrontierUtility u : utilities) {
                            if (u.ID == best.ID
                                    || u.frontier == best.frontier) {
                                removal.add(u);
                            }
                        }
                        for (FrontierUtility r : removal) {
                            utilities.remove(r);
                        }
                        teammatesAssignedIDs.add(best.ID);
                    }
                } else {
                    utilities.add(best);
                }

            counter++;
        }

        return teammatesAssignedIDs;  // couldn't assign frontier - could be there are more robots than frontiers?
    }

//Calculate Frontiers
    public static void calculateFrontiers(RealAgent agent, SimulatorConfig.frontiertype frontierExpType) {
        long realtimeStart = System.currentTimeMillis();
        // If recalculating frontiers, must set old frontiers dirty for image rendering
        agent.getFrontiers().stream().forEach((f) -> {
            agent.addDirtyCells(f.getPolygonOutline());
        });

        LinkedList<LinkedList> contours = ContourTracer.findAllContours(agent.getOccupancyGrid());
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + "Found " + contours.size() + " contours, took " + (System.currentTimeMillis() - realtimeStart) + "ms.");
        }
        realtimeStart = System.currentTimeMillis();
        PriorityQueue<Frontier> frontiers = new PriorityQueue();
        Frontier currFrontier;

        int contourCounter = 0;
        int contoursSmall = 0;
        int contoursBad = 0;
        for (LinkedList<Point> currContour : contours) {
            currFrontier = new Frontier(agent.getX(), agent.getY(), currContour);
            if (!agent.isBadFrontier(currFrontier)) {
                // only consider frontiers that are big enough and reachable
                if (currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE) {
                    //if (!agent.getOccupancyGrid().safeSpaceAt(currFrontier.getCentre().x, currFrontier.getCentre().y))
                    {
                        frontiers.add(currFrontier);
                        contourCounter++;
                    }
                } else {
                    contoursSmall++;
                    //System.out.println("Disregarding a contour as it's smaller than min_frontier_size which is " + Constants.MIN_FRONTIER_SIZE);
                }
            } else {
                contoursBad++;
            }
        }
        agent.setFrontiers(frontiers);

        if (Constants.DEBUG_OUTPUT) {
            System.out.println("retained " + contourCounter + " of them, disregarded due to size " + contoursSmall
                    + ", disregarded as bad " + contoursBad + ". Took " + (System.currentTimeMillis() - realtimeStart) + "ms.");
        }
        //System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
}
