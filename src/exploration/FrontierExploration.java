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

import agents.*;
import config.Constants;
import config.RobotConfig;
import config.SimulatorConfig;
import environment.*;
import exploration.rendezvous.RendezvousAgentData;
import java.util.*;
import java.awt.*;
import path.Path;

/**
 *
 * @author julh
 */

public class FrontierExploration {
    
    // Returns new X, Y of RealAgent
    public static Point takeStep(RealAgent agent, int timeElapsed, 
            SimulatorConfig.frontiertype frontierExpType) {
        long realtimeStartAgentCycle = System.currentTimeMillis();
        
        Point nextStep = new Point(agent.getX(), agent.getY());

        //<editor-fold defaultstate="collapsed" desc="If base station is in range, update timeLastDirectContactCS and lastContactAreaKnown">
        if(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInRange()) {
            agent.getStats().setTimeLastDirectContactCS(1);
            agent.getStats().setLastContactAreaKnown(agent.getStats().getAreaKnown());
        }
        else
            agent.getStats().incrementLastDirectContactCS();
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="CHECK 0: Take a couple of random steps to start (just to gather some sensor data).">
        if(timeElapsed < Constants.INIT_CYCLES) {
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
        }
        //</editor-fold>
 
        //<editor-fold defaultstate="collapsed" desc="CHECK 1: if agent hasn't moved, then he may be stuck in front of a wall.  Taking a random step might help.">
        else if(agent.getEnvError()) {
            agent.resetPathToBaseStation();
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setEnvError(false);
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="CHECK 2: Agent isn't stuck, not yet time to replan. Continue if we have points left in the previously planned path.">
        else if ((agent.getStats().getTimeSinceLastPlan() < Constants.REPLAN_INTERVAL)
                && agent.getPath().found && agent.getPath().getPoints().size() >= 2)
        {
            nextStep = agent.getNextPathPoint();
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Agent " + agent + " continuing on path.");
            }
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="CHECK 3: Agent isn't stuck. Is it time to replan? Have we moved on to next time cycle?">
        else if(agent.getStats().getTimeSinceLastPlan() >= Constants.REPLAN_INTERVAL)
        {
            nextStep = replan(agent, frontierExpType, timeElapsed);
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="CHECK 4: Agent isn't stuck, not yet time to replan, but we have no points left">
        else
        {
            nextStep = replan(agent, frontierExpType, timeElapsed);
        }
        //</editor-fold>

        agent.getStats().incrementTimeSinceLastPlan();

        //agent.setLastTotalKnowledgeBelief(agent.getCurrentTotalKnowledgeBelief());
        //agent.setLastBaseKnowledgeBelief(agent.getCurrentBaseKnowledgeBelief());
        //agent.setLastNewInfo(agent.getNewInfo());

        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent.toString() + "takeStep took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
        }
        return nextStep;
    }   

    public static Point replan(RealAgent agent, SimulatorConfig.frontiertype frontierExpType, int timeElapsed) {
        Point nextStep;
        
        agent.getStats().setTimeSinceLastPlan(0);
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent.toString() + "starting replan");
        }
        long realtimeStart = System.currentTimeMillis();
                     
        calculateFrontiers(agent, frontierExpType);
        
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent.toString() + "calculateFrontiers took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        }

        //<editor-fold defaultstate="collapsed" desc="If no frontiers found, or reached exploration goal, return to ComStation">
        if (((agent.getFrontiers().isEmpty()) || (agent.getStats().getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))
                && timeElapsed > 100){
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + " setting mission complete");
            }
            agent.setMissionComplete(true);
            agent.setPathToBaseStation();
            nextStep = agent.getNextPathPoint();
            while ((nextStep != null) && (nextStep.equals(agent.getLocation())))
            {
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
            foundFrontier = (chooseFrontier(agent, true, null) == null);
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent.toString() + "chooseFrontier took " + (System.currentTimeMillis()-realtimeStart2) + "ms.");
            }

            //<editor-fold defaultstate="collapsed" desc="If could not find frontier, try to disregard other agents when planning">
            if (!foundFrontier)
            {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(agent.toString() + " could not find frontier, trying to ignore other agents...");
                }
                foundFrontier = (chooseFrontier(agent, false, null) == null);
            }
            //</editor-fold>
        } else {
            LinkedList<Integer> assignedTeammates = new LinkedList<Integer>();
            for (int i = 0; (i < agent.getAllTeammates().size()) && !foundFrontier; i++) {
                assignedTeammates = chooseFrontier(agent, true, assignedTeammates);
                if (assignedTeammates == null)
                    foundFrontier = true;
            }
        }
        
        if ((agent.getRole() == RobotConfig.roletype.Relay) && (agent.getState() == BasicAgent.ExploreState.GoToChild)) {
            return RoleBasedExploration.takeStep_GoToChild(agent);
        }

        //<editor-fold defaultstate="collapsed" desc="If no frontier could be assigned, then go back to base.">
        if(!foundFrontier && timeElapsed > 100){
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
            while ((nextStep != null) && (nextStep.equals(agent.getLocation())))
            {
                nextStep = agent.getNextPathPoint();
            }
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setCurrentGoal(agent.getTeammate(1).getLocation());
            return nextStep;
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="If overlapping another agent, take random step">
        for(TeammateAgent teammate : agent.getAllTeammates().values())
        {
            if (agent.getLocation().equals(teammate.getLocation()))
            {
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
        if ((agent.getPath() == null) || 
                agent.getPath().getPoints() == null ||
                agent.getPath().getPoints().isEmpty() ||
                agent.getPath().getPoints().size() == 1)
        {
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
            System.out.println(agent.toString() + "replan took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        }
        return nextStep;
    }   

// <editor-fold defaultstate="collapsed" desc="Choose best frontier">
    private static LinkedList<Frontier> frontiersOfInterest (RealAgent agent, 
            Frontier lastFrontier, PriorityQueue<Frontier> frontiers, OccupancyGrid grid) {
        PriorityQueue<Frontier> copy = new PriorityQueue();
        LinkedList<Frontier> list = new LinkedList();

        for(Frontier f : frontiers)
            copy.add(f.copy());
        
        Frontier currFrontier;
        int counter = 0;
        for(int i=copy.size(); i>0; i--) {
            currFrontier = copy.poll();

            // To avoid oscillation, add last frontier to list (just in case it
            // still is the best, but is not one of the closest)
            if (currFrontier == lastFrontier) {
                list.add(currFrontier);
                counter++;
                if(counter > Constants.MAX_NUM_FRONTIERS)
                    break;
            }
            else if(currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE &&
               currFrontier.hasUnknownBoundary(grid) &&
               counter < Constants.MAX_NUM_FRONTIERS) 
            {
                //ignore frontiers not reachable from base
                Path pathToFrontier = agent.calculatePath(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation(), 
                        currFrontier.getCentre());
                if (/*(currFrontier.getArea() < Constants.MIN_FRONTIER_SIZE * 4) && */
                       //(grid.obstacleWithinDistance(currFrontier.getCentre().x, currFrontier.getCentre().y, 1)))
                        !pathToFrontier.found)
                {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(agent + "adding bad frontier " + currFrontier);
                    }
                    agent.addBadFrontier(currFrontier);
                } else
                {
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
        return ((frontier.getArea() * 100000000) / Math.pow(agentLoc.distance(frontier.getCentre()), 4) );
    }
    
    private static void calculateUtilityExact(RealAgent agent, Utility ute) {
        Point start;
        boolean isMe = (ute.ID == agent.getID());
        if(isMe)
            start = agent.getLocation();
        else
            start = agent.getTeammate(ute.ID).getLocation();

        // old method:
        //Path p = new Path(agent, start, ute.frontier.getCentre());        
        Path p;
        /*if (ute.frontier.getClosestPoint(start, agent.getOccupancyGrid()).x == 0)
        {
            //System.out.println("Closest point is 0, this shouldn't happen");
            p = new Path();
            p.found = false;
        } else*/
        {
            p = agent.calculatePath(start, ute.frontier.getCentre()/*ute.frontier.getClosestPoint(start, agent.getOccupancyGrid())*/);
        }

        if(p.found) {
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
                        frontierToRV = agent.calculatePath(ute.frontier.getCentre(), RVDestination);
                    }
                    timeMeeting = rvd.getParentRendezvous().getTimeMeeting();
                } else if (agent.getRole() == RobotConfig.roletype.Relay) {
                    RVDestination = rvd.getChildRendezvous().getParentLocation();
                    if (RVDestination != null) {
                        Path frontierToParent = agent.calculatePath(ute.frontier.getCentre(), RVDestination);
                        Path frontierToChild = agent.calculatePath(ute.frontier.getCentre(), rvd.getChildRendezvous().getChildLocation());
                        if (frontierToParent.getLength() < frontierToChild.getLength() && frontierToParent.found) {
                            frontierToRV = frontierToParent;
                        } else if (frontierToParent.getLength() >= frontierToChild.getLength() && frontierToChild.found) {
                            frontierToRV = frontierToChild;
                        } else
                            frontierToRV = frontierToParent;
                    }
                    timeMeeting = rvd.getChildRendezvous().getTimeMeeting();
                }
                if (RVDestination != null) {
                    Path meToFrontier = p;
                    
                    
                    int timeToRV = (int)((meToFrontier.getLength() + frontierToRV.getLength()) / Constants.DEFAULT_SPEED) + agent.getTimeElapsed();
                    //if time available is less than time to frontier, set utility to low value
                    if (timeToRV > timeMeeting) {
                        if (Constants.DEBUG_OUTPUT) {
                            System.out.println(agent + "Cannot explore frontier with centre " + ute.frontier.getCentre() + ", timeToRV is " + timeToRV + ", timeMeeting is " + timeMeeting + ", utility was " + ute.utility + ", setting utility to " + (ute.utility - 100000000));
                        }
                        ute.utility = ute.utility - 100000000;
                        if (agent.getRole() == RobotConfig.roletype.Relay)
                            ute.utility = -1;
                    }
                }
                
            }
        }
        else {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Could not find path from "  + start + " to " + ute.frontier.getCentre());
            }
            ute.utility = -1000;
        }

        /*System.out.println(Constants.INDENT + "New utility with ID " +
                                            ute.ID + " for frontier at " +
                                            ute.frontier.getCentre().x + "," +
                                            ute.frontier.getCentre().y + " is " +
                                            (int)ute.utility);                                            
                                            */
    }
    
    // Calculates Euclidean distance from all known teammates and self to frontiers of interest
    private static PriorityQueue initializeUtilities(RealAgent agent, LinkedList<Frontier> frontiers, 
            boolean considerOtherAgents, LinkedList<Integer> teammatesAssignedIDs) {  
        if (teammatesAssignedIDs == null)
            teammatesAssignedIDs = new LinkedList<Integer>();
        PriorityQueue<Utility> utilities = new PriorityQueue<Utility>();
        
        int lastCommLimit = Constants.REMEMBER_TEAMMATE_FRONTIER_PERIOD;
        if (!considerOtherAgents)
            lastCommLimit = -1;

        // For each frontier of interest
        for(Frontier frontier : frontiers) {
            // Add own utilities
            utilities.add(new Utility(agent.getID(),
                                      agent.getLocation(),
                                      frontier,
                                      utilityEstimate(agent.getLocation(), frontier),
                                      null));
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(Constants.INDENT + "Own utility with ID " +
                                                agent.getID() + " for frontier at " +
                                                frontier.getCentre().x + "," +
                                                frontier.getCentre().y + " is " +
                                                (int)utilityEstimate(agent.getLocation(), frontier));
            }
            // Add teammates' utilities
            for(TeammateAgent teammate : agent.getAllTeammates().values()) {
                if(teammate.getID() != 1 &&
                   teammate.getID() != agent.getID() &&
                   teammate.isExplorer() &&   // THIS LINE ONLY IF non-explorer agents are to be ignored
                   teammate.getTimeSinceLastComm() < lastCommLimit &&
                   !teammatesAssignedIDs.contains(teammate.getID())) {
                    utilities.add(new Utility(teammate.getID(),
                                              teammate.getLocation(), 
                                              frontier,
                                              utilityEstimate(teammate.getLocation(), frontier),
                                              null));
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(Constants.INDENT + "Utility of robot with ID " +
                                        teammate.getID() + " at location " + teammate.getLocation() + 
                                        " for frontier at " +
                                        frontier.getCentre().x + "," +
                                        frontier.getCentre().y + " is " +
                                        (int)utilityEstimate(teammate.getLocation(), frontier));                                        
                    }       
                }
            }
         }

        return utilities;
    }
    
    // Calculates Euclidean distance from all known teammates and self to frontiers of interest
    private static PriorityQueue initializeUtilities(Point p, PriorityQueue<Frontier> frontiers) {      
        PriorityQueue<Utility> utilities = new PriorityQueue<Utility>();

        // For each frontier of interest
        for(Frontier frontier : frontiers) {
            // Add own utilities
            utilities.add(new Utility(99,
                                      p,
                                      frontier,
                                      utilityEstimate(p, frontier),
                                      null));
         }

        return utilities;
    }
    
    private static class Utility implements Comparable<Utility>  {

        public int ID;
        public Point agentLocation;
        public Frontier frontier;
        public double utility;
        public Path path;
        
        public Utility(int id, Point al, Frontier f, double u, Path p) {
            ID = id;
            agentLocation = al;
            frontier = f;
            utility = u;
            path = p;
        }
        
        public int compareTo(Utility other) {
            if(other.utility > this.utility)
                return 1;
            else
                return -1;
        }
        
        @Override
        public String toString() {
            return "Utility ID: " + ID + ", agentLocation: (" + (int)agentLocation.getX() + "," + (int)agentLocation.getX()+ "), frontier: " + frontier 
                    + ", utility: " + utility;
        }
    }
    
    public static double maxFrontierUtility(PriorityQueue<Frontier> frontiers, OccupancyGrid grid, Point start) {
        PriorityQueue<Utility> utilities = initializeUtilities(start, frontiers);
        
        return utilities.peek().utility;
    }
    
    public static LinkedList<Integer> chooseFrontier(RealAgent agent, boolean considerOtherAgents,
            LinkedList<Integer> teammatesAssignedIDs) {
        if (teammatesAssignedIDs == null)
            teammatesAssignedIDs = new LinkedList<Integer>();
        // Step 1:  Create list of frontiers of interest (closest ones)
        LinkedList<Frontier> frontiers = frontiersOfInterest(agent, agent.getLastFrontier(), agent.getFrontiers(), agent.getOccupancyGrid());
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontiers of interest: " + frontiers.size());
        }
        // Step 2:  Create priorityQueue of utility estimates (Euclidean distance)
        PriorityQueue<Utility> utilities = initializeUtilities(agent, frontiers, considerOtherAgents, teammatesAssignedIDs);
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontier utilities: " + utilities.size());
        }
        // Step 3
        Utility best = null;
        LinkedList<Utility> removal;
        boolean isLastFrontier = false;
        int counter = 0;
        
        while(!utilities.isEmpty()) {    // && counter < 5) {
            best = utilities.poll();
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + best.toString());
            }
            // Check if this is the only remaining frontier
            isLastFrontier = true;
            if(!utilities.isEmpty())
                for(Utility u: utilities)
                    if(u.frontier != best.frontier){
                        isLastFrontier = false;
                        break;
                    }

            /*// If this is the only remaining frontier
            if(isLastFrontier) {
                //if(best.ID == agent.getID()) {
                    // just in case path hasn't been computed yet
                    if(best.path == null) {
                        // find closest point on frontier farthest away from teammates
                        double closestDist = 1000000;
                        Point closestPoint = best.frontier.getCentre();
                        double closestTeammateDist = 1000000;
                        for(int i=0; i<best.frontier.getPolygonOutline().size(); i++) {
                            if(!agent.getOccupancyGrid().obstacleWithinDistance(best.frontier.getPolygonOutline().get(i).x, best.frontier.getPolygonOutline().get(i).y, Constants.WALL_DISTANCE)) {
                                if(best.frontier.getPolygonOutline().get(i).distance(agent.getLocation()) < closestDist) {
                                    closestPoint = best.frontier.getPolygonOutline().get(i);
                                    closestDist = best.frontier.getPolygonOutline().get(i).distance(agent.getLocation());
                                    closestTeammateDist = 0;
                                     for(TeammateAgent teammate: agent.getAllTeammates().values())
                                        if(teammate.isInDirectRange())
                                            closestTeammateDist += teammate.getLocation().distance(best.frontier.getPolygonOutline().get(i));

                                }
                                else if(best.frontier.getPolygonOutline().get(i).distance(agent.getLocation()) == closestDist) {
                                    double newTeammateDist = 0;
                                     for(TeammateAgent teammate: agent.getAllTeammates().values())
                                        if(teammate.isInDirectRange())
                                            newTeammateDist += teammate.getLocation().distance(best.frontier.getPolygonOutline().get(i));
                                    if(newTeammateDist > closestTeammateDist) {
                                        closestPoint = best.frontier.getPolygonOutline().get(i);
                                        closestDist = best.frontier.getPolygonOutline().get(i).distance(agent.getLocation());
                                        closestTeammateDist = newTeammateDist;
                                    }

                                }
                            }
                        }
                        
                        best.path = agent.calculatePath(agent.getLocation(), best.frontier.getCentre());
                    }

                    agent.setLastFrontier(best.frontier);
                    agent.setCurrentGoal(best.frontier.getCentre());
                    agent.addDirtyCells(agent.getPath().getAllPathPixels());
                    agent.setPath(best.path);
                    return true;
            }*/

            // If this is an estimate, calculate true utility
            if(best.path == null)
                calculateUtilityExact(agent, best);
            
            if(best.path == null) {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(agent + " could not calculate exact utility: " + best + ", removing frontier: " +
                        best.frontier);
                }
                // it's not possible to plan a path to this frontier, so eliminate it entirely
                removal = new LinkedList<Utility>();
                for(Utility u : utilities)
                    if(u.frontier == best.frontier)
                        removal.add(u);
                for(Utility r : removal) {                
                    utilities.remove(r);                    
                }
                if (best.ID == agent.getID()) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(agent + " adding bad frontier");
                    }
                    agent.addBadFrontier(best.frontier); //only add bad frontiers if they are 'ours'
                }
                    
            } else {
                //System.out.println("UtilityExact: " + best.utility);
                if((utilities.size() == 0) || (best.utility >= utilities.peek().utility)){
                    if(best.ID == agent.getID()){
                        if ((agent.getRole() == RobotConfig.roletype.Relay) && (best.utility < 0)) {//cannot reach frontier in time
                            agent.setState(BasicAgent.ExploreState.GoToChild);
                            return null;
                        }
                        agent.setLastFrontier(best.frontier);
                        agent.setCurrentGoal(best.frontier.getCentre());
                        if(agent.getPath() != null)
                            agent.addDirtyCells(agent.getPath().getAllPathPixels());
                        agent.setPath(best.path);
                        return null;
                    }
                    else {
                        // This robot assigned, so remove all remaining associated utilities
                        if (Constants.DEBUG_OUTPUT) {
                            System.out.println(agent + "This robot assigned, so remove all remaining associated utilities");
                        }
                        removal = new LinkedList<Utility>();
                        for(Utility u : utilities)
                            if(u.ID == best.ID  ||
                               u.frontier == best.frontier)
                                removal.add(u);
                        for(Utility r : removal)
                            utilities.remove(r);
                        teammatesAssignedIDs.add(best.ID);
                    }
                } else
                    utilities.add(best);
            }

            counter++;
        }
        
        return teammatesAssignedIDs;  // couldn't assign frontier - could be there are more robots than frontiers?
    } 
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Calculate Frontiers">
    public static void calculateFrontiers(RealAgent agent, SimulatorConfig.frontiertype frontierExpType) {
        long realtimeStart = System.currentTimeMillis();
        //System.out.print(agent.toString() + "Calculating frontiers. ");

        // If recalculating frontiers, must set old frontiers dirty for image rendering
        for(Frontier f : agent.getFrontiers())
            agent.addDirtyCells(f.getPolygonOutline());
        
        LinkedList <LinkedList> contours = ContourTracer.findAllContours(agent.getOccupancyGrid());
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + "Found " + contours.size() + " contours, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        }
        realtimeStart = System.currentTimeMillis();
        PriorityQueue<Frontier> frontiers = new PriorityQueue();
        Frontier currFrontier;

        int contourCounter = 0;
        int contoursSmall = 0;
        int contoursBad = 0;
        for(LinkedList<Point> currContour : contours) {
            currFrontier = new Frontier(agent.getX(), agent.getY(), currContour);
            if (!agent.isBadFrontier(currFrontier)) {
                // only consider frontiers that are big enough and reachable
                if(currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE){
                    //if (!agent.getOccupancyGrid().safeSpaceAt(currFrontier.getCentre().x, currFrontier.getCentre().y))
                    {
                        frontiers.add(currFrontier);
                        contourCounter++;
                    }
                } else
                {
                    contoursSmall++;
                    //System.out.println("Disregarding a contour as it's smaller than min_frontier_size which is " + Constants.MIN_FRONTIER_SIZE);
                }
            } else {
                contoursBad++;
            }
        }
        agent.setFrontiers(frontiers);

        if (Constants.DEBUG_OUTPUT) {
            System.out.println("retained " + contourCounter + " of them, disregarded due to size " + contoursSmall + 
                ", disregarded as bad " + contoursBad + ". Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        }
        //System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
// </editor-fold> 
}
