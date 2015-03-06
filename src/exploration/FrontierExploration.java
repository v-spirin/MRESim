/*
 *     Copyright 2010, 2014 Julian de Hoog (julian@dehoog.ca), Victor Spirin (victor.spirin@cs.ox.ac.uk)
 *
 *     This file is part of MRESim 2.2, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our paper:
 *
 *     @inproceedings{deHoog2009,
 *         title = "Role-Based Autonomous Multi-Robot Exploration",
 *         author = "Julian de Hoog, Stephen Cameron and Arnoud Visser",
 *         year = "2009",
 *         booktitle = "International Conference on Advanced Cognitive Technologies and Applications (COGNITIVE)",
 *         location = "Athens, Greece",
 *         month = "November",
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
import config.SimulatorConfig;
import environment.*;
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
            SimulatorConfig.frontiertype frontierExpType, SimulatorConfig simConfig) {
        long realtimeStartAgentCycle = System.currentTimeMillis();
        
        Point nextStep = new Point(agent.getX(), agent.getY());

        //<editor-fold defaultstate="collapsed" desc="If base station is in range, update timeLastDirectContactCS and lastContactAreaKnown">
        if(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInRange()) {
            agent.timeLastDirectContactCS = 1;
            agent.setLastContactAreaKnown(agent.getAreaKnown());
        }
        else
            agent.timeLastDirectContactCS++;
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="CHECK 0: Take a couple of random steps to start (just to gather some sensor data).">
        if(timeElapsed < Constants.INIT_CYCLES) {
            nextStep = RandomWalk.takeStep(agent);
            agent.setTimeSinceLastPlan(0);
        }
        //</editor-fold>
 
        //<editor-fold defaultstate="collapsed" desc="CHECK 1: if agent hasn't moved, then he may be stuck in front of a wall.  Taking a random step might help.">
        else if(agent.getEnvError()) {
            nextStep = RandomWalk.takeStep(agent);
            agent.setTimeSinceLastPlan(0);
            agent.setEnvError(false);
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="CHECK 2: Agent isn't stuck, not yet time to replan. Continue if we have points left in the previously planned path.">
        else if ((agent.timeSinceLastPlan < Constants.REPLAN_INTERVAL)
                && agent.getPath().found && agent.getPath().getPoints().size() >= 2)
        {
            nextStep = agent.getNextPathPoint();
            System.out.println("Agent " + agent + " continuing on path.");
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="CHECK 3: Agent isn't stuck. Is it time to replan? Have we moved on to next time cycle?">
        else if(agent.getTimeSinceLastPlan() >= Constants.REPLAN_INTERVAL)
        {
            nextStep = replan(agent, frontierExpType, timeElapsed, simConfig);
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="CHECK 4: Agent isn't stuck, not yet time to replan, but we have no points left">
        else
        {
            nextStep = replan(agent, frontierExpType, timeElapsed, simConfig);
        }
        //</editor-fold>

        agent.timeSinceLastPlan++;

        //agent.setLastTotalKnowledgeBelief(agent.getCurrentTotalKnowledgeBelief());
        //agent.setLastBaseKnowledgeBelief(agent.getCurrentBaseKnowledgeBelief());
        //agent.setLastNewInfo(agent.getNewInfo());

        System.out.println(agent.toString() + "takeStep took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
        return nextStep;
    }   

    public static Point replan(RealAgent agent, SimulatorConfig.frontiertype frontierExpType, int timeElapsed, SimulatorConfig simConfig) {
        Point nextStep;
        
        agent.setTimeSinceLastPlan(0);
        System.out.println(agent.toString() + "starting replan");
        long realtimeStart = System.currentTimeMillis();
                     
        calculateFrontiers(agent, frontierExpType);
        
        System.out.println(agent.toString() + "calculateFrontiers took " + (System.currentTimeMillis()-realtimeStart) + "ms.");

        //<editor-fold defaultstate="collapsed" desc="If no frontiers found, or reached exploration goal, return to ComStation">
        if ((agent.getFrontiers().isEmpty()) || (agent.getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL)) {
            agent.setMissionComplete();
            agent.setPathToBaseStation();
            nextStep = agent.getNextPathPoint();
            while ((nextStep != null) && (nextStep.equals(agent.getLocation())))
            {
                nextStep = agent.getNextPathPoint();
            }
            agent.setTimeSinceLastPlan(0);
            agent.setCurrentGoal(agent.getTeammate(1).getLocation());
            return nextStep;
        }
        //</editor-fold>

        long realtimeStart2 = System.currentTimeMillis();
        boolean foundFrontier = chooseFrontier(agent, true);
        System.out.println(agent.toString() + "chooseFrontier took " + (System.currentTimeMillis()-realtimeStart2) + "ms.");
        
        //<editor-fold defaultstate="collapsed" desc="If could not find frontier, try to disregard other agents when planning">
        if (!foundFrontier)
        {
            System.out.println(agent.toString() + " could not find frontier, trying to ignore other agents...");
            foundFrontier = chooseFrontier(agent, false);
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="If no frontier could be assigned, then go back to base.">
        if(!foundFrontier){
            /*System.out.println(agent.toString() + " No frontier chosen, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
            agent.setTimeSinceLastPlan(0);
            agent.setCurrentGoal(nextStep);
            return nextStep;*/
            // mission complete
            agent.setMissionComplete();
            agent.setPathToBaseStation();
            nextStep = agent.getNextPathPoint();
            while ((nextStep != null) && (nextStep.equals(agent.getLocation())))
            {
                nextStep = agent.getNextPathPoint();
            }
            agent.setTimeSinceLastPlan(0);
            agent.setCurrentGoal(agent.getTeammate(1).getLocation());
            return nextStep;
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="If overlapping another agent, take random step">
        for(TeammateAgent teammate : agent.getAllTeammates().values())
        {
            if (agent.getLocation().equals(teammate.getLocation()))
            {
                nextStep = RandomWalk.takeStep(agent);
                agent.setTimeSinceLastPlan(0);
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
                agent.getPath().getPoints().isEmpty())
        {
            nextStep = RandomWalk.takeStep(agent);
            agent.setTimeSinceLastPlan(0);
            agent.setEnvError(false);
            return nextStep;
        }
        // If we reach this point, we have a path.  Remove the first point
        // since this is the robot itself.
        agent.getPath().getPoints().remove(0);        
        nextStep = agent.getNextPathPoint();
        System.out.println(agent.toString() + "replan took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        return nextStep;
    }   

// <editor-fold defaultstate="collapsed" desc="Choose best frontier">
    private static LinkedList<Frontier> frontiersOfInterest (Frontier lastFrontier, PriorityQueue<Frontier> frontiers, OccupancyGrid grid) {
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
                //ignore the small frontiers inside walls
               if ((currFrontier.getArea() < Constants.MIN_FRONTIER_SIZE * 4) && (grid.obstacleWithinDistance(currFrontier.getCentre().x, currFrontier.getCentre().y, 3)))
               {
                   
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
        if(ute.ID == agent.getID())
            start = agent.getLocation();
        else
            start = agent.getTeammate(ute.ID).getLocation();

        // old method:
        //Path p = new Path(agent, start, ute.frontier.getCentre());        
        Path p;
        if (ute.frontier.getClosestPoint(start, agent.getOccupancyGrid()).x == 0)
        {
            //System.out.println("Closest point is 0, this shouldn't happen");
            p = new Path();
            p.found = false;
        } else
        {
            p = agent.calculatePath(start, ute.frontier.getCentre()/*ute.frontier.getClosestPoint(start, agent.getOccupancyGrid())*/);
        }

        if(p.found) {
            ute.path = p;
            ute.utility = (ute.frontier.getArea() * 100000000) / Math.pow(p.getLength(), 4);
        }
        else
            ute.utility = -1000;

        /*System.out.println(Constants.INDENT + "New utility with ID " +
                                            ute.ID + " for frontier at " +
                                            ute.frontier.getCentre().x + "," +
                                            ute.frontier.getCentre().y + " is " +
                                            (int)ute.utility);                                            
                                            */
    }
    
    // Calculates Euclidean distance from all known teammates and self to frontiers of interest
    private static PriorityQueue initializeUtilities(RealAgent agent, LinkedList<Frontier> frontiers, boolean considerOtherAgents) {      
        PriorityQueue<Utility> utilities = new PriorityQueue<Utility>();
        
        int lastCommLimit = 30;
        if (!considerOtherAgents)
            lastCommLimit = 1;

        // For each frontier of interest
        for(Frontier frontier : frontiers) {
            // Add own utilities
            utilities.add(new Utility(agent.getID(),
                                      agent.getLocation(),
                                      frontier,
                                      utilityEstimate(agent.getLocation(), frontier),
                                      null));
            /*System.out.println(Constants.INDENT + "Own utility with ID " +
                                                agent.getID() + " for frontier at " +
                                                frontier.getCentre().x + "," +
                                                frontier.getCentre().y + " is " +
                                                (int)utilityEstimate(agent.getLocation(), frontier));
            */
            // Add teammates' utilities
            for(TeammateAgent teammate : agent.getAllTeammates().values()) {
                if(teammate.getID() != 1 &&
                   teammate.getID() != agent.getID() &&
                   teammate.isExplorer() &&   // THIS LINE ONLY IF non-explorer agents are to be ignored
                   teammate.getTimeSinceLastComm() < lastCommLimit) {
                    utilities.add(new Utility(teammate.getID(),
                                              teammate.getLocation(), 
                                              frontier,
                                              utilityEstimate(teammate.getLocation(), frontier),
                                              null));
                    /*System.out.println(Constants.INDENT + "Utility of robot with ID " +
                                        teammate.getID() + " for frontier at " +
                                        frontier.getCentre().x + "," +
                                        frontier.getCentre().y + " is " +
                                        (int)utilityEstimate(teammate.getLocation(), frontier));                                        
                                        */
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
    }
    
    public static double maxFrontierUtility(PriorityQueue<Frontier> frontiers, OccupancyGrid grid, Point start) {
        PriorityQueue<Utility> utilities = initializeUtilities(start, frontiers);
        
        return utilities.peek().utility;
    }
    
    public static boolean chooseFrontier(RealAgent agent, boolean considerOtherAgents) {
        // Step 1:  Create list of frontiers of interest (closest ones)
        LinkedList<Frontier> frontiers = frontiersOfInterest(agent.getLastFrontier(), agent.getFrontiers(), agent.getOccupancyGrid());
        
        // Step 2:  Create priorityQueue of utility estimates (Euclidean distance)
        PriorityQueue<Utility> utilities = initializeUtilities(agent, frontiers, considerOtherAgents);

        // Step 3
        Utility best = null;
        LinkedList<Utility> removal;
        boolean isLastFrontier = false;
        int counter = 0;
        
        while(!utilities.isEmpty()) {    // && counter < 5) {
            best = utilities.poll();

            // Check if this is the only remaining frontier
            isLastFrontier = true;
            if(!utilities.isEmpty())
                for(Utility u: utilities)
                    if(u.frontier != best.frontier){
                        isLastFrontier = false;
                        break;
                    }

            // If this is the only remaining frontier
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
            }

            // If this is an estimate, calculate true utility
            if(best.path == null)
                calculateUtilityExact(agent, best);

            //System.out.println("UtilityExact: " + best.utility);
            if(best.utility >= utilities.peek().utility){
                if(best.ID == agent.getID()){
                    agent.setLastFrontier(best.frontier);
                    agent.setCurrentGoal(best.frontier.getCentre());
                    if(agent.getPath() != null)
                        agent.addDirtyCells(agent.getPath().getAllPathPixels());
                    agent.setPath(best.path);
                    return true;
                }
                else {
                    // This robot assigned, so remove all remaining associated utilities
                    removal = new LinkedList<Utility>();
                    for(Utility u : utilities)
                        if(u.ID == best.ID  ||
                           u.frontier == best.frontier)
                            removal.add(u);
                    for(Utility r : removal)
                        utilities.remove(r);
                }
            }
            else if(best.path == null) {
                // it's not possible to plan a path to this frontier, so eliminate it entirely
                removal = new LinkedList<Utility>();
                for(Utility u : utilities)
                    if(u.frontier == best.frontier)
                        removal.add(u);
                for(Utility r : removal) {                
                    utilities.remove(r);                    
                }
                agent.addBadFrontier(best.frontier);
            }
            else
                utilities.add(best);

            counter++;
        }
        
        return false;  // should only happen if there are no frontiers at all
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
        System.out.print("Found " + contours.size() + " contours, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        realtimeStart = System.currentTimeMillis();
        PriorityQueue<Frontier> frontiers = new PriorityQueue();
        Frontier currFrontier;

        int contourCounter = 0;
        for(LinkedList<Point> currContour : contours) {
            currFrontier = new Frontier(agent.getX(), agent.getY(), currContour);
            if (!agent.isBadFrontier(currFrontier)) {
                // only consider frontiers that are big enough and reachable
                if(currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE){
                    if (!agent.getOccupancyGrid().safeSpaceAt(currFrontier.getCentre().x, currFrontier.getCentre().y))
                    {
                        frontiers.add(currFrontier);
                        contourCounter++;
                    }
                } else
                {
                    //System.out.println("Disregarding a contour as it's smaller than min_frontier_size which is " + Constants.MIN_FRONTIER_SIZE);
                }
            }
        }
        agent.setFrontiers(frontiers);

        System.out.print("retained " + contourCounter + " of them. Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        //System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
// </editor-fold> 
}
