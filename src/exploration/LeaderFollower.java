/*
 *     Copyright 2010, 2015 Julian de Hoog (julian@dehoog.ca), Victor Spirin (victor.spirin@cs.ox.ac.uk)
 *
 *     This file is part of MRESim 2.2, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our papers:
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
import environment.ContourTracer;
import environment.Frontier;
import environment.OccupancyGrid;
import java.awt.Point;
import java.util.LinkedList;
import java.util.PriorityQueue;
import path.Path;

/**
 *
 * @author juliandehoog
 */
public class LeaderFollower {

    private static int TIME_BETWEEN_PLANS = 10;

// <editor-fold defaultstate="collapsed" desc="Take Step">
    public static Point takeStep(RealAgent agent, int timeElapsed) {
        Point nextStep = new Point(agent.getX(), agent.getY());

        // if env reports error, agent may be stuck in front of a wall and the
        // simulator isn't allowing him to go through.  Taking a random step might
        // help.
        // Update:  this state is never really reached but leave in just in case
        if(agent.getEnvError()) {
            System.out.println(agent.toString() + "LeaderFollower: Env reports error, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
            agent.setEnvError(false);
            return nextStep;
        }

        switch(agent.getRole()) {
            case Explorer :         nextStep = takeStep_Explorer(agent, timeElapsed);
                                    break;
            case Relay :            nextStep = takeStep_Relay(agent, timeElapsed);
                                    break;
            default :               break;
        }

        if(nextStep == null)
            nextStep = RandomWalk.takeStep(agent);

        return nextStep;
    }

    public static Point takeStep_Relay(RealAgent agent, int timeElapsed) {
        agent.setMissionComplete(true); // to stop scripted runs when all agents complete mission
        Point nextStep = new Point(agent.getX(), agent.getY());

        // CHECK 0
        // Wait for a few time steps at start (to let Explorer get some moves in).
        if(timeElapsed < 3) {
            System.out.println(agent.toString() + "LeaderFollower: Starting up, staying stationary for now.");
            nextStep = new Point(agent.getX(), agent.getY());
            agent.getStats().setTimeSinceLastPlan(0);
        }


        // CHECK 1
        // if agent hasn't moved, then he may be stuck in front of a wall and the
        // simulator isn't allowing him to go through.  Taking a random step might
        // help.
        else if(agent.getEnvError()) {
            System.out.println(agent.toString() + "LeaderFollower: No step taken since last timeStep, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setEnvError(false);
        }

        // Check 1.5, make sure parent is in range
        else if(!agent.getParentTeammate().isInRange()) {
            agent.getStats().setTimeSinceLastPlan(0);
            return (new Point(agent.getPrevX(), agent.getPrevY()));
        }


        // CHECK 2
        // Agent isn't stuck.
        // Is it time to replan?
        else if(agent.getStats().getTimeSinceLastPlan() > TIME_BETWEEN_PLANS) {
            nextStep = replanRelay(agent);
            agent.getStats().setTimeSinceLastPlan(0);
        }

        // CHECK 3
        // Agent isn't stuck, not yet time to replan.
        // Do we have points left in the previously planned path?
        else if(agent.getPath().found && agent.getPath().getPoints().size() >= 2)
               nextStep = agent.getNextPathPoint();


        // CHECK 4
        // Agent isn't stuck, not yet time to replan, but we have no points left
        else {
            nextStep = replanRelay(agent);
            agent.getStats().setTimeSinceLastPlan(0);
        }

        // UPDATE
        if(agent.getTeammate(1).isInRange())
            agent.getStats().setTimeLastDirectContactCS(1);
        else
            agent.getStats().incrementLastDirectContactCS();
        agent.getStats().incrementTimeSinceLastPlan();


        return nextStep;
    }

    public static Point replanRelay(RealAgent agent) {
        Path P2C = agent.calculatePath(agent.getLocation(), agent.getChildTeammate().getLocation());
        
        if (agent.getLocation().distance(agent.getParentTeammate().getLocation()) < 
                agent.getCommRange() - 1* agent.getSpeed())
        {
            agent.setPath(P2C);
            return agent.getNextPathPoint();
        } else
            return (new Point(agent.getPrevX(), agent.getPrevY()));

        // total hack:  for leader-follower, use "parent" value as number of robots between this relay and comstation, including self
        /*if(agent.getChildTeammate().getLocation().distance(agent.getTeammate(1).getLocation()) <=
           (agent.getParent() * agent.getCommRange() - 2*Constants.DEFAULT_SPEED - 20)) {
                agent.setPath(agent.calculatePath(agent.getLocation(), agent.getChildTeammate().getLocation()));
                System.out.println("LeaderFollower: my child is still in my allowed range (" + (agent.getParent() * agent.getCommRange() - 2*Constants.DEFAULT_SPEED) + ")");
        }
        else {
            System.out.println("LeaderFollower: my child is no longer in my allowed range (" + (agent.getParent() * agent.getCommRange() - 2*Constants.DEFAULT_SPEED) + ")");
            double cDist = 1000000;
            Point cPoint = new Point(agent.getX(), agent.getY());
            // find closest point within allowed range that is near parent
            for(int i=agent.getX()-10; i<= agent.getX()+10; i+=10)
                for(int j=agent.getY()-10; j<=agent.getY()+10; j+=10) {
                    if(agent.getOccupancyGrid().locationExists(i, j) &&
                       //agent.getOccupancyGrid().freeSpaceAt(i, j) &&
                      !agent.getOccupancyGrid().obstacleWithinDistance(i, j, Constants.WALL_DISTANCE) &&
                       agent.getTeammate(1).getLocation().distance(new Point(i,j)) < agent.getParent() * agent.getCommRange() - 2*Constants.DEFAULT_SPEED &&
                       //(new Path(agent, new Point(i,j), agent.getTeammate(1).getLocation())).getLength() < agent.getParent() * agent.getCommRange() - 2*Constants.STEP_SIZE &&
                       agent.getChildTeammate().getLocation().distance(new Point(i,j)) < cDist) {
                            cPoint = new Point(i,j);
                            cDist = agent.getChildTeammate().getLocation().distance(cPoint);
                    }
                }
            System.out.println("Chose point at " + cPoint.x + "," + cPoint.y);
            agent.setPath(agent.calculatePath(agent.getLocation(), cPoint));
        }

        if(agent.getPath() == null || agent.getPath().getPoints().size()<2)
            return RandomWalk.takeStep(agent);
        
        agent.getPath().getPoints().remove(0);
        return agent.getNextPathPoint();*/
    }

    public static Point takeStep_Explorer(RealAgent agent, int timeElapsed) {
        Point nextStep = new Point(agent.getX(), agent.getY());

        // CHECK 0
        // Take a couple of random steps to start (just to gather some sensor data).
        if(timeElapsed < 3) {
            System.out.println(agent.toString() + "LeaderFollower: Starting up, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
        }


        // CHECK 1
        // if agent hasn't moved, then he may be stuck in front of a wall and the
        // simulator isn't allowing him to go through.  Taking a random step might
        // help.
        else if(agent.getEnvError()) {
            System.out.println(agent.toString() + "LeaderFollower: No step taken since last timeStep, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setEnvError(false);
        }


        // CHECK 2
        // Agent isn't stuck.
        // Is it time to replan?
        else if(agent.getStats().getTimeSinceLastPlan() % TIME_BETWEEN_PLANS == 0) {
            nextStep = replanExplorer(agent);
            agent.getStats().setTimeSinceLastPlan(0);
        }

        // CHECK 3
        // Agent isn't stuck, not yet time to replan.
        // Do we have points left in the previously planned path?
        else if(agent.getPath().found && agent.getPath().getPoints().size() >= 2)
               nextStep = agent.getNextPathPoint();


        // CHECK 4
        // Agent isn't stuck, not yet time to replan, but we have no points left
        else {
            nextStep = replanExplorer(agent);
            agent.getStats().setTimeSinceLastPlan(0);
        }

        // UPDATE
        if(agent.getTeammate(1).isInRange())
            agent.getStats().setTimeLastDirectContactCS(1);
        else
            agent.getStats().incrementLastDirectContactCS();
        agent.getStats().incrementTimeSinceLastPlan();


        return nextStep;
    }


    // </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Replan">

    public static Point replanExplorer(RealAgent agent) {
        if(!agent.getParentTeammate().isInRange()) {
            agent.getStats().setTimeSinceLastPlan(0);
            return (new Point(agent.getPrevX(), agent.getPrevY()));
        }
        Point nextStep;

        // Find frontiers
        calculateFrontiers(agent);

        // If no frontiers found, return to ComStation
        if(agent.getFrontiers().isEmpty()) {
            System.out.println(agent.toString() + "No frontiers found, returning home.");
            agent.setMissionComplete(true);
            agent.setPathToBaseStation();
            nextStep = agent.getNextPathPoint();
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setCurrentGoal(agent.getTeammate(1).getLocation());
            return nextStep;
        }

        // If we reach this point, there are frontiers to explore -- find best one
        long realtimeStart = System.currentTimeMillis();
        System.out.println(agent.toString() + "Choosing a frontier ...");

        boolean foundFrontier = chooseFrontier(agent);

        // If no best frontier could be assigned (can happen e.g. when more robots than frontiers),
        // then take random step.
        if(!foundFrontier){
            System.out.println(agent.toString() + "No frontier chosen, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setCurrentGoal(nextStep);
            return nextStep;
        }

        // Note: Path to best frontier has already been set when calculating
        // utility, no need to recalculate

        // If no path could be found, take random step.
        if(agent.getPath() == null || agent.getPath().getPoints() == null || agent.getPath().getPoints().size() == 0){
            System.out.println(agent.toString() + "No path found, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
            agent.getStats().setTimeSinceLastPlan(0);
            agent.setCurrentGoal(nextStep);
            return nextStep;
        }

        // If we reach this point, we have a path.  Remove the first point
        // since this is the robot itself.
        agent.getPath().getPoints().remove(0);
            agent.getStats().setTimeSinceLastPlan(0);
        System.out.print(Constants.INDENT + "Chose frontier at " + agent.getLastFrontier().getCentre().x + "," + agent.getLastFrontier().getCentre().y + ". ");
        System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        return agent.getNextPathPoint();
    }

// </editor-fold>

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
               counter < Constants.MAX_NUM_FRONTIERS) {
                list.add(currFrontier);
                counter++;
                // no need to break here, as we still want to add last frontier
                // if we haven't iterated through it yet.
            }

        }

        return list;
    }

    private static double utilityEstimate(Point agentLoc, Frontier frontier) {
        return (frontier.getArea() / Math.pow(agentLoc.distance(frontier.getCentre()), 4) * 100000000);
    }

    private static void calculateUtilityExact(RealAgent agent, Utility ute) {
        Point start;
        if(ute.ID == agent.getID())
            start = agent.getLocation();
        else
            start = agent.getTeammate(ute.ID).getLocation();

        Path p = agent.calculatePath(start, ute.frontier.getClosestPointInRange(agent));

        if(p.found) {
            ute.path = p;
            ute.utility = ute.frontier.getArea() / Math.pow(p.getLength(), 4) * 100000000;
        }
        else
            ute.utility = -1000;

        System.out.println(Constants.INDENT + "New utility with ID " +
                                            ute.ID + " for frontier at " +
                                            ute.frontier.getCentre().x + "," +
                                            ute.frontier.getCentre().y + " is " +
                                            (int)ute.utility);
    }

    // Calculates Euclidean distance from all known teammates and self to frontiers of interest
    private static PriorityQueue initializeUtilities(RealAgent agent, LinkedList<Frontier> frontiers) {
        PriorityQueue<Utility> utilities = new PriorityQueue<Utility>();

        // For each frontier of interest
        for(Frontier frontier : frontiers) {
            // Add own utilities
            utilities.add(new Utility(agent.getID(),
                                      agent.getLocation(),
                                      frontier,
                                      utilityEstimate(agent.getLocation(), frontier),
                                      null));
            System.out.println(Constants.INDENT + "Own utility with ID " +
                                                agent.getID() + " for frontier at " +
                                                frontier.getCentre().x + "," +
                                                frontier.getCentre().y + " is " +
                                                (int)utilityEstimate(agent.getLocation(), frontier));

            // Add teammates' utilities
            for(TeammateAgent teammate : agent.getAllTeammates().values()) {
                if(teammate.getID() != 1 &&
                   teammate.getID() != agent.getID() &&
                   teammate.isExplorer() &&   // THIS LINE ONLY IF non-explorer agents are to be ignored
                   teammate.getTimeSinceLastComm() < 30) {
                    utilities.add(new Utility(teammate.getID(),
                                              teammate.getLocation(),
                                              frontier,
                                              utilityEstimate(teammate.getLocation(), frontier),
                                              null));
                    System.out.println(Constants.INDENT + "Utility of robot with ID " +
                                        teammate.getID() + " for frontier at " +
                                        frontier.getCentre().x + "," +
                                        frontier.getCentre().y + " is " +
                                        (int)utilityEstimate(teammate.getLocation(), frontier));
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

    public static boolean chooseFrontier(RealAgent agent) {
        // Step 1:  Create list of frontiers of interest (closest ones)
        LinkedList<Frontier> frontiers = frontiersOfInterest(agent.getLastFrontier(), agent.getFrontiers(), agent.getOccupancyGrid());

        // Step 2:  Create priorityQueue of utility estimates (Euclidean distance)
        PriorityQueue<Utility> utilities = initializeUtilities(agent, frontiers);

        /*
        for(Utility p : utilities)
            System.out.println(agent.toString() +
                               "Agent " + p.ID + "; " +
                               "Frontier at " + p.frontier.getCentre().x + ", " + p.frontier.getCentre().y + "; " +
                               "Utility " + p.utility + "." );*/

        // Step 3
        Utility best = null;
        LinkedList<Utility> removal;
        int counter = 0;
        boolean isLastFrontier = false;

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
                    if(best.path == null)
                        best.path = agent.calculatePath(agent.getLocation(), best.frontier.getClosestPointInRange(agent));

                    agent.setLastFrontier(best.frontier);
                    agent.setCurrentGoal(best.frontier.getCentre());
                    agent.addDirtyCells(agent.getPath().getAllPathPixels());
                    agent.setPath(best.path);
                    return true;
                //}
                //else
                //    return false;  // no frontier found for this agent
            }

            // If this is an estimate, calculate true utility
            if(best.path == null)
                calculateUtilityExact(agent, best);

            //System.out.println("UtilityExact: " + best.utility);
            if(best.utility >= utilities.peek().utility){
                //System.out.println(best.agentLocation + " " + agent.getLocation());

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
                for(Utility r : removal)
                    utilities.remove(r);
            }
            else
                utilities.add(best);

            /*    for(Utility p : utilities)
            System.out.println(agent.toString() +
                               "RealAgent at " + p.agentLocation.x + ", " + p.agentLocation.y + "; " +
                               "Frontier at " + p.frontier.getCentre().x + ", " + p.frontier.getCentre().y + "; " +
                               "Utility " + p.utility + "." );*/
           // System.out.println(utilities.peek().utility);

            counter++;
        }

        return false;  // should only happen if there are no frontiers at all
    }

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Calculate Frontiers">

    private static boolean inRangeOfTeammate(Frontier f, RealAgent agent){
        for(TeammateAgent t : agent.getAllTeammates().values())
            if(f.getCentre().distance(t.getLocation()) < agent.getCommRange())
                return true;
        return false;
    }

    public static void calculateFrontiers(RealAgent agent) {
        long realtimeStart = System.currentTimeMillis();
        System.out.print(agent.toString() + "Calculating frontiers. ");

        // If recalculating frontiers, must set old frontiers dirty for image rendering
        for(Frontier f : agent.getFrontiers())
            agent.addDirtyCells(f.getPolygonOutline());

        LinkedList <LinkedList> contours = ContourTracer.findAllContours(agent.getOccupancyGrid());
        System.out.print("Found " + contours.size() + " contours, ");
        PriorityQueue<Frontier> frontiers = new PriorityQueue();
        Frontier currFrontier;

        int contourCounter = 0;
        for(LinkedList<Point> currContour : contours) {
            currFrontier = new Frontier(agent.getX(), agent.getY(), currContour);
            // only consider frontiers that are big enough and reachable
            if(currFrontier.getArea() >= Constants.MIN_FRONTIER_SIZE &&
              // !agent.getOccupancyGrid().obstacleWithinDistance(currFrontier.getCentre().x, currFrontier.getCentre().y, 2*Constants.WALL_DISTANCE)) {
                //The line below is the key to choosing only frontiers within range
                /*Point cp = currFrontier.getClosestPointInRange(agent);
                if(cp.x == 0 && cp.y == 0)
                    continue;
                Path tp = new Path(agent, agent.getParentTeammate().getLocation(), cp);
                if(tp.getLength() <= (1.5 * agent.getCommRange() - 2*Constants.STEP_SIZE) ||
                   inRangeOfTeammate(currFrontier, agent))*/
                   currFrontier.getCentre().distance(agent.getTeammate(1).getLocation()) < agent.getCommRange() * agent.getParent() - 5) {
                    frontiers.add(currFrontier);
                contourCounter++;
            }
        }
        agent.setFrontiers(frontiers);

        System.out.print("retained " + contourCounter + " of them. ");
        System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

// </editor-fold>

}
