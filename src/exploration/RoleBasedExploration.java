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


import agents.BasicAgent;
import agents.RealAgent;
import communication.PropModel1;
import config.Constants;
import config.RobotConfig;
import config.SimulatorConfig;
import environment.*;
import java.util.*;
import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import path.Path;

/**
 *
 * @author julh
 */

public class RoleBasedExploration {    

// <editor-fold defaultstate="collapsed" desc="Take Step">
    public static boolean useImprovedRendezvous;
    public static boolean allowReplanning;
    public static boolean rvCommRange;
    public static boolean rvThroughWalls;
    public static int timeElapsed;
    public static int oldTimeElapsed;
    
    // Returns new X, Y of ExploreAgent
    public static Point takeStep(RealAgent agent, int te, boolean rv, boolean ar, SimulatorConfig simConfig) {
        long realtimeStart = System.currentTimeMillis();
        //<editor-fold defaultstate="collapsed" desc="Assign local variables">
        timeElapsed = te;
        useImprovedRendezvous = rv;
        allowReplanning = ar;
        rvCommRange = simConfig.RVCommRangeEnabled();
        rvThroughWalls = simConfig.RVThroughWallsEnabled();        
        
        Point nextStep = null;
        //</editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="Handle environment error - agent stuck next to a wall">
        // if env reports error, agent may be stuck in front of a wall and the
        // simulator isn't allowing him to go through.  Taking a random step might
        // help.
        // Update:  this state is never really reached but leave in just in case
        if(agent.getEnvError() && (agent.getState() != BasicAgent.ExploreState.GetInfoFromChild)
                               && (agent.getState() != BasicAgent.ExploreState.GiveParentInfo)
                               && (agent.getState() != BasicAgent.ExploreState.WaitForChild)
                               && (agent.getState() != BasicAgent.ExploreState.WaitForParent)) {
            System.out.println(agent.toString() + "!!!RoleBasedExploration: Env reports error, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
            agent.setEnvError(false);
            return nextStep;
        }
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="Run correct takeStep function depending on agent state, set nextStep to output">
        switch(agent.getState()) {
            case Initial :          nextStep = takeStep_Initial(agent, simConfig);
                                    break;
            case Explore :          nextStep = takeStep_Explore(agent, simConfig);
                                    break;
            case ReturnToParent :   nextStep = takeStep_ReturnToParent(agent, simConfig);
                                    break;
            case WaitForParent :    nextStep = takeStep_WaitForParent(agent, simConfig);
                                    break;
            case GiveParentInfo :   nextStep = takeStep_GiveParentInfo(agent, simConfig);
                                    break;
            case GoToChild :        nextStep = takeStep_GoToChild(agent, simConfig);
                                    break;
            case WaitForChild :     nextStep = takeStep_WaitForChild(agent, simConfig);
                                    break;
            case GetInfoFromChild : nextStep = takeStep_GetInfoFromChild(agent, simConfig);
                                    break;
            default :               break;
        }
        
        // this shouldn't happen, looks like one of takeSteps returned an error
        if(nextStep == null)
        {
            System.out.println(agent.toString() + "!!!RoleBasedExploration: nextStep is null, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
        }
        // </editor-fold>
     
        // <editor-fold defaultstate="collapsed" desc="Increment state timers">
        agent.setStateTimer(agent.getStateTimer() + 1);
        agent.setTimeSinceLastRoleSwitch(agent.getTimeSinceLastRoleSwitch() + 1);
        agent.setTimeSinceLastRVCalc(agent.getTimeSinceLastRVCalc() + 1);
        //</editor-fold>
        System.out.println(agent.toString() + " takeStep " + agent.getState() + ", took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        return nextStep;
    }
    
    private static Point takeStep_Initial(RealAgent agent, SimulatorConfig simConfig) {
        // Small number of random steps to get initial range data
        // <editor-fold defaultstate="collapsed" desc="First 3 steps? Explorers take 2 random steps while others wait, then everyone takes a random step">
        if (agent.getStateTimer() < 2)
            if(agent.isExplorer())
                return agent.getLocation();//RandomWalk.takeStep(agent);
            else 
                return(agent.getLocation()); 
        else if (agent.getStateTimer() < 3)
                //return RandomWalk.takeStep(agent);
            return agent.getLocation();
        // </editor-fold>
        // <editor-fold defaultstate="collapsed" desc="Otherwise? Explorers go into Explore state, others go into GoToChild state. Explorers replan using FrontierExploration, others do nothing.">
        else {
            if(agent.isExplorer()) {
                agent.setState(RealAgent.ExploreState.Explore);                
                agent.setTimeSinceLastPlan(0);
                return FrontierExploration.replan(agent, SimulatorConfig.frontiertype.ReturnWhenComplete, 0, simConfig);
            }
            else {
                agent.setState(RealAgent.ExploreState.GoToChild);
                agent.setStateTimer(0);
                return(agent.getLocation());
            }
        }
        // </editor-fold>
    }
    
    private static Point takeStep_Explore(RealAgent agent, SimulatorConfig simConfig) {
        // <editor-fold defaultstate="collapsed" desc="If parent is in range, and we have been exploring for longer than MIN_TIME_IN_EXPLORE_STATE, give parent info. Else, decrease time until RV">
        if (agent.getParentTeammate().isInRange()) {
            if ((agent.getStateTimer() > Constants.MIN_TIME_IN_EXPLORE_STATE) || 
                    // we have a new RV point, and must communicate it to the parent - otherwise the parent will be waiting at the old RV forever!
                    (!agent.getParentRendezvous().equals(agent.getParentTeammate().getChildRendezvous()))) 
            { 
                agent.setState(RealAgent.ExploreState.GiveParentInfo);
                agent.setStateTimer(0);
                return(takeStep_GiveParentInfo(agent, simConfig));
            }
        }
        else
            agent.setTimeUntilRendezvous(agent.getTimeUntilRendezvous() - 1);
        // </editor-fold>
        // <editor-fold defaultstate="collapsed" desc="Every CHECK_INTERVAL_TIME_TO_RV steps, check if we're due to meet our parent again (unless parent is basestation, in which case explore continuously)">
        if (agent.getParent() != Constants.BASE_STATION_TEAMMATE_ID
                && (agent.getStateTimer() % Constants.CHECK_INTERVAL_TIME_TO_RV) == (Constants.CHECK_INTERVAL_TIME_TO_RV - 1)) 
        {
            System.out.println(agent.toString() + "Checking if it's time to rendezvous ... ");
            Path pathToParentRendezvous = agent.calculatePath(agent.getLocation(), agent.getParentRendezvous().getChildLocation());
            System.out.println(Constants.INDENT + "rendezvous is " + (int) pathToParentRendezvous.getLength() + " away, time left is " + agent.getTimeUntilRendezvous() + ".");

            // If we are due to meet parent again, return to last agreed rendezvous point
            if (pathToParentRendezvous.found) {
                if (((pathToParentRendezvous.getLength() / Constants.DEFAULT_SPEED) + timeElapsed) >= 
                        agent.getParentRendezvous().getTimeMeeting()) {
                    if (!useImprovedRendezvous) {
                        //if we are not using improved RV, then next RV point is the point at which the Explorer turns back to RV with the parent
                        agent.setChildRendezvous(new RVLocation(agent.getLocation()));
                    }
                    if (agent.getPath() != null) {
                        agent.addDirtyCells(agent.getPath().getAllPathPixels());
                    }
                    agent.setPath(pathToParentRendezvous);
                    agent.setState(RealAgent.ExploreState.ReturnToParent);
                    agent.setStateTimer(0);

                    
                    agent.setCurrentGoal(agent.getParentRendezvous().getChildLocation());
                    return ((Point) agent.getPath().getPoints().remove(0));
                }
                // relay must be waiting for us, we are near rv, have new info - try to comm with relay
                if (((pathToParentRendezvous.getLength() / Constants.DEFAULT_SPEED) + timeElapsed) >= 
                        agent.getParentRendezvous().getMinTimeMeeting()) {
                    if (pathToParentRendezvous.getLength() < 100) {
                        System.out.println(agent + " returning to relay early.");
                        if (agent.getPath() != null) {
                            agent.addDirtyCells(agent.getPath().getAllPathPixels());
                        }
                        agent.setPath(pathToParentRendezvous);
                        agent.setState(RealAgent.ExploreState.ReturnToParent);
                        agent.setStateTimer(0);


                        agent.setCurrentGoal(agent.getParentRendezvous().getChildLocation());
                        return ((Point) agent.getPath().getPoints().remove(0));
                    }
                }
                
            } else
            {
                System.out.println(agent.toString() + "!!!Cannot plan path to parent RV!!! - will continue to explore and try again in " + Constants.CHECK_INTERVAL_TIME_TO_RV);    
            }
        }
        // </editor-fold>   

        //if we reach this point we continue exploring
        Point nextStep = FrontierExploration.takeStep(agent, timeElapsed, SimulatorConfig.frontiertype.ReturnWhenComplete, simConfig);
        
        //<editor-fold defaultstate="collapsed" desc="If there are no frontiers to explore, we must be finished.  Return to ComStation.">
        if ((agent.getFrontiers().isEmpty() || (agent.getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))) {
            Path pathToParentRendezvous = agent.calculatePath(agent.getLocation(), agent.getParentRendezvous().getChildLocation());
            agent.setPath(pathToParentRendezvous);
            agent.setState(RealAgent.ExploreState.ReturnToParent);
            agent.setStateTimer(0);


            agent.setCurrentGoal(agent.getParentRendezvous().getChildLocation());
            return ((Point) agent.getPath().getPoints().remove(0));
                    /*
            agent.setMissionComplete();
            agent.setParentRendezvous(new RVLocation(agent.getTeammate(Constants.BASE_STATION_ID).getLocation()));
            agent.addDirtyCells(agent.getPath().getAllPathPixels());
            Path path = agent.calculatePath(agent.getLocation(), agent.getParentRendezvous().getChildLocation());
            agent.setPath(path);
            agent.setState(RealAgent.ExploreState.ReturnToParent);
            agent.setStateTimer(0);
            
            if(agent.getPath().getPoints() != null) {
                agent.setCurrentGoal(agent.getParentRendezvous().getChildLocation());
                return((Point)agent.getPath().getPoints().remove(0));
            }
            else {
                System.out.println(agent.toString() + "!!!Nothing left to explore, but cannot plan path to parent!!!");
                nextStep = RandomWalk.takeStep(agent);
                agent.setCurrentGoal(nextStep);
                return(nextStep);
            }*/
        }
        //</editor-fold>
        return nextStep;
    }
    
    private static Point takeStep_ReturnToParent(RealAgent agent, SimulatorConfig simConfig) { 
        //<editor-fold defaultstate="collapsed" desc="If parent is in range, GiveParentInfo">
        if(agent.getParentTeammate().isInRange()) {
            agent.setState(RealAgent.ExploreState.GiveParentInfo);
            agent.setStateTimer(0);
            return takeStep_GiveParentInfo(agent, simConfig);
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="If mission complete and we have path, keep going">
        if(agent.isMissionComplete() && !agent.getPath().getPoints().isEmpty())
            return((Point)agent.getPath().getPoints().remove(0));
        //</editor-fold>
        
        //<editor-fold defaultstate="collapsed" desc="Recalculate path every PATH_RECALC_PARENT_INTERVAL steps, if fail try A*, if that fails try using existing path, if that fails take random step">
        Path existingPath = agent.getPath();
        if((agent.getStateTimer() % Constants.PATH_RECALC_PARENT_INTERVAL) == (Constants.PATH_RECALC_PARENT_INTERVAL - 1)) {
            //<editor-fold defaultstate="collapsed" desc="If path already exists, update dirty cells with that path">
            if (existingPath != null)
                agent.addDirtyCells(existingPath.getAllPathPixels());
            //</editor-fold>
            Path path = agent.calculatePath(agent.getLocation(), agent.getParentRendezvous().getChildLocation());
            //<editor-fold defaultstate="collapsed" desc="If path not found, try A*">
            if (!path.found)
            {
                System.out.println(agent.toString() + "ERROR!  Could not find full path! Trying pure A*");
                path = agent.calculatePath(agent.getLocation(), agent.getParentRendezvous().getChildLocation(), true);
            }
            //</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="If path still not found, try existing path. If existing path doesn't exist or exhausted, take random step">
            if(!path.found) {
                System.out.println(agent.toString() + "!!!ERROR!  Could not find full path!");
                if ((existingPath != null) && (existingPath.getPoints().size() > 2))
                {
                    agent.setPath(existingPath);
                    agent.setCurrentGoal(existingPath.getGoalPoint());
                } else
                {
                    agent.setCurrentGoal(agent.getLocation());
                    return RandomWalk.takeStep(agent);
                }
            }
            //</editor-fold>
            else {
                agent.setPath(path);
                agent.setCurrentGoal(agent.getParentRendezvous().getChildLocation());
                // Must remove first point in path as this is robot's location.
                agent.getPath().getPoints().remove(0);
            }
        }
        //</editor-fold>
        
        if(agent.getPath().found && !agent.getPath().getPoints().isEmpty())
            return((Point)agent.getPath().getPoints().remove(0));
        
        // If we reach this point, we are not in range of the parent and the
        // path is empty, so we must have reached rendezvous point.
        // Just check to make sure we are though and if not take random step.
        if(agent.getLocation().distance(agent.getParentRendezvous().getChildLocation()) > 2*Constants.STEP_SIZE)
        {
            System.out.println(agent.toString() + "!!!ERROR! We should have reached parent RV, but we are too far from it! Taking random step");
            return RandomWalk.takeStep(agent);
        }
        
        // If we reach this point, we're at the rendezvous point and waiting.
        agent.setState(RealAgent.ExploreState.WaitForParent);
        agent.setStateTimer(0);        
        return new Point(agent.getX(), agent.getY());
    }
    
    private static Point takeStep_WaitForParent(RealAgent agent, SimulatorConfig simConfig) {
        //<editor-fold defaultstate="collapsed" desc="If parent is in range, GiveParentInfo">
        if(agent.getParentTeammate().isInRange()) {
            agent.setState(RealAgent.ExploreState.GiveParentInfo);
            agent.setStateTimer(0);
            return takeStep_GiveParentInfo(agent, simConfig);
        }
        //</editor-fold>

        boolean canStillWait = (timeElapsed <= 
                (agent.getParentRendezvous().getTimeMeeting() + agent.getParentRendezvous().getTimeWait()));
        boolean throughWall = isParentRVThroughWall(agent);
        //<editor-fold defaultstate="collapsed" desc="If replanning is allowed, and we have been waiting longer than agreed, Explore">
        if(allowReplanning && (!canStillWait) && !throughWall) {
            agent.setState(RealAgent.ExploreState.Explore);
            agent.setStateTimer(0);
            return takeStep_Explore(agent, simConfig);
        }
        //</editor-fold>
        
        //<editor-fold defaultstate="collapsed" desc="If RV is through a wall, and we can still wait, move closer to wall, else go2backupRV">
        if (throughWall) {
            if (canStillWait) {
                //<editor-fold defaultstate="collapsed" desc="Try to move as close as possible to the wall">
                Point point1 = agent.getLocation();
                Point point2 = agent.getParentRendezvous().getParentLocation();
                double distance = point1.distance(point2);
                double min_step = 1 / distance;
                double cur_ratio = min_step;
                double cur_distance = 0;
                Point newPoint = new Point();
                while (cur_distance < Constants.DEFAULT_SPEED) {
                    newPoint.x = point1.x + (int)((point2.x - point1.x)*cur_ratio);
                    newPoint.y = point1.y + (int)((point2.y - point1.y)*cur_ratio);
                    if (!agent.getOccupancyGrid().directLinePossible(point1.x, point1.y, newPoint.x, newPoint.y)) {
                        break;
                    } else
                    {
                        cur_ratio += min_step;
                        cur_distance = point1.distance(newPoint);
                    }
                }
                cur_ratio = cur_ratio - min_step;
                newPoint.x = point1.x + (int)((point2.x - point1.x)*cur_ratio);
                newPoint.y = point1.y + (int)((point2.y - point1.y)*cur_ratio);
                //</editor-fold>
                return newPoint;
            } else
            {
                agent.setParentRendezvous(agent.getParentBackupRendezvous());
                agent.setParentBackupRendezvous(null);
                agent.setState(RealAgent.ExploreState.ReturnToParent);
                agent.setStateTimer(0);
                return takeStep_ReturnToParent(agent, simConfig);
            }
        }
        //</editor-fold>
            
        return new Point(agent.getX(), agent.getY());
    }

    private static Point takeStep_GiveParentInfo(RealAgent agent, SimulatorConfig simConfig) {
        // We've exchanged info, what's next
        //<editor-fold defaultstate="collapsed" desc="If mission complete, make ComStation parent, return to it">
        if(agent.isMissionComplete()) {
            agent.setParent(Constants.BASE_STATION_TEAMMATE_ID);
            agent.setParentRendezvous(new RVLocation(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation()));
            agent.addDirtyCells(agent.getPath().getAllPathPixels());
            Path path = agent.calculatePath(agent.getLocation(), agent.getParentRendezvous().getChildLocation());
            agent.setPath(path);
            // must remove first point as this is agent's location
            agent.getPath().getPoints().remove(0);
            agent.setState(RealAgent.ExploreState.ReturnToParent);
            agent.setStateTimer(0);
            
            if(agent.getPath().getPoints().size() > 0){
                agent.setCurrentGoal(agent.getParentRendezvous().getChildLocation());
                return((Point)agent.getPath().getPoints().remove(0));
            }
            else {
                agent.setCurrentGoal(agent.getLocation());
                return(agent.getLocation());
            }
        }        
        //</editor-fold>
        
        //<editor-fold defaultstate="collapsed" desc="If we just got into range, recalc next RV, exchange info">
        if(agent.getStateTimer() == 0) {
            //<editor-fold defaultstate="collapsed" desc="Case 1: Explorer">
            if(agent.isExplorer()) {
                // First, plan next frontier, as we need this for rendezvous point calculation
                FrontierExploration.replan(agent, SimulatorConfig.frontiertype.ReturnWhenComplete, 0, simConfig);
                
                // Second, calculate rendezvous, but stick around for one time step to communicate
                if(useImprovedRendezvous) {
                    calculateRendezvous(agent);
                    if (rvThroughWalls && timeElapsed > 100) {
                        if (agent.getTimeSinceLastRVCalc() == 0)
                            agent.setTimeSinceLastRVCalc(100);
                        agent.setParentBackupRendezvous(agent.getParentRendezvous());
                        calculateRVThroughWalls(agent);
                    }
                }
                else
                    agent.setParentRendezvous(agent.getChildRendezvous());  
                
                agent.setStateTimer(1);
                
                return agent.getLocation();
            }
//</editor-fold>
            
            //<editor-fold defaultstate="collapsed" desc="Case 2: Relay with another relay as parent">
            else if(agent.getParent() != Constants.BASE_STATION_TEAMMATE_ID) {
                calculateRendezvous2(agent);
                agent.setStateTimer(1);
                
                return agent.getLocation();
            }
            //</editor-fold>
            
            //<editor-fold defaultstate="collapsed" desc="Case 3: Relay with base station as parent, no need to recalculate rv">
            else {
                agent.setStateTimer(1);
                return agent.getLocation();
            }
            //</editor-fold>
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="else, we've recalculated rv, time to move on">
        else {
            //<editor-fold defaultstate="collapsed" desc="Explorer">
            if(agent.isExplorer()) {
                agent.setState(RealAgent.ExploreState.Explore);
                agent.setStateTimer(0);
                
                // we don't do this in the 0th state time, because agents have not communicated yet by then
                //Calculate time to next RV, taking parent communication range into account (using simple circle model)
                calculateParentTimeToRV(agent);
                calculateParentTimeToBackupRV(agent);
                
                
                return takeStep_Explore(agent, simConfig);
            }
            //</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="Relay">
            else {
                agent.setState(RealAgent.ExploreState.GoToChild);
                agent.setStateTimer(0);
                agent.addDirtyCells(agent.getPath().getAllPathPixels());
                Path path = agent.calculatePath(agent.getLocation(), agent.getChildRendezvous().getParentLocation());
                agent.setPath(path);
                agent.setCurrentGoal(agent.getChildRendezvous().getParentLocation());
                
                if(path.found)
                {
                    // stay in comm range with base, till it's time to go meet child
                    /*double estTimeToRV = (path.getLength() / Constants.DEFAULT_SPEED) + timeElapsed;
                    if (estTimeToRV < agent.getChildRendezvous().getTimeMeeting())
                    {
                        agent.setState(RealAgent.ExploreState.GiveParentInfo);
                        agent.setStateTimer(0);
                        return agent.getLocation();
                    } else*/
                        return((Point)agent.getPath().getPoints().remove(0));
                }
                else
                {
                    //<editor-fold defaultstate="collapsed" desc="If path not found, try A*">
                    System.out.println(agent.toString() + "ERROR!  Could not find full path! Trying pure A*");
                    path = agent.calculatePath(agent.getLocation(), agent.getChildRendezvous().getParentLocation(), true);
                    //</editor-fold>
                    //<editor-fold defaultstate="collapsed" desc="If path still not found, take random step">
                    if(!path.found) {
                        System.out.println(agent.toString() + "!!!ERROR!  Could not find full path! Taking random step");
                        agent.setCurrentGoal(agent.getLocation());
                        return RandomWalk.takeStep(agent);
                    } else
                    {
                        System.out.println(agent.toString() + "Pure A* worked");
                        agent.setPath(path);
                        agent.setCurrentGoal(agent.getChildRendezvous().getParentLocation());
                        return((Point)agent.getPath().getPoints().remove(0));
                    }
                    //</editor-fold>
                }
            }
            //</editor-fold>
        }
        //</editor-fold>
    }
    
    private static Point takeStep_GoToChild(RealAgent agent, SimulatorConfig simConfig) {      
        //<editor-fold defaultstate="collapsed" desc="Check if we are in range of child. If yes, GetInfoFromChild">
        if((agent.getChildTeammate().isInRange()) && !agent.getParentTeammate().isInRange()) {
            agent.setState(RealAgent.ExploreState.GetInfoFromChild);
            agent.setStateTimer(0);
            
            return takeStep_GetInfoFromChild(agent, simConfig);
        }
        //</editor-fold>
        
        //<editor-fold defaultstate="collapsed" desc="Assume that a path back to child has been calculated in previous state, recalculate every PATH_RECALC_CHILD_INTERVAL steps">
        Path existingPath = agent.getPath();
        if(((agent.getStateTimer() % Constants.PATH_RECALC_CHILD_INTERVAL) == (Constants.PATH_RECALC_CHILD_INTERVAL - 1))) {
            if (existingPath != null)
                agent.addDirtyCells(existingPath.getAllPathPixels());
            Path path = agent.calculatePath(agent.getLocation(), agent.getChildRendezvous().getParentLocation());
            //<editor-fold defaultstate="collapsed" desc="Could not find full path! Trying pure A*">
            if (!path.found)
            {
                System.out.println(agent.toString() + "!!!ERROR!  Could not find full path! Trying pure A*");
                path = agent.calculatePath(agent.getLocation(), agent.getChildRendezvous().getParentLocation(), true);
            }
            //</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="Still couldn't find path, trying existing path or if that fails, taking random step">
            if(!path.found)
            {
                System.out.println(agent.toString() + "!!!ERROR!  Could not find full path!");
                if ((existingPath != null) && (existingPath.getPoints().size() > 2))
                {
                    agent.setPath(existingPath);
                    agent.setCurrentGoal(existingPath.getGoalPoint());
                } else
                {
                    agent.setCurrentGoal(agent.getLocation());
                    return RandomWalk.takeStep(agent);
                }
                //</editor-fold>
            } else
            {
                agent.setPath(path);
                agent.setCurrentGoal(agent.getChildRendezvous().getParentLocation());
                // Must remove first point in path as this is robot's location.
                agent.getPath().getPoints().remove(0);
            }
        }
        //</editor-fold>
        
        if(agent.getPath().found && !agent.getPath().getPoints().isEmpty())
            return((Point)agent.getPath().getPoints().remove(0));
        
        // If we reach this point, we are not in range of the child and the
        // path is empty, so we must have reached rendezvous point.
        // Just check to make sure we are though and if not take random step.
        
        if(agent.getLocation().distance(agent.getChildRendezvous().getParentLocation()) > 2*Constants.STEP_SIZE)
        {
            System.out.println(agent.toString() + "!!!ERROR! We should have reached child RV, but we are too far from it! Taking random step");
            return RandomWalk.takeStep(agent);
        }
        
        // If we reach this point, we're at the rendezvous point and waiting.
        agent.setState(RealAgent.ExploreState.WaitForChild);
        agent.setStateTimer(0);
        
        return new Point(agent.getX(), agent.getY());
    }
    
    private static Point takeStep_WaitForChild(RealAgent agent, SimulatorConfig simConfig) {
        if(agent.getChildTeammate().isInRange()) {
            agent.setState(RealAgent.ExploreState.GetInfoFromChild);
            agent.setStateTimer(0);
            return takeStep_GetInfoFromChild(agent, simConfig);
        }
        
        boolean canStillWait = (timeElapsed <= 
                (agent.getChildRendezvous().getTimeMeeting() + agent.getChildRendezvous().getTimeWait()));
        boolean throughWall = isChildRVThroughWall(agent);
        //<editor-fold defaultstate="collapsed" desc="Uncomment this if relays should explore instead of waiting forever">
        /*if(allowReplanning && (!canStillWait) && !throughWall) {
            agent.setState(RealAgent.ExploreState.Explore);
            agent.setStateTimer(0);
            return takeStep_Explore(agent, simConfig);
        }*/
        //</editor-fold>
        
        //<editor-fold defaultstate="collapsed" desc="If RV is through a wall, and we can still wait, move closer to wall, else go2backupRV">
        if (throughWall) {
            if (canStillWait) {
                //<editor-fold defaultstate="collapsed" desc="Try to move as close as possible to the wall">
                Point point1 = agent.getLocation();
                Point point2 = agent.getChildRendezvous().getChildLocation();
                double distance = point1.distance(point2);
                double min_step = 1 / distance;
                double cur_ratio = min_step;
                double cur_distance = 0;
                Point newPoint = new Point();
                while (cur_distance < Constants.DEFAULT_SPEED) {
                    newPoint.x = point1.x + (int)((point2.x - point1.x)*cur_ratio);
                    newPoint.y = point1.y + (int)((point2.y - point1.y)*cur_ratio);
                    if (!agent.getOccupancyGrid().directLinePossible(point1.x, point1.y, newPoint.x, newPoint.y)) {
                        break;
                    } else
                    {
                        cur_ratio += min_step;
                        cur_distance = point1.distance(newPoint);
                    }
                }
                cur_ratio = cur_ratio - min_step;
                newPoint.x = point1.x + (int)((point2.x - point1.x)*cur_ratio);
                newPoint.y = point1.y + (int)((point2.y - point1.y)*cur_ratio);
                //</editor-fold>
                return newPoint;
            } else
            {
                agent.setChildRendezvous(agent.getChildBackupRendezvous());
                agent.setChildBackupRendezvous(null);
                agent.setState(RealAgent.ExploreState.GoToChild);
                agent.setStateTimer(0);
                return takeStep_GoToChild(agent, simConfig);
            }
        }
        //</editor-fold>
            
        return new Point(agent.getX(), agent.getY());
    }
    
    private static Point takeStep_GetInfoFromChild(RealAgent agent, SimulatorConfig simConfig) {
        //we've exchanged info, now return to parent (but wait for one timestep to learn new RV point)
        
        if(agent.getStateTimer() == 0) {
            agent.addDirtyCells(agent.getPath().getAllPathPixels());
            Path path = agent.calculatePath(agent.getLocation(), agent.getParentRendezvous().getChildLocation());

            agent.setPath(path);
            agent.setStateTimer(1);
            agent.setCurrentGoal(agent.getParentRendezvous().getChildLocation());
            return agent.getLocation();
        }
        else {
            agent.setState(RealAgent.ExploreState.ReturnToParent);
            calculateOwnTimeToRV(agent);
            calculateOwnTimeToBackupRV(agent);
            return takeStep_ReturnToParent(agent, simConfig);
        }
    }
    
// </editor-fold>  
    
// <editor-fold defaultstate="collapsed" desc="Calculate rendezvous">
    
    private static void calculateRendezvous(RealAgent agent) {
        // Only calculate rv every several time steps at most
        if(agent.getTimeSinceLastRVCalc() < 15)
            return;
        else
            agent.setTimeSinceLastRVCalc(0);

        long realtimeStart = System.currentTimeMillis();
        long intermediateTime1, intermediateTime2, intermediateTime3, intermediateTime4;
        System.out.println(agent.toString() + "Calculating next rendezvous ... ");

        try{
            PrintWriter outFile = new PrintWriter(new FileWriter("Skeletonisation.txt", true));

            outFile.print(timeElapsed + " " + agent.getName() + " ");


            intermediateTime1 = System.currentTimeMillis();
            System.out.print(Constants.INDENT + "Calculating skeleton ... ");
            int[][] skeletonGrid = Skeleton.findSkeleton(agent.getOccupancyGrid());
            intermediateTime2 = System.currentTimeMillis();
            System.out.println("complete, took " + (intermediateTime2 - intermediateTime1) + " ms.");
            outFile.print((intermediateTime2 - intermediateTime1) + " ");

            System.out.print(Constants.INDENT + "Finding distinct RV points ... ");
            LinkedList<Point> allSkeletonPoints = Skeleton.gridToList(skeletonGrid);
            LinkedList<Point> pts = Skeleton.findRendezvousPoints(skeletonGrid, agent.getOccupancyGrid());
            intermediateTime3 = System.currentTimeMillis();
            System.out.println("complete, took " + (intermediateTime3 - intermediateTime2) + " ms.");
            outFile.print((intermediateTime3 - intermediateTime2) + " ");

            //System.out.println("          Initial functions took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
            //intermediateTime1 = System.currentTimeMillis();

            System.out.println(Constants.INDENT + "Choosing specific RV point ... ");
            if(pts == null) 
                agent.setChildRendezvous(new RVLocation(agent.getLocation()));
            else {
                agent.setSkeleton(allSkeletonPoints);
                agent.setRVPoints(pts);

                LinkedList<Point> rvPtsCopy = new LinkedList();
                for(Point p: pts)
                    rvPtsCopy.add(new Point(p.x, p.y));

                int bestDegree;

                //intermediateTime2 = System.currentTimeMillis();
                //System.out.println("          Stage 1 took " + (intermediateTime2 - intermediateTime1) + "ms.");



                PriorityQueue<NearRVPoint> nearRVPoints = new PriorityQueue<NearRVPoint>();
                Point frontierCentre;
                if(agent.getLastFrontier() != null)
                    frontierCentre = agent.getLastFrontier().getClosestPoint(agent.getLocation(), agent.getOccupancyGrid());
                else
                {
                    System.out.println(agent + " !!!! getLastFrontier returned null, setting frontierCentre to " + agent.getLocation());
                    frontierCentre = agent.getLocation();
                }
                System.out.println(agent + " frontierCentre is " + frontierCentre);
                // create priority queue of all potential rvpoints within given straight line distance
                for(Point p: rvPtsCopy) {
                    if(p.distance(frontierCentre) > 600)
                        continue;

                    nearRVPoints.add(new NearRVPoint(p.x, p.y, p.distance(frontierCentre)));
                }

                // reduce to up to 4 points closest to next frontier, calculate exact path cost
                NearRVPoint tempPoint;
                double pathCost;
                int degree;
                PriorityQueue<Point> prunedNearRvPts = new PriorityQueue();
                for(int i=0; i<4; i++)
                    if(nearRVPoints.size() > 0) {
                        tempPoint = nearRVPoints.remove();
                        pathCost = agent.calculatePath(tempPoint, frontierCentre).getLength();
                        degree = Skeleton.neighborTraversal(skeletonGrid, tempPoint.x, tempPoint.y);
                        prunedNearRvPts.add(new NearRVPoint(tempPoint.x, tempPoint.y, pathCost, degree));
                    }

                Point bestPoint;
                if(prunedNearRvPts.size() > 0)
                    bestPoint = prunedNearRvPts.remove();
                else
                {
                    System.out.println(agent + " !!!!! error pruning - bestpoint is set to " + agent.getParentTeammate().getLocation());
                    bestPoint = agent.getParentTeammate().getLocation();
                }

                // OUCH THIS WHOLE FUNCTION IS SUPER MESSY MUST CLEAN IT UP!!!

                // Keep only the 3 closest
                /*LinkedList<Point> prunedNearRvPts2 = new LinkedList();
                for(int i=0; i<3; i++)
                    prunedNearRvPts2.add(prunedNearRvPts.remove());

                // Choose the one having highest degree
                bestDegree = 0;
                LinkedList<Point> highestDegreePts = new LinkedList();
                for(Point p: prunedNearRvPts2)
                    if(Skeleton.neighborTraversal(skeletonGrid, p.x, p.y) > bestDegree) {
                        highestDegreePts = new LinkedList();
                        highestDegreePts.add(p);
                        bestDegree = Skeleton.neighborTraversal(skeletonGrid, p.x, p.y);
                    }
                    else if(Skeleton.neighborTraversal(skeletonGrid, p.x, p.y) == bestDegree) {
                        highestDegreePts.add(p);
                    }*/

                //intermediateTime2 = System.currentTimeMillis();
                //System.out.println("          Stage 3 took " + (intermediateTime2 - intermediateTime1) + "ms.");

                //Point bestPoint = new Point();
                // if there is a single point with best degree, choose it
                ///if(highestDegreePts.size() == 1) {
                //    bestPoint = highestDegreePts.getFirst();
                //}
            // else {
                // To differentiate ties, choose best comm range


                    /* 20 April 2010 Julian:  this can hog time, and points are usually very close
                    * so will temporarily comment this out and return first in list 
                    for(Point p: highestDegreePts) {
                            currRange = PropModel1.getRangeEstimate(agent, p);
                            if(currRange > bestRange) {
                                bestRange = currRange;
                                bestPoint = p;
                            }                
                        }
                    /* end 20 April 2010 Julian comment */
            //     bestPoint = highestDegreePts.getFirst();
                //}
                //intermediateTime1 = System.currentTimeMillis();
                //System.out.println("          Stage 4 took " + (intermediateTime1 - intermediateTime2) + "ms.");

                agent.setParentRendezvous(new RVLocation(bestPoint));

                /* THIS LINE TEMP FOR SIMPLE ENVS */
                //agent.setParentRendezvous(agent.getLastFrontier().getClosestPoint(agent.getLocation(), agent.getOccupancyGrid()));
            }
            intermediateTime4 = System.currentTimeMillis();
            outFile.print((intermediateTime4 - intermediateTime3) + " ");

            System.out.print(Constants.INDENT + "Choosing complete, chose " + 
                    agent.getParentRendezvous().getChildLocation().x + "," + 
                    agent.getParentRendezvous().getChildLocation().y + ". ");
            System.out.println("Took " + (intermediateTime4 - intermediateTime3) + "ms.");
            System.out.println(Constants.INDENT + "Complete RV calculation process took " + 
                    (System.currentTimeMillis()-realtimeStart) + "ms.");

            outFile.println();
            outFile.close();
            
        }
        catch(IOException e){
            System.out.println("Skeletonisation timing log - error writing data to file!" + e);
        }
    }
    
    private static void calculateRendezvous2(RealAgent agent) {
        long realtimeStart = System.currentTimeMillis();
        System.out.println(agent.toString() + "Calculating next rendezvous2 ... ");

        int[][] skeletonGrid = Skeleton.findSkeleton(agent.getOccupancyGrid());
        LinkedList<Point> allSkeletonPoints = Skeleton.gridToList(skeletonGrid);
        LinkedList<Point> pts = Skeleton.findRendezvousPoints(skeletonGrid, agent.getOccupancyGrid());
        
        System.out.println("          Initial functions took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        
        if(pts == null) 
            agent.setParentRendezvous(new RVLocation(agent.getLocation()));

        else {
            agent.setSkeleton(allSkeletonPoints);
            agent.setRVPoints(pts);
            
            // calculate path from base station to own childrendezvous
            // NOTE this is for branch depth 3, greater depth needs to calculate
            // path from parent's parentrendezvous (must be communicated) to own childrendezvous
            Path pathToChild = agent.calculatePath(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation(), 
                    agent.getChildRendezvous().getParentLocation());
            Point middle = (Point)pathToChild.getPoints().get((int)(pathToChild.getPoints().size()/2));
            double pathCost;
            double bestPathCost = Double.MAX_VALUE;
            Point bestPoint = agent.getLocation();
            
            for(Point p: pts) {
                //skip points that are too far anyway
                if(p.distance(middle) > 100)
                    continue;
                pathCost = agent.calculatePath(p, middle).getLength();
                if(pathCost < bestPathCost) {
                    bestPathCost = pathCost;
                    bestPoint = p;
                }
            }
            
            agent.setParentRendezvous(new RVLocation(bestPoint));
        }
        
        System.out.print(" -Chose RV at " + agent.getParentRendezvous().getChildLocation().x + "," + 
                agent.getParentRendezvous().getChildLocation().y + ". ");
        System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
    
    //Calculate time to next RV with parent, taking parent communication range into account (using simple circle model)
    private static void calculateParentTimeToRV(RealAgent agent)
    {
        System.out.println(agent.toString() + "Calculating time to next rendezvous...");
        
        //<editor-fold defaultstate="collapsed" desc="let's find the point, where RV will actually communicate with base">
            Point baseLoc = agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation();
            Point relayLoc = agent.getParentTeammate().getLocation();
            Point baseComm = baseLoc;
            OccupancyGrid occGrid = agent.getOccupancyGrid();
            
            Polygon commPoly = PropModel1.getRangeForRV(occGrid,
                    new BasicAgent(0, "", 0, baseLoc.x, baseLoc.y, 0, 0, 400, 0,
                            RobotConfig.roletype.Relay, 0, 0, 0)
            );
            
            LinkedList<Point> candidatePoints = new LinkedList<Point>();
            //for(Point p : ExplorationImage.polygonPoints(commPoly))
            for (int i = 0; i < commPoly.npoints; i++)
            {
                Point p = new Point(commPoly.xpoints[i], commPoly.ypoints[i]);
                if (occGrid.freeSpaceAt(p.x, p.y) /*&& !env.directLinePossible(firstRV.x, firstRV.y, p.x, p.y)*/)
                {                    
                    if (occGrid.directLinePossible(baseLoc.x, baseLoc.y, p.x, p.y))
                        candidatePoints.add(p);
                }
            }
            
            double minBaseRelayDistance = agent.calculatePath(baseLoc, relayLoc).getLength();
            
            for (Point p: candidatePoints)
            {
                double distance = agent.calculatePath(p, relayLoc).getLength();
                if (distance < minBaseRelayDistance)
                {
                    minBaseRelayDistance = distance;
                    baseComm = p;
                }
            }
            //</editor-fold>
        
        Path pathParentToCS = agent.calculatePath(agent.getParentTeammate().getLocation(),
                baseComm);
        Path pathCSToRendezvous = agent.calculatePath(baseComm,
                agent.getParentRendezvous().getParentLocation());
        
        //<editor-fold defaultstate="collapsed" desc="Couldn't find pathCSToRV - approximate">
        if ((pathCSToRendezvous.getLength() == 0) &&
                (!agent.getParentRendezvous().getParentLocation().equals(
                        agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation())))
        {
            System.out.println("Could not calculate pathCSToRendezvous!!!!");
            //let's at least set it to a rough approximation - better than setting it to 0!
            pathCSToRendezvous = pathParentToCS;
        }
        //</editor-fold>
        
        agent.setTimeUntilRendezvous(Math.max((int)((pathParentToCS.getLength()+pathCSToRendezvous.getLength())/Constants.DEFAULT_SPEED), 10));

        
        
        //<editor-fold defaultstate="collapsed" desc="Check time for explorer to reach frontier, to make sure he has time to explore before returning">
        Point frontierCentre = null;
        if(agent.getLastFrontier() != null)
            frontierCentre = agent.getLastFrontier().getClosestPoint(agent.getLocation(), agent.getOccupancyGrid());
        else
        {
            System.out.println(agent + " Setting frontierCentre to agent location");
            frontierCentre = agent.getLocation();
        }
        if (frontierCentre != null)
        {                
            Path here2Frontier = agent.calculatePath(agent.getLocation(), frontierCentre);
            Path front2rv = agent.calculatePath(frontierCentre, agent.getParentRendezvous().getChildLocation());
            int expTime = (int)(here2Frontier.getLength() + front2rv.getLength())/Constants.DEFAULT_SPEED;
            if (!agent.getParentRendezvous().getParentLocation().equals(agent.getParentRendezvous().getChildLocation()))
            {
                agent.getParentRendezvous().setMinTimeMeeting(Math.max(timeElapsed + agent.getTimeUntilRendezvous(), 
                        timeElapsed + expTime + 15));
                expTime += Constants.FRONTIER_MIN_EXPLORE_TIME;
            }
            agent.setTimeUntilRendezvous(Math.max(agent.getTimeUntilRendezvous(), expTime));
            System.out.println(agent + " here2Frontier: " + here2Frontier.getLength() + " front2rv " + front2rv.getLength() + " minExplore " + Constants.FRONTIER_MIN_EXPLORE_TIME);
        } else
        {
            System.out.println(agent + " frontier is null");
        }
        //</editor-fold>
        
        agent.getParentRendezvous().setTimeMeeting(timeElapsed + agent.getTimeUntilRendezvous());
        agent.getParentRendezvous().setTimeWait(Constants.WAIT_AT_RV_BEFORE_REPLAN);
        
        System.out.println("\nP2CS " + pathParentToCS.getLength() + "; " +
                " CS2R " + pathCSToRendezvous.getLength() + "; " +
                agent.getParentTeammate().getCommRange() + "; " +
                Constants.DEFAULT_SPEED);
        System.out.println(Constants.INDENT + "Assume that parent will take " + agent.getTimeUntilRendezvous() + " time steps until rendezvous.");
    }
    
    private static void calculateParentTimeToBackupRV(RealAgent agent)
    {
        if (agent.getParentBackupRendezvous() == null) return;
        //System.out.println(agent.toString() + "Calculating time to next rendezvous...");
        int timeAtStart = agent.getParentRendezvous().getTimeMeeting() + agent.getParentRendezvous().getTimeWait();
        
        Path pathMeToRV2 = agent.calculatePath(agent.getParentRendezvous().getChildLocation(),
                agent.getParentBackupRendezvous().getChildLocation());
        
        Path pathParentToRV2 = agent.calculatePath(agent.getParentRendezvous().getParentLocation(),
                agent.getParentBackupRendezvous().getParentLocation());
        
        if (pathMeToRV2.found && pathParentToRV2.found)
        {
            agent.getParentBackupRendezvous().setTimeMeeting(timeAtStart + 
                    Math.max((int)pathMeToRV2.getLength(), (int)pathParentToRV2.getLength())/Constants.DEFAULT_SPEED);
            agent.getParentBackupRendezvous().setMinTimeMeeting(timeAtStart + 
                    Math.max((int)pathMeToRV2.getLength(), (int)pathParentToRV2.getLength())/Constants.DEFAULT_SPEED);
            agent.getParentBackupRendezvous().setTimeWait(Constants.WAIT_AT_RV_BEFORE_REPLAN);
        } else
        {
            System.out.println(agent + "  !!!FAILED to calculate backup RV times!");
            agent.getParentBackupRendezvous().setTimeMeeting(Constants.MAX_TIME);
            agent.getParentBackupRendezvous().setTimeWait(Constants.MAX_TIME);
        }
    }
    
    //Calculate time to next RV (we are the relay)
    private static void calculateOwnTimeToRV(RealAgent agent)
    {
        //System.out.println(agent.toString() + "Calculating time to next rendezvous...");
        
        //<editor-fold defaultstate="collapsed" desc="let's find the point, where RV will actually communicate with base">
            Point baseLoc = agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation();
            Point relayLoc = agent.getLocation();
            Point baseComm = baseLoc;
            OccupancyGrid occGrid = agent.getOccupancyGrid();
            
            Polygon commPoly = PropModel1.getRangeForRV(occGrid,
                    new BasicAgent(0, "", 0, baseLoc.x, baseLoc.y, 0, 0, 400, 0,
                            RobotConfig.roletype.Relay, 0, 0, 0)
            );
            
            LinkedList<Point> candidatePoints = new LinkedList<Point>();
            //for(Point p : ExplorationImage.polygonPoints(commPoly))
            for (int i = 0; i < commPoly.npoints; i++)
            {
                Point p = new Point(commPoly.xpoints[i], commPoly.ypoints[i]);
                if (occGrid.freeSpaceAt(p.x, p.y) /*&& !env.directLinePossible(firstRV.x, firstRV.y, p.x, p.y)*/)
                {                    
                    if (occGrid.directLinePossible(baseLoc.x, baseLoc.y, p.x, p.y))
                        candidatePoints.add(p);
                }
            }
            
            double minBaseRelayDistance = agent.calculatePath(baseLoc, relayLoc).getLength();
            
            for (Point p: candidatePoints)
            {
                double distance = agent.calculatePath(p, relayLoc).getLength();
                if (distance < minBaseRelayDistance)
                {
                    minBaseRelayDistance = distance;
                    baseComm = p;
                }
            }
            //</editor-fold>
        
        Path pathParentToCS = agent.calculatePath(agent.getLocation(),
                baseComm);
        Path pathCSToRendezvous = agent.calculatePath(baseComm,
                agent.getChildRendezvous().getParentLocation());
        
        //<editor-fold defaultstate="collapsed" desc="Couldn't find pathCSToRV - approximate">
        if ((pathCSToRendezvous.getLength() == 0) &&
                (!agent.getChildRendezvous().getParentLocation().equals(
                        agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation())))
        {
            System.out.println("Could not calculate pathCSToRendezvous!!!!");
            //let's at least set it to a rough approximation - better than setting it to 0!
            pathCSToRendezvous = pathParentToCS;
        }
        //</editor-fold>
        
        agent.setTimeUntilRendezvous(Math.max((int)((pathParentToCS.getLength()+pathCSToRendezvous.getLength())
                /Constants.DEFAULT_SPEED), 10));

        
        //<editor-fold defaultstate="collapsed" desc="Check time for explorer to reach frontier, to make sure he has time to explore before returning">
        Point frontierCentre = agent.getChildTeammate().getFrontierCentre();
        if (frontierCentre != null)
        {                
            Path here2Frontier = agent.calculatePath(agent.getChildTeammate().getLocation(), frontierCentre);
            Path front2rv = agent.calculatePath(frontierCentre, agent.getChildRendezvous().getChildLocation());
            
            int expTime = (int)(here2Frontier.getLength() + front2rv.getLength())/Constants.DEFAULT_SPEED;
            if (!agent.getChildRendezvous().getParentLocation().equals(agent.getChildRendezvous().getChildLocation()))
            {
                agent.getChildRendezvous().setMinTimeMeeting(Math.max(timeElapsed + agent.getTimeUntilRendezvous(), 
                        timeElapsed + expTime + 15));
                expTime += Constants.FRONTIER_MIN_EXPLORE_TIME;
            }
            
            agent.setTimeUntilRendezvous(Math.max(agent.getTimeUntilRendezvous(), 
                    expTime));
        }
        //</editor-fold>
        
        agent.getChildRendezvous().setTimeMeeting(timeElapsed + agent.getTimeUntilRendezvous());        
        agent.getChildRendezvous().setTimeWait(Constants.WAIT_AT_RV_BEFORE_REPLAN);
        
        System.out.println("\nP2CS " + pathParentToCS.getLength() + "; " +
                " CS2R " + pathCSToRendezvous.getLength() + "; " +
                agent.getParentTeammate().getCommRange() + "; " +
                Constants.DEFAULT_SPEED);
        System.out.println(Constants.INDENT + "Assume that we will take " + agent.getTimeUntilRendezvous() + " time steps until rendezvous.");
    }
    
    private static void calculateOwnTimeToBackupRV(RealAgent agent)
    {
        //System.out.println(agent.toString() + "Calculating time to next rendezvous...");
        if (agent.getChildBackupRendezvous() == null) return;
        
        int timeAtStart = agent.getChildRendezvous().getTimeMeeting() + agent.getChildRendezvous().getTimeWait();
        
        Path pathMeToRV2 = agent.calculatePath(agent.getChildRendezvous().getParentLocation(),
                agent.getChildBackupRendezvous().getParentLocation());
        
        Path pathChildToRV2 = agent.calculatePath(agent.getChildRendezvous().getChildLocation(),
                agent.getChildBackupRendezvous().getChildLocation());
        
        if (pathMeToRV2.found && pathChildToRV2.found)
        {
            agent.getChildBackupRendezvous().setTimeMeeting(timeAtStart + 
                    Math.max((int)pathMeToRV2.getLength(), (int)pathChildToRV2.getLength())/Constants.DEFAULT_SPEED);
            agent.getChildBackupRendezvous().setMinTimeMeeting(timeAtStart + 
                    Math.max((int)pathMeToRV2.getLength(), (int)pathChildToRV2.getLength())/Constants.DEFAULT_SPEED);
            agent.getChildBackupRendezvous().setTimeWait(Constants.WAIT_AT_RV_BEFORE_REPLAN);
        } else
        {
            System.out.println(agent + "  !!!FAILED to calculate backup RV times!");
            agent.getChildBackupRendezvous().setTimeMeeting(Constants.MAX_TIME);
            agent.getChildBackupRendezvous().setTimeWait(Constants.MAX_TIME);
        }
    }
    
    public static void calculateRVThroughWalls(RealAgent agent) {
        // Only calculate rv every several time steps at most
        if(agent.getTimeSinceLastRVCalc() < 15)
            return;
        else
            agent.setTimeSinceLastRVCalc(0);
        
        long realtimeStart = System.currentTimeMillis();
        System.out.println(agent.toString() + "Calculating RV Through Walls");
        
        agent.forceUpdateTopologicalMap();
        
        agent.getTopologicalMap().generateSkeletonNearBorders();
        
        System.out.println(" 1 Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        realtimeStart = System.currentTimeMillis();
        
        agent.getTopologicalMap().findKeyPointsBorder();
        
        System.out.println(" 2 Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        realtimeStart = System.currentTimeMillis();
        
        agent.getTopologicalMap().findSecondKeyPointsBorder(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation(), 
                agent);
        
        System.out.println(" 3 Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        realtimeStart = System.currentTimeMillis();
        
        RVLocation newLocation = 
            agent.getTopologicalMap().findNearestBorderKeyPoint(agent.getParentRendezvous().getChildLocation(), agent);
        
        System.out.println(" 4 Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        realtimeStart = System.currentTimeMillis();
        
        
        agent.setParentRendezvous(newLocation);
    }
    
    public static boolean isParentRVThroughWall(RealAgent agent)
    {
        return !agent.getOccupancyGrid().directLinePossible(
                agent.getParentRendezvous().getChildLocation().x, agent.getParentRendezvous().getChildLocation().y, 
                agent.getParentRendezvous().getParentLocation().x, agent.getParentRendezvous().getParentLocation().y);
    }
    
    public static boolean isChildRVThroughWall(RealAgent agent)
    {
        return !agent.getOccupancyGrid().directLinePossible(
                agent.getChildRendezvous().getChildLocation().x, agent.getChildRendezvous().getChildLocation().y, 
                agent.getChildRendezvous().getParentLocation().x, agent.getChildRendezvous().getParentLocation().y);
    }
    
// </editor-fold>     

}
