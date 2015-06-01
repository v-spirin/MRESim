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


import exploration.rendezvous.Rendezvous;
import agents.BasicAgent;
import agents.RealAgent;
import config.Constants;
import config.SimulatorConfig;
import environment.*;
import exploration.rendezvous.IRendezvousStrategy;
import exploration.rendezvous.RendezvousAgentData;
import java.util.*;
import java.awt.*;
import java.util.concurrent.atomic.AtomicReference;
import path.Path;
import org.apache.commons.math3.random.SobolSequenceGenerator;

/**
 *
 * @author julh
 */

public class RoleBasedExploration {    

// <editor-fold defaultstate="collapsed" desc="Take Step">
    public static boolean rvCommRange;
    public static boolean rvThroughWalls;
    public static int timeElapsed;
    public static int oldTimeElapsed;
    
    // Returns new X, Y of ExploreAgent
    public static Point takeStep(RealAgent agent, int curTime, IRendezvousStrategy rendezvousStrategy) {
        long realtimeStart = System.currentTimeMillis();
        //<editor-fold defaultstate="collapsed" desc="Assign local variables">
        timeElapsed = curTime;   
        
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
            case Initial :          nextStep = takeStep_Initial(agent);
                                    break;
            case Explore :          nextStep = takeStep_Explore(agent);
                                    break;
            case ReturnToParent :   nextStep = takeStep_ReturnToParent(agent);
                                    break;
            case WaitForParent :    nextStep = takeStep_WaitForParent(agent);
                                    break;
            case GiveParentInfo :   nextStep = takeStep_GiveParentInfo(agent);
                                    break;
            case GoToChild :        nextStep = takeStep_GoToChild(agent);
                                    break;
            case WaitForChild :     nextStep = takeStep_WaitForChild(agent);
                                    break;
            case GetInfoFromChild : nextStep = takeStep_GetInfoFromChild(agent);
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
        agent.getRendezvousAgentData().setTimeSinceLastRoleSwitch(agent.getRendezvousAgentData().getTimeSinceLastRoleSwitch() + 1);
        agent.getRendezvousAgentData().setTimeSinceLastRVCalc(agent.getRendezvousAgentData().getTimeSinceLastRVCalc() + 1);
        //</editor-fold>
        System.out.println(agent.toString() + " takeStep " + agent.getState() + ", took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        return nextStep;
    }
    
    public static Point takeStep_Initial(RealAgent agent) {
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
                return FrontierExploration.replan(agent, SimulatorConfig.frontiertype.ReturnWhenComplete, 0);
            }
            else {
                agent.setState(RealAgent.ExploreState.GoToChild);
                agent.setStateTimer(0);
                return(agent.getLocation());
            }
        }
        // </editor-fold>
    }
    
    public static Point takeStep_Explore(RealAgent agent) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        RendezvousAgentData parentRvd = agent.getParentTeammate().getRendezvousAgentData();
        boolean haveNewRVDetailsForParent = !rvd.getParentRendezvous().equals(parentRvd.getChildRendezvous());
        boolean noRVAgreed = (rvd.getParentRendezvous().getTimeMeeting() == Constants.MAX_TIME);
        // <editor-fold defaultstate="collapsed" desc="If parent is in range, and we have been exploring for longer than MIN_TIME_IN_EXPLORE_STATE, give parent info.">
        if (agent.getParentTeammate().isInRange()) {
            if ((agent.getStateTimer() > Constants.MIN_TIME_IN_EXPLORE_STATE) || 
                    // we have a new RV point, and must communicate it to the parent - otherwise the parent will be waiting at the old RV forever!
                    haveNewRVDetailsForParent || noRVAgreed) 
            {
                System.out.println(agent + "[" + agent.getID() + "]" + " is in range with parent " + agent.getParentTeammate() +
                        ", switching state to GiveParentInfo");
                agent.setState(RealAgent.ExploreState.GiveParentInfo);
                agent.setStateTimer(0);
                return(takeStep_GiveParentInfo(agent));
            }
        }
        //else
        //    agent.setTimeUntilRendezvous(agent.getTimeUntilRendezvous() - 1);
        // </editor-fold>
        // <editor-fold defaultstate="collapsed" desc="Every CHECK_INTERVAL_TIME_TO_RV steps, check if we're due to meet our parent again (unless parent is basestation, in which case explore continuously)">
        if (agent.getParent() != Constants.BASE_STATION_TEAMMATE_ID
                && (agent.getStateTimer() % Constants.CHECK_INTERVAL_TIME_TO_RV) == (Constants.CHECK_INTERVAL_TIME_TO_RV - 1)) 
        {
            Path pathToParentRendezvous = null; //output parameter
            AtomicReference<Path> outPathRef = new AtomicReference<Path>();
            if (isDueToReturnToRV(agent, outPathRef)) {
                pathToParentRendezvous = outPathRef.get();
                agent.getRendezvousStrategy().processExplorerStartsHeadingToRV();
                
                if (agent.getPath() != null) {
                    agent.addDirtyCells(agent.getPath().getAllPathPixels());
                }
                
               if ((pathToParentRendezvous == null) || (!pathToParentRendezvous.found) ||
                       (pathToParentRendezvous.getPoints().isEmpty())) {
                   //take random step and try again
                   agent.setStateTimer(Constants.CHECK_INTERVAL_TIME_TO_RV - 2);
                   return RandomWalk.takeStep(agent);
               } else {                
                    agent.setPath(pathToParentRendezvous);
                    agent.setState(RealAgent.ExploreState.ReturnToParent);
                    agent.setStateTimer(0);
                    agent.setCurrentGoal(rvd.getParentRendezvous().getChildLocation());
                    return ((Point) agent.getPath().getPoints().remove(0));
               }
            }
            // relay must be waiting for us, we are near rv, have new info - try to comm with relay
            /*else if (((pathToParentRendezvous.getLength() / Constants.DEFAULT_SPEED) + timeElapsed) >= 
                    agent.getRendezvousAgentData().getParentRendezvous().getMinTimeMeeting()) {
                if (pathToParentRendezvous.getLength() < 100) {
                    System.out.println(agent + " returning to relay early.");
                    if (agent.getPath() != null) {
                        agent.addDirtyCells(agent.getPath().getAllPathPixels());
                    }
                    agent.setPath(pathToParentRendezvous);
                    agent.setState(RealAgent.ExploreState.ReturnToParent);
                    agent.setStateTimer(0);


                    agent.setCurrentGoal(agent.getRendezvousAgentData().getParentRendezvous().getChildLocation());
                    return ((Point) agent.getPath().getPoints().remove(0));
                }
            }*/
        }
        // </editor-fold>   

        //if we reach this point we continue exploring
        Point nextStep = FrontierExploration.takeStep(agent, timeElapsed, SimulatorConfig.frontiertype.ReturnWhenComplete);
        
        //<editor-fold defaultstate="collapsed" desc="If there are no frontiers to explore, we must be finished.  Return to ComStation.">
        if ((agent.getFrontiers().isEmpty() || (agent.getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))) {
            Path pathToParentRendezvous = agent.calculatePath(agent.getLocation(), rvd.getParentRendezvous().getChildLocation());
            agent.setPath(pathToParentRendezvous);
            agent.setState(RealAgent.ExploreState.ReturnToParent);
            agent.setStateTimer(0);


            agent.setCurrentGoal(rvd.getParentRendezvous().getChildLocation());
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
    
    public static Point takeStep_ReturnToParent(RealAgent agent) { 
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //<editor-fold defaultstate="collapsed" desc="If parent is in range, GiveParentInfo">
        if(agent.getParentTeammate().isInRange()) {
            agent.setState(RealAgent.ExploreState.GiveParentInfo);
            agent.setStateTimer(0);
            return takeStep_GiveParentInfo(agent);
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
            
            agent.getRendezvousStrategy().processReturnToParentReplan();
            
            Path path = agent.calculatePath(agent.getLocation(), rvd.getParentRendezvous().getChildLocation());
            //<editor-fold defaultstate="collapsed" desc="If path not found, try A*">
            if (!path.found)
            {
                System.out.println(agent.toString() + "ERROR!  Could not find full path! Trying pure A*");
                path = agent.calculatePath(agent.getLocation(), rvd.getParentRendezvous().getChildLocation(), true);
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
                agent.setCurrentGoal(rvd.getParentRendezvous().getChildLocation());
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
        if(agent.getLocation().distance(rvd.getParentRendezvous().getChildLocation()) > 2*Constants.STEP_SIZE)
        {
            System.out.println(agent.toString() + "!!!ERROR! We should have reached parent RV, but we are too far from it! Taking random step");
            return RandomWalk.takeStep(agent);
        }
        
        // If we reach this point, we're at the rendezvous point and waiting.
        agent.setState(RealAgent.ExploreState.WaitForParent);
        agent.setStateTimer(0);        
        return new Point(agent.getX(), agent.getY());
    }
    
    public static Point takeStep_WaitForParent(RealAgent agent) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //<editor-fold defaultstate="collapsed" desc="If parent is in range, GiveParentInfo">
        if(agent.getParentTeammate().isInRange()) {
            agent.setState(RealAgent.ExploreState.GiveParentInfo);
            agent.setStateTimer(0);
            return takeStep_GiveParentInfo(agent);
        }
        //</editor-fold>
        
        boolean canStillWait = (RoleBasedExploration.timeElapsed <= 
                (rvd.getParentRendezvous().getTimeMeeting() + rvd.getParentRendezvous().getTimeWait()));
        
        if (canStillWait) {
            System.out.println(agent + "rvStrategy agent is " + agent.getRendezvousStrategy().getAgent());
            return agent.getRendezvousStrategy().processWaitForParent();
        } else
        {
            //Go to backup RV if available. We wil go into Explore state which will head to backup RV to arrive there
            //at the pre-arranged time.
            if (rvd.getParentBackupRendezvous() != null) {
                System.out.println(agent + " heading to backup rendezvous: " + rvd.getParentBackupRendezvous());
                rvd.setParentRendezvous(rvd.getParentBackupRendezvous());
                rvd.setTimeUntilRendezvous(rvd.getParentRendezvous().getTimeMeeting() - timeElapsed);
                agent.getRendezvousAgentData().setParentBackupRendezvous(null);
            }
            System.out.println(agent + " has no backup RV, exploring.");
            agent.setState(RealAgent.ExploreState.Explore);
            agent.setStateTimer(0);
            return takeStep_Explore(agent);
        }

        //<editor-fold defaultstate="collapsed" desc="If RV is through a wall, and we can still wait, move closer to wall, else go2backupRV">
        /*boolean throughWall = isParentRVThroughWall(agent);
        if (throughWall) {
            if (canStillWait) {
                //<editor-fold defaultstate="collapsed" desc="Try to move as close as possible to the wall">
                Point point1 = agent.getLocation();
                Point point2 = agent.getRendezvousAgentData().getParentRendezvous().getParentLocation();
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
                agent.getRendezvousAgentData().setParentRendezvous(rvd.getParentBackupRendezvous());
                agent.getRendezvousAgentData().setParentBackupRendezvous(null);
                agent.setState(RealAgent.ExploreState.ReturnToParent);
                agent.setStateTimer(0);
                return takeStep_ReturnToParent(agent);
            }
        }
        //</editor-fold>
            
        return new Point(agent.getX(), agent.getY());*/

    }

    public static Point takeStep_GiveParentInfo(RealAgent agent) {
        // We've exchanged info, what's next
        //<editor-fold defaultstate="collapsed" desc="If mission complete, make ComStation parent, return to it">
        if(agent.isMissionComplete()) {
            agent.setParent(Constants.BASE_STATION_TEAMMATE_ID);
            agent.getRendezvousAgentData().setParentRendezvous(new Rendezvous(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation()));
            agent.addDirtyCells(agent.getPath().getAllPathPixels());
            Path path = agent.calculatePath(agent.getLocation(), agent.getRendezvousAgentData().getParentRendezvous().getChildLocation());
            agent.setPath(path);
            // must remove first point as this is agent's location
            agent.getPath().getPoints().remove(0);
            agent.setState(RealAgent.ExploreState.ReturnToParent);
            agent.setStateTimer(0);
            
            if(agent.getPath().getPoints().size() > 0){
                agent.setCurrentGoal(agent.getRendezvousAgentData().getParentRendezvous().getChildLocation());
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
            System.out.println(agent + "give parent info.");
            return agent.getRendezvousStrategy().processJustGotIntoParentRange();
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="else, we've recalculated rv, time to move on">
        else {
            //<editor-fold defaultstate="collapsed" desc="Explorer - process & go into Explore state">
            if(agent.isExplorer()) {
                agent.getRendezvousStrategy().processAfterGiveParentInfoExplorer();
                
                agent.setState(RealAgent.ExploreState.Explore);
                agent.setStateTimer(0);               
                
                return takeStep_Explore(agent);
            }
            //</editor-fold>
            //<editor-fold defaultstate="collapsed" desc="Relay - process & go to child">
            else {
                agent.getRendezvousStrategy().processAfterGiveParentInfoRelay();
                
                agent.setState(RealAgent.ExploreState.GoToChild);
                agent.setStateTimer(0);
                agent.addDirtyCells(agent.getPath().getAllPathPixels());
                Path path = agent.calculatePath(agent.getLocation(), agent.getRendezvousAgentData().getChildRendezvous().getParentLocation());
                agent.setPath(path);
                agent.setCurrentGoal(agent.getRendezvousAgentData().getChildRendezvous().getParentLocation());
                
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
                    path = agent.calculatePath(agent.getLocation(), agent.getRendezvousAgentData().getChildRendezvous().getParentLocation(), true);
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
                        agent.setCurrentGoal(agent.getRendezvousAgentData().getChildRendezvous().getParentLocation());
                        return((Point)agent.getPath().getPoints().remove(0));
                    }
                    //</editor-fold>
                }
            }
            //</editor-fold>
        }
        //</editor-fold>
    }
    
    public static Point takeStep_GoToChild(RealAgent agent) {      
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //<editor-fold defaultstate="collapsed" desc="Check if we are in range of child. If yes, GetInfoFromChild">
        if((agent.getChildTeammate().isInRange()) /*&& !agent.getParentTeammate().isInRange()*/) {
            agent.setState(RealAgent.ExploreState.GetInfoFromChild);
            agent.setStateTimer(0);
            
            return takeStep_GetInfoFromChild(agent);
        }
        //</editor-fold>
        
        //<editor-fold defaultstate="collapsed" desc="Assume that a path back to child has been calculated in previous state, recalculate every PATH_RECALC_CHILD_INTERVAL steps">
        Path existingPath = agent.getPath();
        if(((agent.getStateTimer() % Constants.PATH_RECALC_CHILD_INTERVAL) == (Constants.PATH_RECALC_CHILD_INTERVAL - 1))) {
            if (existingPath != null)
                agent.addDirtyCells(existingPath.getAllPathPixels());
            
            agent.getRendezvousStrategy().processGoToChildReplan();
            
            Path path = agent.calculatePath(agent.getLocation(), rvd.getChildRendezvous().getParentLocation());
            //<editor-fold defaultstate="collapsed" desc="Could not find full path! Trying pure A*">
            if (!path.found)
            {
                System.out.println(agent.toString() + "!!!ERROR!  Could not find full path! Trying pure A*");
                path = agent.calculatePath(agent.getLocation(), agent.getRendezvousAgentData().getChildRendezvous().getParentLocation(), true);
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
                agent.setCurrentGoal(rvd.getChildRendezvous().getParentLocation());
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
        
        if(agent.getLocation().distance(rvd.getChildRendezvous().getParentLocation()) > 2*Constants.STEP_SIZE)
        {
            System.out.println(agent.toString() + 
                    "!!!ERROR! We should have reached child RV, but we are too far from it! Taking random step");
            return RandomWalk.takeStep(agent);
        }
        
        // If we reach this point, we're at the rendezvous point and waiting.
        agent.setState(RealAgent.ExploreState.WaitForChild);
        agent.setStateTimer(0);
        
        return new Point(agent.getX(), agent.getY());
    }
    
    public static Point takeStep_WaitForChild(RealAgent agent) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        if(agent.getChildTeammate().isInRange()) {
            agent.setState(RealAgent.ExploreState.GetInfoFromChild);
            agent.setStateTimer(0);
            return takeStep_GetInfoFromChild(agent);
        }
        
        boolean canStillWait = (timeElapsed <= 
                (rvd.getChildRendezvous().getTimeMeeting() + rvd.getChildRendezvous().getTimeWait()));
        
        if (canStillWait) {
            return agent.getRendezvousStrategy().processWaitForChild();
        } else
        {
            //Go to backup RV if available. Otherwise do what the strategy requires us to do, e.g. become an explorer.
            if (rvd.getChildBackupRendezvous() != null) {
                rvd.setChildRendezvous(rvd.getChildBackupRendezvous());
                rvd.setChildBackupRendezvous(null);
                agent.setState(RealAgent.ExploreState.GoToChild);
                agent.setStateTimer(0);
                return takeStep_GoToChild(agent);
            } else {
                return agent.getRendezvousStrategy().processWaitForChildTimeoutNoBackup();
            }
        }
        
        //boolean throughWall = isChildRVThroughWall(agent);
        //<editor-fold defaultstate="collapsed" desc="Uncomment this if relays should explore instead of waiting forever">
        /*if(allowReplanning && (!canStillWait) && !throughWall) {
            agent.setState(RealAgent.ExploreState.Explore);
            agent.setStateTimer(0);
            return takeStep_Explore(agent, simConfig);
        }*/
        //</editor-fold>
        
        //<editor-fold defaultstate="collapsed" desc="If RV is through a wall, and we can still wait, move closer to wall, else go2backupRV">
        /*if (throughWall) {
            if (canStillWait) {
                //<editor-fold defaultstate="collapsed" desc="Try to move as close as possible to the wall">
                Point point1 = agent.getLocation();
                Point point2 = agent.getRendezvousAgentData().getChildRendezvous().getChildLocation();
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
                
            }
        }*/
        //</editor-fold>
            
        //return new Point(agent.getX(), agent.getY());
    }
    
    public static Point takeStep_GetInfoFromChild(RealAgent agent) {
        //we've exchanged info, now return to parent (but wait for one timestep to learn new RV point)
        
        if(agent.getStateTimer() == 0) {
            agent.addDirtyCells(agent.getPath().getAllPathPixels());
            Path path = agent.calculatePath(agent.getLocation(), agent.getRendezvousAgentData().getParentRendezvous().getChildLocation());

            agent.setPath(path);
            agent.setStateTimer(1);
            agent.setCurrentGoal(agent.getRendezvousAgentData().getParentRendezvous().getChildLocation());
            return agent.getLocation();
        }
        else {
            agent.setState(RealAgent.ExploreState.ReturnToParent);
            return takeStep_ReturnToParent(agent);
        }
    }
    
// </editor-fold>  
    
    private static boolean isDueToReturnToRV(RealAgent agent, AtomicReference<Path> outPathToParentRendezvous) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        System.out.println(agent.toString() + "Checking if it's time to rendezvous ... ");
        Path pathToParentRendezvous = agent.calculatePath(agent.getLocation(), rvd.getParentRendezvous().getChildLocation());
        System.out.println(Constants.INDENT + "rendezvous is " + (int) pathToParentRendezvous.getLength() + 
                " away, time left is " + (rvd.getParentRendezvous().getTimeMeeting() - timeElapsed) + 
                " (meeting scheduled at " + rvd.getParentRendezvous().getTimeMeeting() + ")");

        // If we are due to meet parent again, return to last agreed rendezvous point
        if (pathToParentRendezvous.found) {
            outPathToParentRendezvous.set(pathToParentRendezvous);
            if (((pathToParentRendezvous.getLength() / Constants.DEFAULT_SPEED) + timeElapsed) >= 
                    agent.getRendezvousAgentData().getParentRendezvous().getTimeMeeting()) {
                return true;
            }
        } else {
            System.out.println(agent.toString() + 
                    "!!!Cannot plan path to parent RV!!! - will continue to explore and try again in " + 
                    Constants.CHECK_INTERVAL_TIME_TO_RV);
        }
        
        return false;
    }
    
// <editor-fold defaultstate="collapsed" desc="Calculate rendezvous">    
    private static LinkedList<NearRVPoint> generateSobolPoints(OccupancyGrid grid) {
        SobolSequenceGenerator sobolGen = new SobolSequenceGenerator(2);
        
        //int numPointsToGenerate = grid.getNumFreeCells() * 150 / 432000;
        int numPointsToGenerate = grid.getNumFreeCells() / 2500; //roughly every 50 sq. cells
        System.out.println("Generating " + numPointsToGenerate + " Sobol points");
        
        LinkedList<NearRVPoint> generatedPoints = new LinkedList<NearRVPoint>();
        
        for (int i = 0; i < numPointsToGenerate; i++) {
            int x = 0;
            int y = 0;
            double[] vector;
            do {
                vector = sobolGen.nextVector();
                x = (int)(vector[0] * grid.width);
                y = (int)(vector[1] * grid.height);
            } while(!grid.freeSpaceAt(x, y));
            
            NearRVPoint pd = new NearRVPoint(x, y);
            
            /*simConfig.getEnv().setPathStart(pd.point);
            simConfig.getEnv().setPathGoal(expLocation);
            simConfig.getEnv().getTopologicalPath(false);
            pd.distance1 = simConfig.getEnv().getPath().getLength();
            
            simConfig.getEnv().setPathStart(pd.point);
            simConfig.getEnv().setPathGoal(relLocation);
            simConfig.getEnv().getTopologicalPath(false);
            pd.distance2 = simConfig.getEnv().getPath().getLength();*/
            
            generatedPoints.add(pd);
            //freeSpace.remove(index);
        }
        
        return generatedPoints;
    }
    
    //This method is called by Explorer (child) to arrange RV with Relay (parent)
    /*private static void calculateRendezvousRandomSamplingEx(RealAgent agent) {
        // Only calculate rv every several time steps at most
        if(agent.getTimeSinceLastRVCalc() < 15)
            return;
        else
            agent.setTimeSinceLastRVCalc(0);

        long realtimeStart = System.currentTimeMillis();
        long intermediateTime1, intermediateTime2, intermediateTime3, intermediateTime4;
        System.out.println(agent.toString() + "Calculating next rendezvous ... ");

        TeammateAgent relay = agent.getParentTeammate();
        
        intermediateTime1 = System.currentTimeMillis();
        System.out.print(Constants.INDENT + "Generating random points ... ");

        LinkedList<NearRVPoint> generatedPoints = generateSobolPoints(agent.getOccupancyGrid());
        //add base station to it. Could also add any special points here as well
        NearRVPoint base = new NearRVPoint(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getX(), 
                agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getY());
        generatedPoints.add(base);

        intermediateTime2 = System.currentTimeMillis();
        System.out.println("complete, took " + (intermediateTime2 - intermediateTime1) + " ms.");

        System.out.print(Constants.INDENT + "Finding commlinks ... ");
        LinkedList<CommLink> commLinks = new LinkedList<CommLink>();
        LinkedList<CommLink> connectionsToBase = new LinkedList<CommLink>();
        
        for (NearRVPoint p1: generatedPoints) {
            for (NearRVPoint p2: generatedPoints) {
                if (p1.distance(p2) <= PropModel1.getMaxRange()) {
                    //TODO: range should be min of ours and our teammate's
                    if (PropModel1.isConnected(agent.getOccupancyGrid(), agent.getCommRange(), (Point)p1, (Point)p2)) {
                        //check if connection is line of sight?
                        int numWalls = agent.getOccupancyGrid().numObstaclesOnLine(p1.x, p1.y, p2.x, p2.y);
                        
                        CommLink link = new CommLink(p1, p2);
                        link.numObstacles = numWalls;
                        commLinks.add(link);
                        p1.commLinks.add(link);
                        if (p1 == base) {
                            System.out.println(Constants.INDENT + "Base is " + p1 + ", adding connected point " + p2);                            
                            connectionsToBase.add(link);
                        }
                        //commLinks.add(new CommLink(p2, p1, null, null));
                    }
                }
            }
        }

        intermediateTime3 = System.currentTimeMillis();
        System.out.println("complete, took " + (intermediateTime3 - intermediateTime2) + " ms.");
        System.out.println(Constants.INDENT + "Choosing specific RV point ... ");
        
        PriorityQueue<NearRVPoint> pointsNearFrontier = new PriorityQueue<NearRVPoint>();
        Point frontierCentre;
        if(agent.getLastFrontier() != null)
            frontierCentre = agent.getLastFrontier().getCentre();//getClosestPoint(agent.getLocation(), agent.getOccupancyGrid());
        else
        {
            System.out.println(agent + " !!!! getLastFrontier returned null, setting frontierCentre to " + agent.getLocation());
            frontierCentre = agent.getLocation();
        }
            
        System.out.println(agent + " frontierCentre is " + frontierCentre);
        // create priority queue of all potential rvpoints within given straight line distance
        for(NearRVPoint p: generatedPoints) {
            double dist = p.distance(frontierCentre);
            if(dist > 600)
                continue;
            p.setDistanceToFrontier(dist);
            pointsNearFrontier.add(p);
        }
        
        intermediateTime3 = System.currentTimeMillis();
        System.out.println(agent + " connectionsToBase count is " + connectionsToBase.size());
        
        int pathsCalculated = 0;
        //Now for top K points, let's calculate p' distances to base, and find the nearest point connected to base
        PriorityQueue<NearRVPoint> pointsNearFrontierReal = new PriorityQueue<NearRVPoint>();
        for (int k = 0; (k < 50) && !pointsNearFrontier.isEmpty(); k++) {
            NearRVPoint p = pointsNearFrontier.poll();
            double minDistToBase = Double.MAX_VALUE;
            
            for(CommLink link: p.commLinks) {
                NearRVPoint connectedPoint = link.getRemotePoint();
                // only calculate nearest base point for connectedPoint if we haven't already.
                if (connectedPoint.distanceToParent == Double.MAX_VALUE) {
                    PriorityQueue<NearRVPoint> lineOfSightBasePoints = new PriorityQueue<NearRVPoint>();
                    PriorityQueue<NearRVPoint> nonLOSBasePoints = new PriorityQueue<NearRVPoint>();
                    for(CommLink baseLink: connectionsToBase) {
                        NearRVPoint basePoint = new NearRVPoint(baseLink.getRemotePoint().x, baseLink.getRemotePoint().y);
                        double approxPathLen = basePoint.distance(connectedPoint);
                        basePoint.setDistanceToFrontier(approxPathLen);
                        if (baseLink.numObstacles == 0)
                            lineOfSightBasePoints.add(basePoint);
                        else
                            nonLOSBasePoints.add(basePoint);
                    }
                    
                    LinkedList<NearRVPoint> pointsConnectedToBase = new LinkedList<NearRVPoint>();
                    
                    for (int j = 0; (j < 5) && !lineOfSightBasePoints.isEmpty(); j++) {
                        pointsConnectedToBase.add(lineOfSightBasePoints.poll());
                    }
                    
                    for (int j = 0; (j < 5) && !nonLOSBasePoints.isEmpty(); j++) {
                        pointsConnectedToBase.add(nonLOSBasePoints.poll());
                    }
                    
                    for(NearRVPoint basePoint: pointsConnectedToBase) {
                        pathsCalculated++;
                        Path pathToBase = agent.calculatePath(connectedPoint, basePoint);
                        double pathLen = Double.MAX_VALUE;
                        if (pathToBase.found)
                            pathLen = pathToBase.getLength();
                        if (pathLen < connectedPoint.distanceToParent) {
                            connectedPoint.distanceToParent = pathLen;
                            connectedPoint.parentPoint = basePoint;
                        }
                    }
                }
                if (connectedPoint.distanceToParent < minDistToBase) {
                    minDistToBase = connectedPoint.distanceToParent;
                    p.commLinkClosestToBase = link;
                }
            }
            //At this point, for p, we know:
            //  1. Connected point p' that is nearest to comm range of Base
            //  2. Distance from p' to comm range of Base
            //  3. Nearest point from p' that is within comm range of Base
            //So we know how long each point p will have to wait for relay, and so can estimate
            //where explorer will be at the time, to calculate regret accurately.
            
            //For now, just calculate accurate distance to next frontier:
            Path pathToFrontier = agent.calculatePath(p, frontierCentre);
            double distToFrontier = Double.MAX_VALUE;
            if (pathToFrontier.found)
                distToFrontier = pathToFrontier.getLength();
            pathsCalculated++;
            p.setDistanceToFrontier(distToFrontier);
            p.utility = NearRVPoint.getFullRVUtility(p.distanceToFrontier, 
                    p.commLinkClosestToBase.getRemotePoint().distanceToParent, p.commLinkClosestToBase.numObstacles);
            System.out.println(agent + " utility is " + p.utility + " for point " + p + " linked to " + 
                    p.commLinkClosestToBase.getRemotePoint() + "; distToFrontier: " + p.distanceToFrontier + 
                    ", distToParent: " + p.commLinkClosestToBase.getRemotePoint().distanceToParent);
            pointsNearFrontierReal.add(p);
        }
        System.out.println(agent + "complete, took " + (System.currentTimeMillis() - intermediateTime3) + 
                " ms., paths calculated: " + pathsCalculated);
        
        
        //Now just need to retrieve the best point

        NearRVPoint childPoint = pointsNearFrontierReal.peek();
        NearRVPoint parentPoint = childPoint.commLinkClosestToBase.getRemotePoint();
        
        Rendezvous meetingLocation = new Rendezvous(childPoint);
        meetingLocation.setParentLocation(parentPoint);
        
        Rendezvous parentsMeetingLocation = new Rendezvous(parentPoint.parentPoint);
        Point baseLocation = agent.getTeammate(agent.getParentTeammate().getParent()).getLocation();
        System.out.println("    base location: " + baseLocation);
        parentsMeetingLocation.setParentLocation(agent.getTeammate(agent.getParentTeammate().getParent()).getLocation());
        
        meetingLocation.parentsRVLocation = parentsMeetingLocation;
        agent.setParentRendezvous(meetingLocation);
        
        //Calculate RV timings
        NearRVPoint basePoint = parentPoint.parentPoint;
        double timeRelayToBase = agent.calculatePath(relay.getLocation(), basePoint).getLength();
        double timeBaseToRV = agent.calculatePath(basePoint, parentPoint).getLength();
        double timeExpToFrontier = agent.calculatePath(agent.getLocation(), frontierCentre).getLength();
        double timeFrontierToRV = agent.calculatePath(frontierCentre, childPoint).getLength();
        
        double timeToMeetingR = timeRelayToBase + timeBaseToRV;
        timeToMeetingR = timeElapsed + timeToMeetingR / Constants.DEFAULT_SPEED;
        double timeToMeetingE = timeExpToFrontier + Constants.FRONTIER_MIN_EXPLORE_TIME + timeFrontierToRV;
        timeToMeetingE = timeElapsed + timeToMeetingE / Constants.DEFAULT_SPEED;
        int timeToMeeting = (int)Math.ceil(Math.max(timeToMeetingR, timeToMeetingE));
        agent.getParentRendezvous().setTimeMeeting(timeToMeeting);
        agent.getParentRendezvous().setMinTimeMeeting(timeToMeeting);
        agent.getParentRendezvous().setTimeWait(Constants.WAIT_AT_RV_BEFORE_REPLAN);
        
        System.out.println(agent + " timeToMeetingR: " + timeToMeetingR + ", timeToMeetingE: " + timeToMeetingE + ", timeToMeeting: " + timeToMeeting);
        
        //Output the process as an image
        try
        {
            OccupancyGrid agentGrid = agent.getOccupancyGrid();
            ExplorationImage img = new ExplorationImage(new Environment(agentGrid.height, agentGrid.width));
            ShowSettingsAgent agentSettings = new ShowSettingsAgent();
            agentSettings.showFreeSpace = true;
            img.fullUpdateRVPoints(agentGrid, pointsNearFrontierReal, generatedPoints, frontierCentre, agentSettings);
            img.saveScreenshot("c:\\Temp\\rvgen\\");
            System.out.println("Outputting path debug screens");
        } catch (Exception e)
        {
            System.out.println("Couldn't save path error screenshot, reason: " + e.getMessage());
        }

        intermediateTime4 = System.currentTimeMillis();

        System.out.print(Constants.INDENT + "Choosing complete, chose " + 
                agent.getParentRendezvous().getChildLocation().x + "," + 
                agent.getParentRendezvous().getChildLocation().y + ". ");
        System.out.println("Took " + (intermediateTime4 - intermediateTime3) + "ms.");
        System.out.println(Constants.INDENT + "Complete RV calculation process took " + 
                (System.currentTimeMillis()-realtimeStart) + "ms.");
    }*/
    
    /*//Calculate time to next RV (we are the relay)
    private static void calculateOwnTimeToRVEx(RealAgent agent)
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
                if (occGrid.freeSpaceAt(p.x, p.y)//&& !env.directLinePossible(firstRV.x, firstRV.y, p.x, p.y))
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
    }*/
    
    /*private static void calculateOwnTimeToBackupRVEx(RealAgent agent)
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
            System.out.println(agent + "Calculating child backup RV");
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
    }*/
    
    /*public static void calculateRVThroughWallsEx(RealAgent agent) {
        // Only calculate rv every several time steps at most
        if(agent.getTimeSinceLastRVCalc() < 15)
            return;
        else
            agent.setTimeSinceLastRVCalc(0);
        
        long realtimeStart = System.currentTimeMillis();
        System.out.println(agent.toString() + "Calculating RV Through Walls");
        
        agent.forceUpdateTopologicalMap(true);
        
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
        
        Rendezvous newLocation = 
            agent.getTopologicalMap().findNearestBorderKeyPoint(agent.getParentRendezvous().getChildLocation(), agent);
        
        System.out.println(" 4 Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        realtimeStart = System.currentTimeMillis();
        
        
        agent.setParentRendezvous(newLocation);
    }
    
    public static boolean isParentRVThroughWallEx(RealAgent agent)
    {
        return !agent.getOccupancyGrid().directLinePossible(
                agent.getParentRendezvous().getChildLocation().x, agent.getParentRendezvous().getChildLocation().y, 
                agent.getParentRendezvous().getParentLocation().x, agent.getParentRendezvous().getParentLocation().y);
    }
    
    public static boolean isChildRVThroughWallEx(RealAgent agent)
    {
        return !agent.getOccupancyGrid().directLinePossible(
                agent.getChildRendezvous().getChildLocation().x, agent.getChildRendezvous().getChildLocation().y, 
                agent.getChildRendezvous().getParentLocation().x, agent.getChildRendezvous().getParentLocation().y);
    }*/
    
// </editor-fold>     

}
