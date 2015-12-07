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


import exploration.rendezvous.Rendezvous;
import agents.BasicAgent;
import agents.RealAgent;
import config.Constants;
import config.RobotConfig;
import config.SimulatorConfig;
import environment.*;
import exploration.rendezvous.IRendezvousStrategy;
import exploration.rendezvous.RendezvousAgentData;
import java.util.*;
import java.awt.*;
import java.util.concurrent.atomic.AtomicReference;
import path.Path;

/**
 *
 * @author julh, vspirin
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
                return RandomWalk.takeStep(agent);
            else 
                return(agent.getLocation()); 
        else if (agent.getStateTimer() < 3)
            return RandomWalk.takeStep(agent);
        // </editor-fold>
        // <editor-fold defaultstate="collapsed" desc="Otherwise? Explorers go into Explore state, others go into GoToChild state. Explorers replan using FrontierExploration, others do nothing.">
        else {
            if(agent.isExplorer()) {
                agent.setState(RealAgent.ExploreState.Explore);                
                agent.getStats().setTimeSinceLastPlan(0);
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
        if (agent.getParentTeammate().isInRange() && (agent.getRole() != RobotConfig.roletype.Relay)) {
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
        //if we are a relay, check if we are due to return to child and return if it's time
        if (agent.getRole() == RobotConfig.roletype.Relay) {
            if((agent.getChildTeammate().isInRange()) /*&& !agent.getParentTeammate().isInRange()*/) {
                agent.setState(RealAgent.ExploreState.GetInfoFromChild);
                agent.setStateTimer(0);

                return takeStep_GetInfoFromChild(agent);
            }
            if ((agent.getStateTimer() % Constants.CHECK_INTERVAL_TIME_TO_RV) == (Constants.CHECK_INTERVAL_TIME_TO_RV - 1)) {
                Path path = agent.calculatePath(agent.getLocation(), rvd.getChildRendezvous().getParentLocation());
                if ((path.getLength() / Constants.DEFAULT_SPEED) + agent.getTimeElapsed() >= rvd.getChildRendezvous().getTimeMeeting()) {
                    agent.setState(BasicAgent.ExploreState.GoToChild);
                    return takeStep_GoToChild(agent);
                }
            }
        }
        
        // <editor-fold defaultstate="collapsed" desc="Every CHECK_INTERVAL_TIME_TO_RV steps, check if we're due to meet our parent again (unless parent is basestation, in which case explore continuously)">
        if (agent.getParent() != Constants.BASE_STATION_TEAMMATE_ID
                && (agent.getStateTimer() % Constants.CHECK_INTERVAL_TIME_TO_RV) == (Constants.CHECK_INTERVAL_TIME_TO_RV - 1)) 
        {           
            
            Path pathToParentRendezvous; //output parameter
            AtomicReference<Path> outPathRef = new AtomicReference<Path>();
            if (isDueToReturnToRV(agent, outPathRef)) {
                //run below method here as it may be costly and we only want to run it when we are about to return to RV anyway
                //so we want to check if perhaps there is a better RV we could go to that is closer, so we can explore some more
                agent.getRendezvousStrategy().processExplorerCheckDueReturnToRV();
                if (isDueToReturnToRV(agent, outPathRef)) {
                    pathToParentRendezvous = outPathRef.get();
                    agent.setState(RealAgent.ExploreState.ReturnToParent);

                    agent.getRendezvousStrategy().processExplorerStartsHeadingToRV();

                    if (agent.getPath() != null) {
                        agent.addDirtyCells(agent.getPath().getAllPathPixels());
                    }

                    if (agent.getState() == RealAgent.ExploreState.ReturnToParent) {
                        if ((pathToParentRendezvous == null) || (!pathToParentRendezvous.found) ||
                                (pathToParentRendezvous.getPoints().isEmpty())) {
                            //take random step and try again
                            agent.setState(BasicAgent.ExploreState.Explore);
                            agent.setStateTimer(Constants.CHECK_INTERVAL_TIME_TO_RV - 2);
                            return RandomWalk.takeStep(agent);
                        } else {                
                             agent.setPath(pathToParentRendezvous);
                             agent.setStateTimer(0);
                             agent.setCurrentGoal(rvd.getParentRendezvous().getChildLocation());
                             return ((Point) agent.getPath().getPoints().remove(0));
                        }
                    }
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
        if (timeElapsed > 100) { //prevent setting mission complete at the very start of the exploration
            if ((agent.getFrontiers().isEmpty() || (agent.getStats().getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))) {
                Path pathToParentRendezvous = agent.calculatePath(agent.getLocation(), agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation());//rvd.getParentRendezvous().getChildLocation());
                agent.setPath(pathToParentRendezvous);
                agent.setState(RealAgent.ExploreState.ReturnToParent);
                System.out.println(agent + "RB Explore setting mission complete, returning to base");
                agent.setMissionComplete(true);
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
            //System.out.println(agent + "rvStrategy agent is " + agent.getRendezvousStrategy().getAgent());
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
            //<editor-fold defaultstate="collapsed" desc="Case 1: Explorer">
            if(agent.isExplorer()) {
                // First, plan next frontier, as we need this for rendezvous point calculation
                FrontierExploration.replan(agent, SimulatorConfig.frontiertype.ReturnWhenComplete, 0);
            }
            
            agent.getRendezvousStrategy().processJustGotIntoParentRange();
 
            agent.setStateTimer(1);
            return agent.getLocation();
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
                if (agent.getState() == RealAgent.ExploreState.Explore) {
                    agent.setStateTimer(0);
                    return takeStep_Explore(agent);
                }
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
