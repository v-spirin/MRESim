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

import agents.*;
import agents.BasicAgent.ExploreState;
import config.Constants;
import config.RobotConfig;
import config.SimulatorConfig;
import config.SimulatorConfig.frontiertype;
import environment.*;
import java.util.*;
import java.awt.*;
import path.Path;

/**
 *
 * @author Victor
 */

public class UtilityExploration {
    
    private static final int TIME_BETWEEN_PLANS = 1;
    private static final int TIME_BETWEEN_RECOMPUTE_PATHS = 10;
    public static int timeElapsed;
    public static int oldTimeElapsed;

// <editor-fold defaultstate="collapsed" desc="Take Step">

    // Returns new X, Y of ExploreAgent
    public static Point takeStep(RealAgent agent, int te, SimulatorConfig simConfig) {
        long realtimeStart = System.currentTimeMillis();
        //<editor-fold defaultstate="collapsed" desc="Assign local variables">
        timeElapsed = te;
        
        Point nextStep = null;
        //</editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="Handle environment error - agent stuck next to a wall">
        // if env reports error, agent may be stuck in front of a wall and the
        // simulator isn't allowing him to go through.  Taking a random step might
        // help.
        // Update:  this state is never really reached but leave in just in case
        if(agent.getEnvError()) {
            System.out.println(agent.toString() + "!!!UtilityExploration: Env reports error, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
            agent.setEnvError(false);
            return nextStep;
        }
        // </editor-fold>
        
        // <editor-fold defaultstate="collapsed" desc="Run correct takeStep function depending on agent state, set nextStep to output">
        // Explore is normal exploration, ReturnToParent is relaying information to base
        switch(agent.getState()) {
            case Initial :          nextStep = takeStep_Initial(agent, simConfig);
                                    break;
            case Explore :          nextStep = takeStep_Explore(agent, simConfig);
                                    break;
            case ReturnToParent :   nextStep = takeStep_ReturnToParent(agent, simConfig);
                                    break;
            default :               break;
        }
        
        // this shouldn't happen, looks like one of takeSteps returned an error
        if(nextStep == null)
        {
            System.out.println(agent.toString() + "!!!UtilityExploration: nextStep is null, taking random step.");
            nextStep = RandomWalk.takeStep(agent);
        }
        // </editor-fold>
     
        // <editor-fold defaultstate="collapsed" desc="Increment state timers">
        agent.setStateTimer(agent.getStateTimer() + 1);
        //TODO: Should this be in UtilityExploration?
        agent.getRendezvousAgentData().setTimeSinceLastRoleSwitch(agent.getRendezvousAgentData().getTimeSinceLastRoleSwitch() + 1);
        //</editor-fold>
        System.out.println(agent.toString() + " takeStep " + agent.getState() + ", took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        return nextStep;
    }
    
    private static Point takeStep_Initial(RealAgent agent, SimulatorConfig simConfig) {
        // Small number of random steps to get initial range data
        // <editor-fold defaultstate="collapsed" desc="First 3 steps? Take random step">
        if (agent.getStateTimer() < 3)
            //return RandomWalk.takeStep(agent);
            return agent.getLocation();
        // </editor-fold>
        // <editor-fold defaultstate="collapsed" desc="Otherwise? Explorers go into Explore state, others go into GoToChild state. Explorers replan using FrontierExploration, others do nothing.">
        else {
            agent.setState(RealAgent.ExploreState.Explore);                
            agent.getStats().setTimeSinceLastPlan(0);
            return takeStep_Explore(agent, simConfig);
        }
        // </editor-fold>
    }
    
    private static Point takeStep_Explore(RealAgent agent, SimulatorConfig simConfig) {  
        Point nextStep;
        // <editor-fold defaultstate="collapsed" desc="Every CHECK_INTERVAL_TIME_TO_RV steps, check if we're due to meet our parent again (unless parent is basestation, in which case explore continuously)">
        int totalNewInfo = agent.getStats().getNewInfo();
        //double infoRatio = (double)agent.getLastContactAreaKnown() / 
        //        (double)(agent.getLastContactAreaKnown() + totalNewInfo);
        double infoRatio = (double)agent.getStats().getCurrentBaseKnowledgeBelief() / 
                (double)(agent.getStats().getCurrentBaseKnowledgeBelief() + totalNewInfo);
        
        if ((!agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInRange()) && (infoRatio < simConfig.TARGET_INFO_RATIO))
        {
            //System.out.println(agent.toString() + " Decided to return.");     
            agent.setState(ExploreState.ReturnToParent);
            //agent.setRole(RobotConfig.roletype.Relay);
            agent.setPathToBaseStation();

            if (agent.getPath() == null || agent.getPath().getPoints() == null || agent.getPath().getPoints().size() <= 1)
            {
                //System.out.println(agent.toString() + "Can't find my way home, taking random step.");
                nextStep = RandomWalk.takeStep(agent);
                agent.getStats().setTimeSinceLastPlan(0);
                agent.setCurrentGoal(nextStep);
                return nextStep;
            }

            nextStep = agent.getNextPathPoint();
            if (nextStep.equals(agent.getLocation()))
                nextStep = agent.getNextPathPoint();
            if (nextStep == null) nextStep = agent.getLocation();

            agent.getStats().setTimeSinceLastPlan(0);
            agent.setCurrentGoal(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation());
            return nextStep;
        }
        // </editor-fold>   

        //if we reach this point we continue exploring
        //make sure we replan, if we just entered Explore state
        if (agent.getStateTimer() == 0)
            agent.getStats().setTimeSinceLastPlan(Constants.REPLAN_INTERVAL + 1);
        nextStep = FrontierExploration.takeStep(agent, timeElapsed, SimulatorConfig.frontiertype.ReturnWhenComplete);
        
        //<editor-fold defaultstate="collapsed" desc="If there are no frontiers to explore, we must be finished.  Return to ComStation.">
        if ((agent.getFrontiers().isEmpty() || (agent.getStats().getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))) {
            agent.setMissionComplete(true);
            Point baseLocation = agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation();
            agent.addDirtyCells(agent.getPath().getAllPathPixels());
            Path path = agent.calculatePath(agent.getLocation(), baseLocation);
            agent.setPath(path);
            agent.setState(RealAgent.ExploreState.ReturnToParent);
            agent.setStateTimer(0);
            
            if(agent.getPath().getPoints() != null) {
                agent.setCurrentGoal(baseLocation);
                return((Point)agent.getPath().getPoints().remove(0));
            }
            else {
                System.out.println(agent.toString() + "!!!Nothing left to explore, but cannot plan path to parent!!!");
                nextStep = RandomWalk.takeStep(agent);
                agent.setCurrentGoal(nextStep);
                return(nextStep);
            }
        }
        //</editor-fold>
        return nextStep;
    }
    
    private static Point takeStep_ReturnToParent(RealAgent agent, SimulatorConfig simConfig) { 
        //<editor-fold defaultstate="collapsed" desc="If base is in range, go back to exploring">
        if(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInRange()) {
            agent.setState(RealAgent.ExploreState.Explore);
            agent.setStateTimer(0);
            return takeStep_Explore(agent, simConfig);
        }
        //</editor-fold>
        
        //<editor-fold defaultstate="collapsed" desc="If mission complete and we have path, keep going">
        if(agent.isMissionComplete() && !agent.getPath().getPoints().isEmpty())
            return((Point)agent.getPath().getPoints().remove(0));
        //</editor-fold>
        
        //<editor-fold defaultstate="collapsed" desc="If newInfo goes under ratio, go exploring (can happen if we meet a relay)">
        int totalNewInfo = agent.getStats().getNewInfo();
        double infoRatio = (double)agent.getStats().getLastContactAreaKnown() / 
                (double)(agent.getStats().getLastContactAreaKnown() + totalNewInfo);
        if (infoRatio > simConfig.TARGET_INFO_RATIO) {
            agent.setState(RealAgent.ExploreState.Explore);
            agent.setStateTimer(0);
            return takeStep_Explore(agent, simConfig);
        }
        //</editor-fold>

        Point baseLocation = agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation();
        //<editor-fold defaultstate="collapsed" desc="Recalculate path every PATH_RECALC_PARENT_INTERVAL steps, if fail try A*, if that fails try using existing path, if that fails take random step">
        Path existingPath = agent.getPath();
        if((agent.getStateTimer() % Constants.PATH_RECALC_PARENT_INTERVAL) == (Constants.PATH_RECALC_PARENT_INTERVAL - 1)) {
            //<editor-fold defaultstate="collapsed" desc="If path already exists, update dirty cells with that path">
            if (existingPath != null)
                agent.addDirtyCells(existingPath.getAllPathPixels());
            //</editor-fold>            
            Path path = agent.calculatePath(agent.getLocation(), baseLocation);
            //<editor-fold defaultstate="collapsed" desc="If path not found, try A*">
            if (!path.found)
            {
                System.out.println(agent.toString() + "ERROR!  Could not find full path! Trying pure A*");
                path = agent.calculatePath(agent.getLocation(), baseLocation, true);
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
                agent.setCurrentGoal(baseLocation);
                // Must remove first point in path as this is robot's location.
                agent.getPath().getPoints().remove(0);
            }
        }
        //</editor-fold>
        
        if(agent.getPath().found && !agent.getPath().getPoints().isEmpty())
            return((Point)agent.getPath().getPoints().remove(0));
        
        // If we reach this point, we are not in range of the base and the
        // path is empty, so we must have the base.
        // Just check to make sure we are though and if not take random step.
        if(agent.getLocation().distance(baseLocation) > 2*Constants.STEP_SIZE)
        {
            System.out.println(agent.toString() + "!!!ERROR! We should have reached parent RV, but we are too far from it! Taking random step");
            return RandomWalk.takeStep(agent);
        }
        
        // If we reach this point, we're at the base station; go back to exploring.
        agent.setState(RealAgent.ExploreState.Explore);
        agent.setStateTimer(0);
        return takeStep_Explore(agent, simConfig);
    }
    // </editor-fold>     

}
