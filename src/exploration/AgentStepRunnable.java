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
import config.SimulatorConfig;
import environment.Environment;
import java.awt.Point;
import java.util.LinkedList;

/**
 *
 * @author Victor
 */
public class AgentStepRunnable implements Runnable{
    private RealAgent agent;
    private SimulatorConfig simConfig;
    private int timeElapsed;
    private Environment env;
    private SimulationFramework simFramework;
    
    AgentStepRunnable(RealAgent agent, SimulatorConfig simConfig, int timeElapsed, 
            Environment env, SimulationFramework simFramework) 
    {
        this.agent = agent;
        this.simConfig = simConfig;
        this.timeElapsed = timeElapsed;
        this.env = env;
        this.simFramework = simFramework;
    }

    @Override
    public void run() 
    {
        Point nextStep = null;
        double[] sensorData = null;
        double distance_left = agent.getSpeed();
        //profiling
        long realtimeStartAgentCycle = System.currentTimeMillis();
        
        //<editor-fold defaultstate="collapsed" desc="Continue along the path, until we have exhausted agent 'speed' per cycle or run out of path">
        while (distance_left > 0)
        {
            //<editor-fold defaultstate="collapsed" desc="Get next step">
            nextStep = agent.takeStep(timeElapsed);
            if(nextStep == null) {
                nextStep = agent.getLocation();
                System.out.println(agent + " !!! setting envError because nextStep is null, distance_left is " + distance_left);
                agent.setEnvError(true);
                distance_left = 0;
            }
            agent.flush();
            //</editor-fold>
            System.out.println(agent.toString() + "Get next step took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
                        
            //<editor-fold defaultstate="collapsed" desc="Check to make sure step is legal">
            if(env.directLinePossible(agent.getX(), agent.getY(), nextStep.x, nextStep.y)) {
                //check here we don't 'teleport'
                double dist = agent.getLocation().distance(nextStep);
                //System.out.println(agent.toString() + "1 took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
                //<editor-fold defaultstate="collapsed" desc="If we don't have enough 'speed' left to reach nextPoint, go as far as we can and keep nextPoint in the path">
                if (dist > distance_left) {
                    //System.out.println(agent.toString() + " exceeded speed. Distance left: " + distance_left + ", dist to next path point: " + dist);
                    //Add nextStep back to path, as we will not reach it yet
                    if ((agent.getPath() != null) && (agent.getPath().getPoints() != null))
                        agent.getPath().getPoints().add(0, nextStep);
                    double ratio = distance_left / dist;
                    nextStep.x = agent.getX() + (int)Math.round((nextStep.x - agent.getX()) * ratio);
                    nextStep.y = agent.getY() + (int)Math.round((nextStep.y - agent.getY()) * ratio);
                    if (!env.directLinePossible(agent.getX(), agent.getY(), nextStep.x, nextStep.y))
                    {
                        nextStep.x = agent.getX();
                        nextStep.y = agent.getY();
                        System.out.println(agent.toString() + " directLinePossible returned wrong result!");
                    }
                    //System.out.println(agent.toString() + " speed corrected. Now is: " + agent.getLocation().distance(nextStep));
                    distance_left = 0;
                //</editor-fold>
                } else
                {
                    distance_left = distance_left - dist;
                }
                // comment below out to process sensor data once at the end of each time step, to speed the simulation up
                // if agents cover too much distance in each timestep, we may need to process it more frequently
                //System.out.println(agent.toString() + "2 took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
                sensorData = simFramework.findSensorData(agent, nextStep);
                //System.out.println(agent.toString() + "3 took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
                agent.writeStep(nextStep, sensorData, true);
                //System.out.println(agent.toString() + "4 took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
            }
            else
            {
                System.out.println(agent + " !!! setting envError because direct line not possible between " 
                        + agent.getLocation() + " and " + nextStep);
                //Remove safe space status for the points along the line, so that obstacles can be sensed there
                LinkedList<Point> ptsNonSafe = 
                        agent.getOccupancyGrid().pointsAlongSegment(agent.getLocation().x, agent.getLocation().y, 
                                nextStep.x, nextStep.y);
                for (Point p: ptsNonSafe)
                    agent.getOccupancyGrid().setNoSafeSpaceAt(p.x, p.y);
                nextStep.x = agent.getX();
                nextStep.y = agent.getY();
                agent.setEnvError(true);
            }
            //</editor-fold>
                        
            //<editor-fold defaultstate="collapsed" desc="Conditions for breaking even if we have 'speed' left">
            boolean canContinueOnPath = (agent.getPath() != null) && (agent.getPath().getPoints() != null) && 
                    (agent.getPath().getPoints().size() > 0) && (!agent.getEnvError());
            if (!canContinueOnPath)
                break;
            
            if ((agent.getState() != BasicAgent.ExploreState.Explore)
                    && (agent.getState() != BasicAgent.ExploreState.GoToChild)
                    && (agent.getState() != BasicAgent.ExploreState.ReturnToParent)
                    && (agent.getState() != BasicAgent.ExploreState.Initial))
                break;
            if (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.RunFromLog)
                break;
            //</editor-fold>
        }
        //</editor-fold>
        /*if (nextStep != null) {
            sensorData = simFramework.findSensorData(agent, nextStep);
            agent.writeStep(nextStep, sensorData, true);
        }*/
        /*if (simConfig.getExpAlgorithm() != SimulatorConfig.exptype.RunFromLog)
            agent.updateTrueAreaKnown(env);*/
        //benchmark
        agent.incrementTimeLastCentralCommand();
        System.out.println(agent.toString() + "Agent cycle complete, took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
    }

}
