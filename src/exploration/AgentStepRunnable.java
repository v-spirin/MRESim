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

import agents.BasicAgent;
import agents.RealAgent;
import config.Constants;
import config.SimulatorConfig;
import environment.Environment;
import java.awt.Point;
import java.util.LinkedList;

/**
 *
 * @author Victor
 */
public class AgentStepRunnable implements Runnable {

    private RealAgent agent;
    private SimulatorConfig simConfig;
    private int timeElapsed;
    private Environment env;
    private SimulationFramework simFramework;

    AgentStepRunnable(RealAgent agent, SimulatorConfig simConfig, int timeElapsed,
            Environment env, SimulationFramework simFramework) {
        this.agent = agent;
        this.simConfig = simConfig;
        this.timeElapsed = timeElapsed;
        this.env = env;
        this.simFramework = simFramework;
    }

    @Override
    public void run() {
        Point nextStep;
        double[] sensorData;
        double distance_left = agent.getSpeed();
        //profiling
        long realtimeStartAgentCycle = System.currentTimeMillis();

        //<editor-fold defaultstate="collapsed" desc="Continue along the path,">
        //until we have exhausted agent 'speed' per cycle or run out of path
        while (distance_left > 0) {
            //<editor-fold defaultstate="collapsed" desc="Get next step">
            nextStep = agent.takeStep(timeElapsed);
            if (nextStep == null) {
                nextStep = agent.getLocation();
                agent.setEnvError(true);
                distance_left = 0;
                System.err.println(agent + " !!! setting envError because nextStep is null, "
                        + "distance_left is " + distance_left);
            }
            agent.flush();
            if (Constants.PROFILING) {
                System.out.println(agent.toString() + "Get next step (" + nextStep.x + "," 
                        + nextStep.y + ") took " 
                        + (System.currentTimeMillis() - realtimeStartAgentCycle) + "ms.");
            }
            //</editor-fold>

            //<editor-fold defaultstate="collapsed" desc="Check to make sure step is legal">
            if (env.legalMove(agent.getX(), agent.getY(), nextStep.x, nextStep.y, agent.ability)) {
                //check here we don't 'teleport'
                double dist = agent.getLocation().distance(nextStep);
                //<editor-fold defaultstate="collapsed" desc="If we don't have enough 'speed'">
                // left to reach nextPoint, go as far as we can and keep nextPoint in the path
                if (dist > distance_left) {
                    //System.out.println(agent.toString() + " exceeded speed. Distance left: " + distance_left + ", dist to next path point: " + dist);
                    //Add nextStep back to path, as we will not reach it yet
                    if ((agent.getPath() != null) && (agent.getPath().getPoints() != null)) {
                        agent.getPath().getPoints().add(0, nextStep);
                    }
                    double ratio = distance_left / dist;
                    nextStep.x = agent.getX() + (int) Math.round((nextStep.x - agent.getX()) * ratio);
                    nextStep.y = agent.getY() + (int) Math.round((nextStep.y - agent.getY()) * ratio);
                    if (!env.legalMove(agent.getX(), agent.getY(), nextStep.x, nextStep.y, agent.ability)) {
                        nextStep.x = agent.getX();
                        nextStep.y = agent.getY();
                        if (Constants.DEBUG_OUTPUT) {
                            System.out.println(agent.toString() + " directLinePossible returned wrong result!");
                        }
                    }
                    //System.out.println(agent.toString() + " speed corrected. Now is: " + agent.getLocation().distance(nextStep));
                    distance_left = 0;
                    //</editor-fold>
                } else {
                    distance_left = distance_left - dist;
                }
                // comment below out to process sensor data once at the end of each time step, to speed the simulation up
                // if agents cover too much distance in each timestep, we may need to process it more frequently
                //System.out.println(agent.toString() + "2 took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
                sensorData = simFramework.findSensorData(agent, nextStep);
                //System.out.println(agent.toString() + "3 took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
                agent.writeStep(nextStep, sensorData, true);
                //System.out.println(agent.toString() + "4 took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
            } else {
                //if (Constants.DEBUG_OUTPUT) {
                System.err.println(agent + " !!! setting envError because direct line not possible between ("
                        + (int) agent.getLocation().getX() + "," + (int) agent.getLocation().getY() + ") and (" + nextStep.x + "," + nextStep.y + ")");
                //}
                //Remove safe space status for the points along the line, so that obstacles can be sensed there
                if (nextStep.distance(agent.getLocation()) == 1) {
                    //We are bordering next step, and because we cannot move there it must be an obstacle
                    agent.getOccupancyGrid().setObstacleAt(nextStep.x, nextStep.y);
                    agent.getOccupancyGrid().setNoFreeSpaceAt(nextStep.x, nextStep.y);
                    agent.getOccupancyGrid().setSafeSpaceAt(nextStep.x, nextStep.y);
                } else {
                    //there are several points between us and nextStep, so we don't know which one exactly has obstacle
                    LinkedList<Point> ptsNonSafe
                            = agent.getOccupancyGrid().pointsAlongSegment(agent.getLocation().x, agent.getLocation().y,
                                    nextStep.x, nextStep.y);
                    ptsNonSafe.stream().filter((p) -> (!p.equals(agent.getLocation()))).forEach((p) -> {
                        agent.getOccupancyGrid().setNoSafeSpaceAt(p.x, p.y);
                    });
                }
                nextStep.x = agent.getX();
                nextStep.y = agent.getY();
                agent.setEnvError(true);
            }
            //</editor-fold>

            //<editor-fold defaultstate="collapsed" desc="Conditions for breaking even if we have 'speed' left">
            boolean canContinueOnPath = (agent.getPath() != null) && (agent.getPath().getPoints() != null)
                    && (agent.getPath().getPoints().size() > 0) && (!agent.getEnvError());
            if (!canContinueOnPath) {
                break;
            }

            if ((agent.getState() != BasicAgent.ExploreState.Explore)
                    && (agent.getState() != BasicAgent.ExploreState.GoToChild)
                    && (agent.getState() != BasicAgent.ExploreState.ReturnToParent)
                    && (agent.getState() != BasicAgent.ExploreState.Initial)) {
                break;
            }
            if (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.RunFromLog) {
                break;
            }
            //</editor-fold>
        }
        //</editor-fold>

 /*if (simConfig.getExpAlgorithm() != SimulatorConfig.exptype.RunFromLog)
            agent.updateTrueAreaKnown(env);*/
        //benchmark
        agent.getStats().incrementTimeLastCentralCommand();
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent.toString() + "Agent cycle complete, took " + (System.currentTimeMillis() - realtimeStartAgentCycle) + "ms.");
        }
    }

}
