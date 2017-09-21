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

package simulator;

import agents.RealAgent;
import config.SimulatorConfig;
import environment.Environment;
import java.awt.Point;

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

        //Continue along the path,
        //until we have exhausted agent 'speed' per cycle or run out of path
        while (distance_left > 0) {
            //Get next step
            try {
                nextStep = agent.takeStep(timeElapsed);
            } catch (RuntimeException e) {
                System.err.println("AgentStepRunnable: " + e.toString());
                e.printStackTrace();
                agent.setEnvError(true);
                nextStep = new Point(agent.getLocation());
            }
            if (nextStep == null) {
                nextStep = agent.getLocation();
                agent.setEnvError(true);
                distance_left = 0;
                System.err.println(agent + " !!! setting envError because nextStep is null, "
                        + "distance_left is " + distance_left);
            }

            //Check to make sure step is legal
            if (env.legalMove(agent.getLocation(), nextStep, agent.ability)) {
                //check here we don't 'teleport'
                double dist = agent.getLocation().distance(nextStep);
                //If we don't have enough 'speed'
                // left to reach nextPoint, go as far as we can and keep nextPoint in the path
                if (dist > distance_left) {
                    //Could not do last step, set this back on step
                    if (agent.getPath() != null) {
                        agent.getPath().resetStep();
                    }
                    double ratio = distance_left / dist;
                    nextStep.x = agent.getX() + (int) Math.round((nextStep.x - agent.getX()) * ratio);
                    nextStep.y = agent.getY() + (int) Math.round((nextStep.y - agent.getY()) * ratio);
                    if (!env.legalMove(agent.getLocation(), nextStep, agent.ability)) {
                        nextStep.x = agent.getX();
                        nextStep.y = agent.getY();
                    }
                    distance_left = 0;
                } else {
                    distance_left = distance_left - dist;
                }
                // comment below out to process sensor data once at the end of each time step, to speed the simulation up
                // if agents cover too much distance in each timestep, we may need to process it more frequently
                sensorData = simFramework.findSensorData(agent, nextStep);
                agent.writeStep(nextStep, sensorData, true);
            } else {
                System.err.println(agent + " at cycle " + timeElapsed + ": setting envError because direct line not possible from ("
                        + (int) agent.getLocation().getX() + "," + (int) agent.getLocation().getY() + ") to (" + nextStep.x + "," + nextStep.y + "), distance: " + nextStep.distance(agent.getLocation()));
                //Remove safe space status for the points along the line, so that obstacles can be sensed there
                //if (nextStep.distance(agent.getLocation()) <= 4) {
                //We are bordering next step, and because we cannot move there it must be an obstacle
                int deltaX = agent.getLocation().x - nextStep.x;
                int deltaY = agent.getLocation().y - nextStep.y;
                if (deltaX <= 2 && deltaX >= -2 && deltaY <= 2 && deltaY >= -2) {
                    agent.getOccupancyGrid().setObstacleAt(agent.getLocation().x + (int) Math.ceil(deltaX / 2.0), agent.getLocation().y + (int) Math.ceil(deltaY / 2.0));
                    agent.getOccupancyGrid().setNoFreeSpaceAt(agent.getLocation().x + (int) Math.ceil(deltaX / 2.0), agent.getLocation().y + (int) Math.ceil(deltaY / 2.0));
                }
                //agent.getOccupancyGrid().setObstacleAt(nextStep.x, nextStep.y);
                //agent.getOccupancyGrid().setNoFreeSpaceAt(nextStep.x, nextStep.y);
//                    agent.getOccupancyGrid().setSafeSpaceAt(nextStep.x, nextStep.y);
                //} else {
                //there are several points between us and nextStep, so we don't know which one exactly has obstacle
//                    LinkedList<Point> ptsNonSafe
//                            = agent.getOccupancyGrid().pointsAlongSegment(agent.getLocation().x, agent.getLocation().y,
//                                    nextStep.x, nextStep.y);
//                    ptsNonSafe.stream().filter((p) -> (!p.equals(agent.getLocation()))).forEach((p) -> {
//                        agent.getOccupancyGrid().setNoSafeSpaceAt(p.x, p.y);
//                    });
                //}
                if (agent.getPath() != null) {
                    agent.getPath().resetStep();
                }
                agent.setEnvError(true);
            }
            agent.flush();

            //Conditions for breaking even if we have 'speed' left
            boolean canContinueOnPath = (agent.getPath() != null) && !agent.getPath().isFinished() && (agent.getPath().getPoints() != null)
                    && (agent.getPath().getPoints().size() > 0) && (!agent.getEnvError());
            if (!canContinueOnPath) {
                break;
            }

            /*if ((agent.getState() != Agent.AgentState.Explore)
                    && (agent.getState() != Agent.AgentState.GoToChild)
                    && (agent.getState() != Agent.AgentState.ReturnToBaseStation)
                    && (agent.getState() != Agent.AgentState.Initial)
                    && (agent.getState() != Agent.AgentState.AKTIVE)) {
                break;
            }*/
            if (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.RunFromLog) {
                break;
            }
            if (agent.isStepFinished()) {
                agent.setStepFinished(false);
                break;
            }
        }

        /*if (simConfig.getExpAlgorithm() != SimulatorConfig.exptype.RunFromLog)
            agent.updateTrueAreaKnown(env);*/
        //benchmark
        agent.getStats().incrementTimeLastCentralCommand();
    }

}
