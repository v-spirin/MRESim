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

import agents.RealAgent;
import config.Constants;
import java.awt.Point;
import java.util.Random;
import path.Path;

/**
 *
 * @author julh
 */
public class RandomWalk implements Exploration {

    public static Random generator = new Random(Constants.RANDOM_SEED);
    RealAgent agent;

    public RandomWalk(RealAgent agent) {
        this.agent = agent;
    }

    @Override
    public Point takeStep(int timeElapsed) {
        int maxcounter = 100;
        int ranVar;
        int newX = agent.getX();
        int newY = agent.getY();
        int counter = 0;

        boolean found = false;

        int acceptableDistanceToWall = Constants.WALL_DISTANCE;
        double speed = agent.getSpeed();

        while (!found && counter < maxcounter) {
            // if after 50 iterations, we couldn't find new location, relax the condition about min distance to nearest wall
            if (counter > (maxcounter / 2)) {
                acceptableDistanceToWall = 1;
            }
            ranVar = generator.nextInt(26);

            if (ranVar == 0) {
                agent.setHeading(agent.getHeading() - Math.PI / 4);
            } else if (ranVar < 3) {
                agent.setHeading(agent.getHeading() - Math.PI / 8);
            } else if (ranVar < 23) {
                agent.setHeading(agent.getHeading() + 0);
            } else if (ranVar < 25) {
                agent.setHeading(agent.getHeading() + Math.PI / 8);
            } else {
                agent.setHeading(agent.getHeading() + Math.PI / 4);
            }

            if (agent.getHeading() >= Math.PI) {
                agent.setHeading(agent.getHeading() - 2 * Math.PI);
            }
            if (agent.getHeading() < -1 * Math.PI) {
                agent.setHeading(agent.getHeading() + 2 * Math.PI);
            }

            newX = agent.getX() + Math.round((float) (speed * Math.cos(agent.getHeading())));
            newY = agent.getY() + Math.round((float) (speed * Math.sin(agent.getHeading())));

            if (agent.getOccupancyGrid().locationExists(newX, newY)
                    && agent.getOccupancyGrid().directLinePossible(agent.getX(), agent.getY(), newX, newY)
                    && !agent.getOccupancyGrid().obstacleWithinDistance(newX, newY, acceptableDistanceToWall)) {
                found = true;
            }

            counter++;
            if (speed > 3) {
                speed = speed - 1;
            }
        }
        // couldn't get a new location, try again but relax the condition about being next to a wall.
        while (!found && counter < 50) {
            ranVar = generator.nextInt(26);

            if (ranVar == 0) {
                agent.setHeading(agent.getHeading() - Math.PI / 4);
            } else if (ranVar < 3) {
                agent.setHeading(agent.getHeading() - Math.PI / 8);
            } else if (ranVar < 23) {
                agent.setHeading(agent.getHeading() + 0);
            } else if (ranVar < 25) {
                agent.setHeading(agent.getHeading() + Math.PI / 8);
            } else {
                agent.setHeading(agent.getHeading() + Math.PI / 4);
            }

            if (agent.getHeading() >= Math.PI) {
                agent.setHeading(agent.getHeading() - 2 * Math.PI);
            }
            if (agent.getHeading() < -1 * Math.PI) {
                agent.setHeading(agent.getHeading() + 2 * Math.PI);
            }

            newX = agent.getX() + Math.round((float) (speed * Math.cos(agent.getHeading())));
            newY = agent.getY() + Math.round((float) (speed * Math.sin(agent.getHeading())));

            if (agent.getOccupancyGrid().locationExists(newX, newY)
                    && agent.getOccupancyGrid().directLinePossible(agent.getX(), agent.getY(), newX, newY)
                    && !agent.getOccupancyGrid().obstacleWithinDistance(newX, newY, 1)) {
                found = true;
            }

            counter++;

            if (speed > 3) {
                speed = speed - 1;
            }
        }
        Point newPoint = new Point(newX, newY);
        Path path = new Path(agent.getOccupancyGrid(), agent.getLocation(), newPoint, true, true);
        agent.setPath(path);

        return (new Point(newX, newY));
    }

    @Override
    public Point replan(int timeElapsed) {
        return takeStep(timeElapsed);
    }

    public static Point takeStep(RealAgent agent) {
        return new RandomWalk(agent).takeStep(0);
    }
}
