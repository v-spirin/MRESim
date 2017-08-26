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

import agents.Agent;
import agents.RealAgent;
import config.SimulatorConfig;
import environment.OccupancyGrid;
import java.awt.Point;
import path.Path;

/**
 *
 * @author Christian Clausen <christian.clausen@uni-bremen.de>
 */
public class WallFollowExploration extends BasicExploration implements Exploration {

    private OccupancyGrid grid;
    private double TURN_HEADING = Math.PI * 0.15;
    private int MEASURE_DISTANCE = 20;
    private int SENSING_DISTANCE = 30;

    public WallFollowExploration(RealAgent agent, SimulatorConfig simConfig, OccupancyGrid grid) {
        super(agent, simConfig, Agent.ExplorationState.Explore);
        System.out.println("Start Wall");
        this.grid = grid;
    }

    @Override
    public Point takeStep(int timeElapsed) {
        //if (path == null || path.isFinished()) {
        double senseDirection = agent.getHeading() + (0.5 * Math.PI);
        double moveDirection = agent.getHeading();
        Point nearPoint = environment.Environment.getPointFromDirection(agent.getLocation(), senseDirection, SENSING_DISTANCE);
        Point farPoint = environment.Environment.getPointFromDirection(agent.getLocation(), senseDirection, SENSING_DISTANCE + MEASURE_DISTANCE);
        Point nextPoint;
        if (!grid.directLinePossible(agent.getLocation(), nearPoint, true, true)) {
            //not enough distance from wall, turn left
            System.out.println("Left");
            nextPoint = environment.Environment.getPointFromDirection(agent.getLocation(), moveDirection - TURN_HEADING, agent.getSpeed());
            /*while (grid.freeSpaceAt(nextPoint)) {
                    heading -= TURN_HEADING;
                    nextPoint = environment.getPointFromDirection(agent.getLocation(), moveDirection - TURN_HEADING, agent.getSpeed());
                }*/
        } else if (!grid.directLinePossible(nearPoint, farPoint, true, true)) {
            //wall beginns between meassurepoints, keep going
            System.out.println("Head On");
            nextPoint = environment.Environment.getPointFromDirection(agent.getLocation(), moveDirection, agent.getSpeed());
        } else {
            //distance from wall, turn right
            System.out.println("Right");
            nextPoint = environment.Environment.getPointFromDirection(agent.getLocation(), moveDirection + TURN_HEADING, agent.getSpeed());

        }

        double heading = moveDirection - TURN_HEADING;
        int counter = 0;
        while (counter < 10 && !grid.obstacleAt(nextPoint)) {
            counter++;
            heading -= TURN_HEADING;
            nextPoint = environment.Environment.getPointFromDirection(agent.getLocation(), heading, agent.getSpeed());
        }

        simulator.ExplorationImage.addErrorMarker(nextPoint, "->", true);
        path = new Path(grid, agent.getLocation(), nextPoint, true, false);
        agent.setPath(path);
        //}
        return path.getPoints().get(1);
    }

    @Override
    protected Point replan(int timeElapsed) {
        throw new UnsupportedOperationException("Not supported, this dows not need a plan");
    }

    public void forceReplan() {
        path = null;
    }

    public void updateGrid(OccupancyGrid grid) {
        this.grid = grid;
    }
}
