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
import environment.OccupancyGrid;
import java.awt.Point;
import path.Path;

/**
 *
 * @author Christian Clausen <christian.clausen@uni-bremen.de>
 */
public class WallFollowExploration extends BasicExploration implements Exploration {

    private final OccupancyGrid grid;
    private double TURN_HEADING = Math.PI * 0.1;

    public WallFollowExploration(RealAgent agent, OccupancyGrid grid) {
        super(agent);
        System.out.println("Start Wall");
        this.grid = grid;
    }

    @Override
    public Point takeStep(int timeElapsed) {
        if (path == null || path.isFinished()) {
            double direction = agent.getHeading() + (0.5 * Math.PI);
            Point nearPoint = environment.Environment.getPointFromDirection(agent.getLocation(), direction + (Math.PI / 2), agent.getSpeed() / 2);
            Point farPoint = environment.Environment.getPointFromDirection(agent.getLocation(), direction + (Math.PI / 2), agent.getSpeed() / 2 + agent.getSpeed());
            Point nextPoint = null;
            if (grid.freeSpaceAt(nearPoint) && grid.freeSpaceAt(farPoint)) {
                //distance from wall, turn right
                System.out.println("Right");
                nextPoint = environment.Environment.getPointFromDirection(agent.getLocation(), direction + TURN_HEADING, agent.getSpeed());
            }
            if (grid.freeSpaceAt(nearPoint) && !grid.freeSpaceAt(farPoint)) {
                //wall beginns between meassurepoints, keep going
                System.out.println("Head On");
                nextPoint = environment.Environment.getPointFromDirection(agent.getLocation(), direction, agent.getSpeed());
            }
            if (!grid.freeSpaceAt(nearPoint)) {
                //not enough distance from wall, turn left
                System.out.println("Left");
                nextPoint = environment.Environment.getPointFromDirection(agent.getLocation(), direction - TURN_HEADING, agent.getSpeed());
            }
            path = new Path(grid, agent.getLocation(), nextPoint, true, false);
        } else {
            System.out.println("Cached");
        }
        return path.nextPoint();
    }

    @Override
    protected Point replan(int timeElapsed) {
        throw new UnsupportedOperationException("Not supported, this dows not need a plan");
    }

    public void forceReplan() {
        path = null;
    }
}
