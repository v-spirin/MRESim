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
import config.SimulatorConfig;
import environment.OccupancyGrid;
import environment.TopologicalMap;
import java.awt.Point;

/**
 *
 * @author Christian Clausen <christian.clausen@uni-bremen.de>
 */
public class RandomExploration extends BasicExploration implements Exploration {

    SimulatorConfig.relaytype relayType = SimulatorConfig.relaytype.None;
    private final OccupancyGrid occGrid;
    TopologicalMap tmap;

    public RandomExploration(RealAgent agent, SimulatorConfig simConfig) {
        super(agent, simConfig);
        this.relayType = simConfig.getRelayAlgorithm();
        this.occGrid = agent.getOccupancyGrid();
        state = ExplorationState.Exploring;
        tmap = new TopologicalMap(occGrid);
    }

    @Override
    public Point takeStep(int timeElapsed) {
        switch (relayType) {
            case Random:
                if ((Math.random() < simConfig.getComStationDropChance() * agent.getSpeed())) {
                    agent.dropComStation();
                }
                break;
            case KeyPoints:
                if (!agent.comStations.isEmpty()) {
                    if (timeElapsed % 5 == 0) {
                        tmap.generateSkeleton();
                    }
                    for (Point p : tmap.getJunctionPoints()) {
                        simulator.ExplorationImage.addErrorMarker(p, "", true);
                        if (agent.getLocation().distance(p) < Constants.KEY_POINT_RELAY_DISTANCE) {
                            if (noRelay(p) && noNearRelay(p)) {
                                state = ExplorationState.SettingRelay;
                                break;
                            }
                        }

                    }
                }
                break;
            case RangeBorder:
                break;
            case None:
            default:
            //Nothing
        }

        if (state == ExplorationState.SettingRelay) {
            agent.dropComStation();
            state = ExplorationState.Exploring;
        }
        return RandomWalk.randomStep(agent);
    }

    @Override
    protected Point replan(int timeElapsed
    ) {
        throw new UnsupportedOperationException("Not supported, this does not need a plan.");
    }
}
