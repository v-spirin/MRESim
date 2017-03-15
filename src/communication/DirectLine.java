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
package communication;

import environment.Environment;
import agents.RealAgent;

/**
 *
 * @author julh
 */
public class DirectLine {

    public static int[][] detectCommunication(Environment env, RealAgent[] agent) {
        int commTable[][] = new int[agent.length][agent.length];

        for (int i = 0; i < agent.length - 1; i++) {
            for (int j = i + 1; j < agent.length; j++) {
                commTable[i][j] = 0;
            }
        }

        for (int i = 0; i < agent.length - 1; i++) {
            for (int j = i + 1; j < agent.length; j++) {
                if (agent[i].distanceTo(agent[j]) < (agent[i].getCommRange() + agent[j].getCommRange())
                        && env.directLinePossible(agent[i].getX(), agent[i].getY(), agent[j].getX(), agent[j].getY(), 3)) {
                    commTable[i][j] = 1;
                    commTable[j][i] = 1;
                }
            }
        }

        return commTable;
    }
}
