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
import agents.TeammateAgent;
import config.SimConstants;
import config.SimulatorConfig;
import java.awt.Point;
import path.Path;

/**
 * Class to inherit from to share some code for all Explorations. Do not use this as type, use
 * Interface 'Exploration' instead
 *
 * @author Christian Clausen
 */
abstract class BasicExploration {

    /**
     * The agent using this ExplorationStrategy.
     */
    RealAgent agent;

    /**
     * The planned path, null if no takeStep_explore called.
     */
    Path path;

    Agent.ExplorationState state = Agent.ExplorationState.Initial;

    SimulatorConfig simConfig;

    String command = "";
    Integer command_data = -1;
    boolean recentEnvError = false;
    int envErrorCounter = 0;

    /**
     * Just builds the object and initializes the agent.
     *
     * @param agent The agend using this ExplorationStrategy
     */
    public BasicExploration(RealAgent agent, SimulatorConfig simConfig, Agent.ExplorationState initialState) {
        this.agent = agent;
        this.simConfig = simConfig;
        this.state = initialState;
    }

    /**
     * Recalculates the current plan. Only need to call this after severe map-changes or similar
     * situations. Will be called by takeStep if necessary.
     *
     * @param timeElapsed Cycle we are in currently
     * @return Nothing ASAP! //TODO no return!
     */
    abstract protected Point takeStep_explore(int timeElapsed);

    protected boolean noRelay(Point p) {

        for (TeammateAgent agt : agent.getAllTeammates().values()) {
            if (agt.isStationary() && agt.getLocation().equals(p)) {
                return false;
            }
        }
        return true;
    }

    protected boolean noNearRelay(Point p) {

        for (TeammateAgent agt : agent.getAllTeammates().values()) {
            if (agt.isStationary() && (agt.getLocation().distance(p) < SimConstants.MIN_RELAY_DISTANCE)) {
                return false;
            }
        }
        return true;
    }

    protected Point takeStep_ReturnToBase(Agent.ExplorationState followUp) {
        agent.setPathToBaseStation(false);
        if (agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID).isInHandoverRange(agent)) {
            agent.setExploreState(followUp);
        }
        return agent.getPath().nextPoint();
    }

    public void setCommand(String command) {
        this.command = command;
    }

    public void setCommandData(Integer command_data) {
        this.command_data = command_data;
    }
}
