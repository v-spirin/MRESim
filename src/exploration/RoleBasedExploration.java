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
import exploration.rendezvous.IRendezvousStrategy;
import exploration.rendezvous.RendezvousAgentData;
import java.awt.Point;

/**
 *
 * @author julh, vspirin, Christian Clausen
 */
public class RoleBasedExploration extends FrontierExploration {

    int timeElapsed;
    IRendezvousStrategy rendezvousStrategy;

    public RoleBasedExploration(int timeElapsed, RealAgent agent, SimulatorConfig simConfig, IRendezvousStrategy rendezvousStrategy, RealAgent baseStation) {
        super(agent, simConfig, baseStation, SimulatorConfig.frontiertype.ReturnWhenComplete);
        this.timeElapsed = timeElapsed;
        this.rendezvousStrategy = rendezvousStrategy;
    }

    /**
     * Returns the next step for the Agent
     *
     * @param timeElapsed
     * @return
     */
    @Override
    public Point takeStep(int timeElapsed) {
        this.timeElapsed = timeElapsed;

        Point nextStep = null;
        //Run correct takeStep function depending on agent state, set nextStep to output
        switch (agent.getExploreState()) {
            case Initial:
                nextStep = takeStep_Initial();
                break;
            case Explore:
                nextStep = takeStep_Explore();
                break;
            case GoToParent:
                nextStep = takeStep_GoToParent();
                break;
            case WaitForParent:
                nextStep = takeStep_WaitForParent();
                break;
            case GoToChild:
            case WaitForChild:
                nextStep = takeStep_GoToChild();
                break;
            case ReturnToBase:
                nextStep = takeStep_ReturnToBase(Agent.ExplorationState.Finished);
            default:
                break;
        }

        // this shouldn't happen, looks like one of takeSteps returned an error
        if (nextStep == null) {
            nextStep = RandomWalk.randomStep(agent);
        }

        agent.setStateTimer(agent.getStateTimer() + 1);
        agent.getRendezvousAgentData().setTimeSinceLastRVCalc(agent.getRendezvousAgentData().getTimeSinceLastRVCalc() + 1);
        return nextStep;
    }

    public Point takeStep_Initial() {
        // Small number of random steps to get initial range data
        // First 3 steps? Explorers take 2 random steps while others wait, then everyone takes a random step
        if (agent.getStateTimer() < 2) {
            if (agent.isExplorer()) {
                return RandomWalk.randomStep(agent);
            } else {
                return agent.getLocation();
            }
        } else if (agent.getStateTimer() < 3) {
            return RandomWalk.randomStep(agent);
        } // Otherwise? Explorers go into Explore state, others go into GoToChild state. Explorers replan using FrontierExploration, others do nothing.
        else if (agent.isExplorer()) {
            agent.setExploreState(RealAgent.ExplorationState.Explore);
            agent.getStats().setTimeSinceLastPlan(0);
            return replan(0);
        } else {
            agent.setExploreState(RealAgent.ExplorationState.GoToChild);
            return agent.getLocation();
        }
    }

    public Point takeStep_Explore() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        if (rvd.getParentRendezvous().getTimeMeeting() <= timeElapsed) {
            agent.setExploreState(Agent.ExplorationState.GoToParent);
            return agent.getLocation();
        }

        Point nextStep = super.takeStep(timeElapsed);

        return nextStep;
    }

    public Point takeStep_GoToParent() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //If parent is in range, GiveParentInfo
        if (agent.getParentTeammate().hasCommunicationLink()) {
            if (agent.isExplorer()) {
                rendezvousStrategy.calculateRendezvousExplorerWithRelay(timeElapsed);
                agent.setExploreState(Agent.ExplorationState.Explore);
            } else {
                rendezvousStrategy.calculateRendezvousRelayWithRelay();
                agent.setExploreState(Agent.ExplorationState.GoToChild);
            }
            return agent.getLocation();
        }

        if (agent.getLocation().equals(rvd.getParentRendezvous().getChildLocation())) {
            agent.setExploreState(RealAgent.ExplorationState.WaitForParent);
        }
        if (agent.getStateTimer() <= 1) {
            //Just changed to this state so need to generate path
            agent.calculatePath(rvd.getParentRendezvous().getChildLocation());
        }
        return agent.getNextPathPoint();

    }

    public Point takeStep_WaitForParent() {
        if (agent.getParentTeammate().hasCommunicationLink()) {
            if (agent.isExplorer()) {
                rendezvousStrategy.calculateRendezvousExplorerWithRelay(timeElapsed);
                agent.setExploreState(RealAgent.ExplorationState.Explore);
            } else {
                rendezvousStrategy.calculateRendezvousRelayWithRelay();
                agent.setExploreState(RealAgent.ExplorationState.GoToChild);
            }
        }
        return agent.getLocation();

    }

    public Point takeStep_WaitForChild() {
        if (agent.getParentTeammate().hasCommunicationLink()) {
            rendezvousStrategy.calculateRendezvousRelayWithRelay();
            agent.setExploreState(RealAgent.ExplorationState.GoToParent);
        }
        return agent.getLocation();

    }

    public Point takeStep_GoToChild() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //If child is in range
        if (agent.getChildTeammate().hasCommunicationLink()) {
            rendezvousStrategy.calculateRendezvousRelayWithRelay();
            agent.setExploreState(Agent.ExplorationState.GoToParent);
            return agent.getLocation();
        }

        if (agent.getLocation().equals(rvd.getChildRendezvous().getParentLocation())) {
            agent.setExploreState(RealAgent.ExplorationState.WaitForChild);
        }
        if (agent.getStateTimer() <= 1) {
            //Just changed to this state so need to generate path
            agent.calculatePath(rvd.getChildRendezvous().getParentLocation());
        }
        return agent.getNextPathPoint();
    }

}
