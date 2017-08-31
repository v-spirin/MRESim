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
    RendezvousAgentData rvd;
    RendezvousAgentData prvd;
    RendezvousAgentData crvd;

    public RoleBasedExploration(int timeElapsed, RealAgent agent, SimulatorConfig simConfig, IRendezvousStrategy rendezvousStrategy, RealAgent baseStation) {
        super(agent, simConfig, baseStation, SimulatorConfig.frontiertype.ReturnWhenComplete);
        this.timeElapsed = timeElapsed;
        this.rendezvousStrategy = rendezvousStrategy;
        this.rvd = agent.getRendezvousAgentData();
        this.prvd = agent.getParentTeammate().getRendezvousAgentData();
        if (agent.getChildTeammate() != null) {
            this.crvd = agent.getChildTeammate().getRendezvousAgentData();
        }
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

        calculateRendezvous(timeElapsed);
        Point nextStep = null;
        //Run correct takeStep function depending on agent state, set nextStep to output
        switch (agent.getExploreState()) {
            case Initial:
                nextStep = takeStep_Initial();
                break;
            case Explore:
                nextStep = takeStep_Explore();
                break;
            case WaitForParent:
            case GoToParent:
                nextStep = takeStep_GoToParent();
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

        calculateRendezvous(timeElapsed);
        agent.incrementStateTimer();
        return nextStep;
    }

    private void calculateRendezvous(int timeElapsed1) {
        if (!agent.isExplorer()) {
            if (agent.getChildTeammate().hasCommunicationLink()) {
                rvd.setChildRendezvous(rendezvousStrategy.calculateRendezvous(timeElapsed1, agent.getChildTeammate()));
                rvd.getChildRendezvous().setTimeMeeting(timeElapsed1 + 50);
            }
            if (agent.getParentTeammate().hasCommunicationLink()) {
                rvd.setParentRendezvous(rendezvousStrategy.calculateRendezvous(timeElapsed1, agent.getParentTeammate()));
                rvd.getParentRendezvous().setTimeMeeting(timeElapsed1 + 50);
            }
            /*int meeting = Math.min(prvd.getChildRendezvous().getTimeMeeting(),
                    rvd.getParentRendezvous().getTimeMeeting());
            meeting = Math.min(crvd.getParentRendezvous().getTimeMeeting(), meeting);
            meeting = Math.min(crvd.getChildRendezvous().getTimeMeeting(), meeting);
            agent.setDynamicInfoText("" + (meeting - timeElapsed));*/
            agent.setDynamicInfoText("--");
        } else {
            agent.setDynamicInfoText("" + (agent.getParentTeammate().getRendezvousAgentData().getChildRendezvous().getTimeMeeting() - timeElapsed));
        }
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
        if (agent.getParentTeammate().getRendezvousAgentData().getChildRendezvous().getTimeMeeting() <= timeElapsed) {
            agent.setExploreState(Agent.ExplorationState.GoToParent);
            return agent.getLocation();
        }

        if (agent.getStateTimer() <= 1) {
            super.replan(timeElapsed);
        }

        return super.takeStep(timeElapsed);
    }

    public Point takeStep_GoToParent() {
        if (agent.getParentTeammate().hasCommunicationLink()) {
            if (agent.isExplorer()) {
                agent.setExploreState(Agent.ExplorationState.Explore);
            } else {
                agent.setExploreState(Agent.ExplorationState.GoToChild);
            }
            return agent.getLocation();
        }

        if (agent.getLocation().equals(prvd.getChildRendezvous().getChildLocation())) {
            agent.setExploreState(RealAgent.ExplorationState.WaitForParent);
            return agent.getLocation();
        }
        if (agent.getStateTimer() <= 1) {
            //Just changed to this state so need to generate path
            agent.setPath(agent.calculatePath(prvd.getChildRendezvous().getChildLocation()));
        }
        return agent.getNextPathPoint();

    }

    public Point takeStep_GoToChild() {
        //If child is in range
        if (agent.getChildTeammate().hasCommunicationLink()) {
            agent.setExploreState(Agent.ExplorationState.GoToParent);
            return agent.getLocation();
        }

        if (agent.getLocation().equals(rvd.getChildRendezvous().getParentLocation())) {
            agent.setExploreState(RealAgent.ExplorationState.WaitForChild);
            return agent.getLocation();
        }
        if (agent.getExploreState() != Agent.ExplorationState.WaitForChild && agent.getStateTimer() <= 1) {
            //Just changed to this state so need to generate path
            agent.setPath(agent.calculatePath(rvd.getChildRendezvous().getParentLocation()));
        }
        return agent.getNextPathPoint();
    }

}
