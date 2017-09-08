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
import environment.TopologicalMap;
import exploration.rendezvous.IRendezvousStrategy;
import exploration.rendezvous.NearRVPoint;
import exploration.rendezvous.RendezvousAgentData;
import java.awt.Point;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.PriorityQueue;
import path.TopologicalNode;

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
    RendezvousAgentData trvd; //temp rendezvous-data for switching childs
    SimulatorConfig.relaytype relayType = SimulatorConfig.relaytype.None;
    TopologicalMap tmap;
    Boolean switchedChildAndNeedToTell = false;

    public RoleBasedExploration(int timeElapsed, RealAgent agent, SimulatorConfig simConfig, IRendezvousStrategy rendezvousStrategy, RealAgent baseStation) {
        super(agent, simConfig, baseStation, SimulatorConfig.frontiertype.ReturnWhenComplete);
        this.timeElapsed = timeElapsed;
        this.rendezvousStrategy = rendezvousStrategy;
        this.relayType = simConfig.getRelayAlgorithm();
        tmap = agent.getTopologicalMap();
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

        if (agent.getEnvError()) {
            agent.setEnvError(false);
            agent.resetPathToBaseStation();
            agent.setExploreState(Agent.ExplorationState.EnvError);
        }

        if (command.equals("new_parent")) {
            agent.setParent(command_data);
            command = "";
            rvd.setParentRendezvous(rendezvousStrategy.calculateRendezvous(timeElapsed, agent.getParentTeammate()));
            int time_for_path = (int) agent.calculatePath(agent.getFrontier().getCentre()).getLength() / agent.getSpeed();
            time_for_path = Math.max(30, (int) (time_for_path * 1.5));
            rvd.getParentRendezvous().setTimeMeeting(timeElapsed + time_for_path);
            agent.setDynamicInfoText("" + (rvd.getParentRendezvous().getTimeMeeting() - timeElapsed));
        } else if (command.equals("reset_parent")) {
            agent.setParent(agent.getOriginalParent());
            command = "";
        }

        calculateRendezvous(timeElapsed);
        Point nextStep;
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
                break;
            case SettingRelay:
                Integer relayId = agent.dropComStation();

                if (relayType == SimulatorConfig.relaytype.BufferRelay && relayId >= 0) {
                    trvd = new RendezvousAgentData(rvd);
                    agent.setChild(relayId);
                    agent.command = "new_parent";
                    agent.command_data = relayId;
                    switchedChildAndNeedToTell = true;
                }

                agent.setExploreState(agent.getPrevExploreState());
                nextStep = agent.stay();
                break;
            case TakingRelay:
                agent.liftComStation();
                if (relayType == SimulatorConfig.relaytype.BufferRelay) {
                    agent.setChild(agent.getOriginalChild());
                    agent.command = "reset_parent";
                }
                agent.setExploreState(agent.getPrevExploreState());
                nextStep = agent.stay();
                break;
            case GoToRelay:
                nextStep = takeStep_GoToRelay();
                break;
            case EnvError:
                nextStep = RandomWalk.randomStep(agent);
                agent.setExploreState(agent.getPrevExploreState());
                break;
            default:
                nextStep = RandomWalk.randomStep(agent);
                break;
        }

        calculateRendezvous(timeElapsed);
        agent.incrementStateTimer();
        return nextStep;
    }

    private void calculateRendezvous(int timeElapsed1) {
        if (!agent.isExplorer()) {
            if (agent.getParentTeammate().hasCommunicationLink()) {
                rvd.setParentRendezvous(rendezvousStrategy.calculateRendezvous(timeElapsed1, agent.getParentTeammate()));
                rvd.getParentRendezvous().setTimeMeeting(timeElapsed1 + 50);
            }
            if (agent.getChildTeammate().hasCommunicationLink()) {
                rvd.setChildRendezvous(rendezvousStrategy.calculateRendezvous(timeElapsed1, agent.getChildTeammate()));
                int time_for_path = (int) agent.calculatePath(rvd.getParentRendezvous().getChildLocation()).getLength() / agent.getSpeed();
                time_for_path = Math.max(50, (int) (time_for_path * 1.5));
                rvd.getChildRendezvous().setTimeMeeting(timeElapsed1 + time_for_path);
            }
            agent.setDynamicInfoText("--");
        } else if (agent.getParentTeammate().isStationary() && agent.getParentTeammate().hasCommunicationLink()) {
            rvd.setParentRendezvous(rendezvousStrategy.calculateRendezvous(timeElapsed1, agent.getParentTeammate()));
            int time_for_path = (int) agent.calculatePath(agent.getFrontier().getCentre()).getLength() / agent.getSpeed();
            time_for_path = Math.max(30, (int) (time_for_path * 1.5));
            rvd.getParentRendezvous().setTimeMeeting(timeElapsed1 + time_for_path);
            agent.setDynamicInfoText("" + (rvd.getParentRendezvous().getTimeMeeting() - timeElapsed));
        } else {
            agent.setDynamicInfoText("" + (prvd.getChildRendezvous().getTimeMeeting() - timeElapsed));
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
            return agent.stay();
        }
    }

    public Point takeStep_Explore() {
        if (!agent.getParentTeammate().isStationary() && prvd.getChildRendezvous().getTimeMeeting() <= timeElapsed) {
            agent.setExploreState(Agent.ExplorationState.GoToParent);
            return agent.stay();
        } else if (agent.getParentTeammate().isStationary() && (rvd.getParentRendezvous().getTimeMeeting() <= timeElapsed)) {
            agent.setExploreState(Agent.ExplorationState.GoToParent);
            return agent.stay();
        }
        /*if (agent.getParentTeammate().getRendezvousAgentData().getChildRendezvous().getTimeMeeting() - timeElapsed >= 500) {
            agent.setExploreState(Agent.ExplorationState.GoToParent);
            return agent.getLocation();
        }*/

        if (agent.getStateTimer() <= 1) {
            super.replan(timeElapsed);
        }

        if (relayType == SimulatorConfig.relaytype.Random) {
            if (!agent.comStations.isEmpty() && (Math.random() < simConfig.getComStationDropChance())) {
                state = Agent.ExplorationState.SettingRelay;
            }
            TeammateAgent relay = agent.findNearComStation(agent.getSpeed());
            if (agent.comStations.size() < agent.getComStationLimit() && relay != null && Math.random() < simConfig.getComStationTakeChance()) {
                state = Agent.ExplorationState.TakingRelay;
                return relay.getLocation();
            }
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
            return agent.stay();
        }

        if (agent.getLocation().equals(prvd.getChildRendezvous().getChildLocation())) {
            agent.setExploreState(RealAgent.ExplorationState.WaitForParent);
            return agent.stay();
        }

        if (agent.getStateTimer() <= 1) {
            //Just changed to this state so need to generate path
            if (agent.getParentTeammate().isStationary()) {
                agent.setPath(agent.calculatePath(rvd.getParentRendezvous().getChildLocation()));
            } else {
                agent.setPath(agent.calculatePath(prvd.getChildRendezvous().getChildLocation()));
            }
        }
        return agent.getPath().nextPoint();

    }

    public Point takeStep_GoToRelay() {

        if (agent.getLocation().equals(agent.getCurrentGoal())) {
            //Setting state twice to get right previous state
            agent.setExploreState(agent.getPrevExploreState());
            if (noRelay(agent.getLocation())) {
                agent.setExploreState(RealAgent.ExplorationState.SettingRelay);
            } else {
                agent.setExploreState(RealAgent.ExplorationState.TakingRelay);
            }
            return agent.getLocation();
        }
        return agent.getPath().nextPoint();
    }

    public Point takeStep_GoToChild() {
        //If child is in range with exceptions (just switched child and need to tell old child, and if the parent is in range too, because this can leed to a deadlock)
        if (agent.getChildTeammate().hasCommunicationLink() && !switchedChildAndNeedToTell && !agent.getParentTeammate().hasCommunicationLink()) {
            agent.setExploreState(Agent.ExplorationState.GoToParent);
            return agent.stay();
        }
        //tell child about switch if we just switched child and are in comRange
        if (!agent.getChildTeammate().equals(agent.getOriginalChildTeammate()) && agent.getOriginalChildTeammate().hasCommunicationLink() && switchedChildAndNeedToTell == true) {
            switchedChildAndNeedToTell = false;
            agent.setExploreState(Agent.ExplorationState.GoToParent);
            return agent.stay();
        }

        //Fill this variable here, because we need it in the next section, otherwise this is for relay-handling
        LinkedList<TeammateAgent> relays = new LinkedList<>();
        for (TeammateAgent mate : agent.getAllTeammates().values()) {
            if (mate.isStationary() && mate.getState() == Agent.AgentState.RELAY) {
                relays.add(mate);
            }
        }

        //replace relay of BufferRelay?
        if (relayType == SimulatorConfig.relaytype.BufferRelay) {
            for (TeammateAgent mate : relays) {
                if (mate.getID() != SimConstants.BASE_STATION_TEAMMATE_ID && mate.getLocation().distance(agent.getOriginalChildTeammate().getFrontierCentre()) > mate.getLocation().distance(baseStation.getLocation())) {
                    //relay is nearer to base than to explorer
                    agent.setPath(agent.calculatePath(mate.getLocation()));
                    agent.setExploreState(Agent.ExplorationState.GoToRelay);
                    return agent.stay();
                }
            }
        }

        if (agent.getLocation().equals(rvd.getChildRendezvous().getParentLocation()) && !switchedChildAndNeedToTell && !agent.getParentTeammate().hasCommunicationLink()) {
            agent.setExploreState(RealAgent.ExplorationState.WaitForChild);
            return agent.stay();
        }
        if (agent.getExploreState() != Agent.ExplorationState.WaitForChild && agent.getStateTimer() <= 1) {
            //Just changed to this state so need to generate path
            if (switchedChildAndNeedToTell) {
                agent.setPath(agent.calculatePath(trvd.getChildRendezvous().getParentLocation()));
            } else {
                agent.setPath(agent.calculatePath(rvd.getChildRendezvous().getParentLocation()));
            }
        }

        // <editor-fold defaultstate="collapsed" desc="Relay-Handling">
        tmap.update(false);
        HashMap<Integer, TopologicalNode> topoNodes = agent.getTopologicalMap().getJTopologicalNodes(true);
        LinkedList<TopologicalNode> nodesWithRelay = new LinkedList<>();
        int baseNodeId = agent.getTopologicalMap().getTopologicalJArea(agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID).getLocation());
        nodesWithRelay.add(topoNodes.get(baseNodeId));
        for (TeammateAgent mate : agent.getAllTeammates().values()) {
            if (mate.isStationary() && mate.getState() == Agent.AgentState.RELAY && mate.getID() != SimConstants.BASE_STATION_TEAMMATE_ID) {
                //Is a Relay
                int nodeid = agent.getTopologicalMap().getTopologicalJArea(mate.getLocation());
                nodesWithRelay.add(topoNodes.get(nodeid));

            }

        }

        switch (relayType) {
            case Random:
                if (!agent.comStations.isEmpty() && (Math.random() < simConfig.getComStationDropChance())) {
                    agent.setExploreState(Agent.ExplorationState.SettingRelay);
                    return agent.stay();
                }

                TeammateAgent relay = agent.findNearComStation(agent.getSpeed());
                if (agent.comStations.size() < agent.getComStationLimit() && relay != null && Math.random() < simConfig.getComStationTakeChance()) {
                    agent.setExploreState(Agent.ExplorationState.TakingRelay);
                    return relay.getLocation();
                }
                break;
            case KeyPoints:
                if (!agent.comStations.isEmpty()) {
                    PriorityQueue<NearRVPoint> tempPoints = new PriorityQueue<>();
                    TeammateAgent base = agent.getTeammate(SimConstants.BASE_STATION_TEAMMATE_ID);
                    for (Point p : tmap.getJunctionPoints()) {
                        for (TeammateAgent mate : relays) {
                            if (agent.getOccupancyGrid().directLinePossible(mate.getLocation(), p, false, false)) {
                                tempPoints.add(new NearRVPoint(p.x, p.y, base.getLocation().distance(p)));
                            }
                        }
                    }
                    if (tempPoints.isEmpty()) {
                        for (Point p : tmap.getKeyPoints()) {
                            for (TeammateAgent mate : relays) {
                                if (agent.getOccupancyGrid().directLinePossible(mate.getLocation(), p, false, false)) {
                                    tempPoints.add(new NearRVPoint(p.x, p.y, base.getLocation().distance(p)));
                                }
                            }
                        }
                    }

                    Iterator<NearRVPoint> keyP_iter = tempPoints.iterator();
                    while (keyP_iter.hasNext()) {
                        NearRVPoint keyP = keyP_iter.next();
                        TopologicalNode keyN = topoNodes.get(agent.getTopologicalMap().getTopologicalJArea(keyP));
                        if (noRelay(keyP) && noNearRelay(keyP) && !keyN.isDeadEnd((LinkedList<TopologicalNode>) nodesWithRelay.clone())) {
                            agent.setPath(agent.calculatePath(keyP));
                            agent.setExploreState(Agent.ExplorationState.GoToRelay);
                            return agent.stay();
                        }
                    }
                }
                break;
            case RangeBorder:
                if (!agent.comStations.isEmpty()) {
                    boolean useful = false;
                    for (TeammateAgent mate : relays) {
                        if (mate.getDirectComLink() != 0 && mate.getDirectComLink() < (agent.getSpeed() * 1.5)) {
                            //Is at range-border
                            useful = true;
                        } else if (mate.getDirectComLink() != 0) {
                            //Is in strong comrange so don't drop! Stop iterating as this is a definit "not useful"
                            useful = false;
                            break;
                        }
                    }
                    if (useful) {
                        agent.setExploreState(Agent.ExplorationState.SettingRelay);
                        return agent.stay();
                    }
                }
                break;
            case BufferRelay:
                if (!agent.comStations.isEmpty()) {
                    PriorityQueue<NearRVPoint> tempPoints = new PriorityQueue<>();
                    for (Point p : tmap.getJunctionPoints()) {
                        TopologicalNode keyN = topoNodes.get(agent.getTopologicalMap().getTopologicalJArea(p));
                        for (TeammateAgent mate : relays) {
                            if (!keyN.isDeadEnd(nodesWithRelay)) {
                                tempPoints.add(new NearRVPoint(p.x, p.y, agent.getChildTeammate().getFrontierCentre().distance(p) * -1));
                            }

                        }
                    }
                    Iterator<NearRVPoint> keyP_iter = tempPoints.iterator();
                    while (keyP_iter.hasNext()) {
                        NearRVPoint keyP = keyP_iter.next();
                        TopologicalNode keyN = topoNodes.get(agent.getTopologicalMap().getTopologicalJArea(keyP));
                        if (noRelay(keyP) && noNearRelay(keyP)) {
                            agent.setPath(agent.calculatePath(keyP));
                            agent.setExploreState(Agent.ExplorationState.GoToRelay);
                            return agent.stay();
                        }
                    }
                }
            case None:
            default:
        }

        PriorityQueue<Point> needlessRelays = checkForNeedlessRelays(topoNodes, nodesWithRelay, relays);
        if (!needlessRelays.isEmpty()) {
            agent.setPath(agent.calculatePath(needlessRelays.peek().getLocation()));
            agent.setExploreState(Agent.ExplorationState.GoToRelay);
            return agent.stay();
        }
        // </editor-fold>

        return agent.getPath().nextPoint();
    }

    private PriorityQueue<Point> checkForNeedlessRelays(HashMap<Integer, TopologicalNode> topoNodes, LinkedList<TopologicalNode> nodesWithRelay, LinkedList<TeammateAgent> relays) {
        PriorityQueue<Point> needless = new PriorityQueue<>();

        for (TeammateAgent mate : relays) {
            if (mate.getID() == SimConstants.BASE_STATION_TEAMMATE_ID) {
                continue;
            }
            TopologicalNode node = topoNodes.get(agent.getTopologicalMap().getTopologicalJArea(mate.getLocation()));
            //check for dead ends with all nodesWithRelays as borders except the own node
            LinkedList<TopologicalNode> tempBorder = (LinkedList<TopologicalNode>) nodesWithRelay.clone();
            tempBorder.remove(node);
            boolean deadEnd = node.isDeadEnd((LinkedList<TopologicalNode>) tempBorder.clone());
            if (deadEnd) {
                needless.add(new NearRVPoint(mate.getLocation().x, mate.getLocation().y, agent.getLocation().distance(node.getPosition()) * -1));
            }
        }
        return needless;
    }

}
