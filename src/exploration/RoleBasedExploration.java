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
import config.Constants;
import config.RobotConfig;
import config.SimulatorConfig;
import exploration.rendezvous.IRendezvousStrategy;
import exploration.rendezvous.RendezvousAgentData;
import java.awt.Point;
import java.util.concurrent.atomic.AtomicReference;
import path.Path;

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
            case ReturnToBaseStation:
                nextStep = takeStep_ReturnToBaseStation();
                break;
            case WaitForParent:
                nextStep = takeStep_WaitForParent();
                break;
            case GiveParentInfo:
                nextStep = takeStep_GiveParentInfo();
                break;
            case GoToChild:
                nextStep = takeStep_GoToChild();
                break;
            case WaitForChild:
                nextStep = takeStep_WaitForChild();
                break;
            case GetInfoFromChild:
                nextStep = takeStep_GetInfoFromChild();
                break;
            default:
                break;
        }

        // this shouldn't happen, looks like one of takeSteps returned an error
        if (nextStep == null) {
            nextStep = RandomWalk.randomStep(agent);
        }

        agent.setStateTimer(agent.getStateTimer() + 1);
        agent.getRendezvousAgentData().setTimeSinceLastRoleSwitch(agent.getRendezvousAgentData().getTimeSinceLastRoleSwitch() + 1);
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
        RendezvousAgentData parentRvd = agent.getParentTeammate().getRendezvousAgentData();
        boolean haveNewRVDetailsForParent = !rvd.getParentRendezvous().equals(parentRvd.getChildRendezvous());
        boolean noRVAgreed = (rvd.getParentRendezvous().getTimeMeeting() == Constants.MAX_TIME);
        // If parent is in range, and we have been exploring for longer than MIN_TIME_IN_EXPLORE_STATE, give parent info.
        if (agent.getParentTeammate().hasCommunicationLink() && (agent.getRole() != RobotConfig.roletype.Relay)) {
            if ((agent.getStateTimer() > Constants.MIN_TIME_IN_EXPLORE_STATE)
                    || haveNewRVDetailsForParent || noRVAgreed) {// we have a new RV point, and must communicate it to the parent - otherwise the parent will be waiting at the old RV forever!
                agent.setExploreState(RealAgent.ExplorationState.GiveParentInfo);
                return takeStep_GiveParentInfo();
            }
        }
        //else
        //    agent.setTimeUntilRendezvous(agent.getTimeUntilRendezvous() - 1);
        //if we are a relay, check if we are due to return to child and return if it's time

        // Every CHECK_INTERVAL_TIME_TO_RV steps, check if we're due to meet our parent again (unless parent is basestation, in which case explore continuously)
        if (agent.getParent() != Constants.BASE_STATION_TEAMMATE_ID
                && (agent.getStateTimer() % Constants.CHECK_INTERVAL_TIME_TO_RV) == (Constants.CHECK_INTERVAL_TIME_TO_RV - 1)) {

            Path pathToParentRendezvous; //output parameter
            AtomicReference<Path> outPathRef = new AtomicReference<Path>();
            if (isDueToReturnToRV(outPathRef)) {
                //run below method here as it may be costly and we only want to run it when we are about to return to RV anyway
                //so we want to check if perhaps there is a better RV we could go to that is closer, so we can explore some more
                rendezvousStrategy.processExplorerCheckDueReturnToRV();
                if (isDueToReturnToRV(outPathRef)) {
                    pathToParentRendezvous = outPathRef.get();
                    agent.setExploreState(RealAgent.ExplorationState.ReturnToBaseStation);

                    rendezvousStrategy.processExplorerStartsHeadingToRV();

                    if (agent.getExploreState() == Agent.ExplorationState.ReturnToBaseStation) {
                        if ((pathToParentRendezvous == null) || (!pathToParentRendezvous.found)
                                || (pathToParentRendezvous.getPoints().isEmpty())) {
                            //take random step and try again
                            agent.setExploreState(Agent.ExplorationState.Explore);
                            agent.setStateTimer(Constants.CHECK_INTERVAL_TIME_TO_RV - 2);
                            return RandomWalk.randomStep(agent);
                        } else {
                            agent.setPath(pathToParentRendezvous);
                            return agent.getNextPathPoint();
                        }
                    }
                }
            }
            // relay must be waiting for us, we are near rv, have new info - try to comm with relay

        }

        //if we reach this point we continue exploring
        Point nextStep = super.takeStep(timeElapsed);

        //If there are no frontiers to explore, we must be finished.  Return to ComStation.
        if ((frontiers.isEmpty() || (agent.getStats().getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))) {
            agent.setPath(agent.calculatePath(agent.getLocation(), agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation(), false));
            agent.setExploreState(RealAgent.ExplorationState.ReturnToBaseStation);
            agent.setMissionComplete(true);

            return agent.getNextPathPoint();
        }

        return nextStep;
    }

    public Point takeStep_ReturnToBaseStation() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //If parent is in range, GiveParentInfo
        if (agent.getParentTeammate().hasCommunicationLink()) {
            agent.setExploreState(RealAgent.ExplorationState.GiveParentInfo);
            return takeStep_GiveParentInfo();
        }

        //If mission complete and we have tpath, keep going
        if (agent.isMissionComplete() && !agent.getPath().getPoints().isEmpty()) {
            return agent.getNextPathPoint();
        }

        //Recalculate tpath every PATH_RECALC_PARENT_INTERVAL steps, if fail try A*, if that fails try using existing tpath, if that fails take random step
        if ((agent.getStateTimer() % Constants.PATH_RECALC_PARENT_INTERVAL) == (Constants.PATH_RECALC_PARENT_INTERVAL - 1)) {
            rendezvousStrategy.processReturnToParentReplan();

            Path tpath = agent.calculatePath(agent.getLocation(), rvd.getParentRendezvous().getChildLocation(), false);
            //If tpath still not found, try existing tpath. If existing tpath doesn't exist or exhausted, take random step
            agent.setPath(tpath);
        }

        if (agent.getPath().found && !agent.getPath().getPoints().isEmpty()) {
            return agent.getNextPathPoint();
        }

        // If we reach this point, we are not in range of the parent and the
        // tpath is empty, so we must have reached rendezvous point.
        // Just check to make sure we are though and if not take random step.
        if (agent.getLocation().distance(rvd.getParentRendezvous().getChildLocation()) > 2 * Constants.STEP_SIZE) {
            return RandomWalk.randomStep(agent);
        }

        // If we reach this point, we're at the rendezvous point and waiting.
        agent.setExploreState(RealAgent.ExplorationState.WaitForParent);
        return agent.getLocation();
    }

    public Point takeStep_WaitForParent() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //If parent is in range, GiveParentInfo
        if (agent.getParentTeammate().hasCommunicationLink()) {
            agent.setExploreState(RealAgent.ExplorationState.GiveParentInfo);
            return takeStep_GiveParentInfo();
        }

        boolean canStillWait = (timeElapsed <= (rvd.getParentRendezvous().getTimeMeeting() + rvd.getParentRendezvous().getTimeWait()));

        if (canStillWait) {
            return rendezvousStrategy.processWaitForParent();
        } else {
            //Go to backup RV if available. We wil go into Explore state which will head to backup RV to arrive there
            //at the pre-arranged time.
            if (rvd.getParentBackupRendezvous() != null) {
                rvd.setParentRendezvous(rvd.getParentBackupRendezvous());
                rvd.setTimeUntilRendezvous(rvd.getParentRendezvous().getTimeMeeting() - timeElapsed);
                agent.getRendezvousAgentData().setParentBackupRendezvous(null);
            }
            agent.setExploreState(RealAgent.ExplorationState.Explore);
            return takeStep_Explore();
        }
    }

    public Point takeStep_GiveParentInfo() {
        // We've exchanged info, what's next
        if (agent.isMissionComplete()) {
            Path tpath = agent.calculatePath(agent.getLocation(), agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation(), false);
            agent.setPath(tpath);
            agent.setExploreState(RealAgent.ExplorationState.ReturnToBaseStation);
            return agent.getNextPathPoint();

        }

        //If we just got into range, recalc next RV, exchange info
        if (agent.getStateTimer() == 0) {
            //Case 1: Explorer
            if (agent.isExplorer()) {
                // First, plan next frontier, as we need this for rendezvous point calculation
                replan(timeElapsed);
            }

            rendezvousStrategy.processJustGotIntoParentRange(timeElapsed);

            return agent.getLocation();
        } else if (agent.isExplorer()) {
            //else, we've recalculated rv, time to move on
            //Explorer - process & go into Explore state
            rendezvousStrategy.processAfterGiveParentInfoExplorer(timeElapsed);

            agent.setExploreState(RealAgent.ExplorationState.Explore);

            return takeStep_Explore();
        } else {
            //Relay - process & go to child

            rendezvousStrategy.processAfterGiveParentInfoRelay();
            agent.setExploreState(RealAgent.ExplorationState.GoToChild);
            Path tpath = agent.calculatePath(agent.getLocation(), agent.getRendezvousAgentData().getChildRendezvous().getParentLocation(), false);
            agent.setPath(tpath);
            // stay in comm range with base, till it's time to go meet child
            return agent.getNextPathPoint();
        }
    }

    public Point takeStep_GoToChild() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //Check if we are in range of child. If yes, GetInfoFromChild
        if ((agent.getChildTeammate().hasCommunicationLink()) /*&& !agent.getParentTeammate().isInRange()*/) {
            agent.setExploreState(RealAgent.ExplorationState.GetInfoFromChild);

            return takeStep_GetInfoFromChild();
        }

        //Assume that a tpath back to child has been calculated in previous state, recalculate every PATH_RECALC_CHILD_INTERVAL steps
        if (((agent.getStateTimer() % Constants.PATH_RECALC_CHILD_INTERVAL) == (Constants.PATH_RECALC_CHILD_INTERVAL - 1))) {

            Path tpath = rendezvousStrategy.processGoToChildReplan();

            if (tpath == null) {
                tpath = agent.calculatePath(agent.getLocation(), rvd.getChildRendezvous().getParentLocation(), false);
            }
            agent.setPath(tpath);
        }

        if (agent.getPath().found && !agent.getPath().getPoints().isEmpty()) {
            return agent.getNextPathPoint();
        }

        // If we reach this point, we're at the rendezvous point and waiting.
        agent.setExploreState(RealAgent.ExplorationState.WaitForChild);

        return agent.getLocation();
    }

    public Point takeStep_WaitForChild() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        if (agent.getChildTeammate().hasCommunicationLink()) {
            agent.setExploreState(RealAgent.ExplorationState.GetInfoFromChild);
            return takeStep_GetInfoFromChild();
        }

        boolean canStillWait = (timeElapsed <= (rvd.getChildRendezvous().getTimeMeeting() + rvd.getChildRendezvous().getTimeWait()));

        if (canStillWait) {
            return rendezvousStrategy.processWaitForChild();
        } else //Go to backup RV if available. Otherwise do what the strategy requires us to do, e.g. become an explorer.
         if (rvd.getChildBackupRendezvous() != null) {
                rvd.setChildRendezvous(rvd.getChildBackupRendezvous());
                rvd.setChildBackupRendezvous(null);
                agent.setExploreState(RealAgent.ExplorationState.GoToChild);
                return takeStep_GoToChild();
            } else {
                return rendezvousStrategy.processWaitForChildTimeoutNoBackup();
            }
    }

    public Point takeStep_GetInfoFromChild() {
        //we've exchanged info, now return to parent (but wait for one timestep to learn new RV point)

        if (agent.getStateTimer() == 0) {
            Path tpath = agent.calculatePath(agent.getLocation(), agent.getRendezvousAgentData().getParentRendezvous().getChildLocation(), false);
            agent.setPath(tpath);
            return agent.getLocation();
        } else {
            if (agent.getChildTeammate().getExploreState() == Agent.ExplorationState.GiveParentInfo) {
                agent.setTimeSinceGetChildInfo(0);
            }
            agent.setExploreState(RealAgent.ExplorationState.ReturnToBaseStation);
            return takeStep_ReturnToBaseStation();
        }
    }

    private boolean isDueToReturnToRV(AtomicReference<Path> outPathToParentRendezvous) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        Path pathToParentRendezvous = agent.calculatePath(agent.getLocation(), rvd.getParentRendezvous().getChildLocation(), false);

        // If we are due to meet parent again, return to last agreed rendezvous point
        if (pathToParentRendezvous.found) {
            outPathToParentRendezvous.set(pathToParentRendezvous);
            if (((pathToParentRendezvous.getLength() / Constants.DEFAULT_SPEED) + timeElapsed)
                    >= agent.getRendezvousAgentData().getParentRendezvous().getTimeMeeting()) {
                return true;
            }
        }

        return false;
    }
}
