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
import communication.CommLink;
import config.Constants;
import config.SimulatorConfig;
import exploration.rendezvous.MultiPointRendezvousStrategy;
import exploration.rendezvous.NearRVPoint;
import java.awt.Point;
import java.util.List;
import path.Path;

/**
 *
 * @author Victor
 */
public class UtilityExploration extends FrontierExploration {

    private final int TIME_BETWEEN_PLANS = 1;
    private final int TIME_BETWEEN_RECOMPUTE_PATHS = 10;
    SimulatorConfig simConfig;

    public UtilityExploration(RealAgent agent, SimulatorConfig simConfig, RealAgent baseStation) {
        super(agent, SimulatorConfig.frontiertype.ReturnWhenComplete, baseStation);
        this.agent = agent;
        this.simConfig = simConfig;
    }

    // Returns new X, Y of ExploreAgent
    @Override
    public Point takeStep(int timeElapsed) {
        long realtimeStart = System.currentTimeMillis();
        Point nextStep = null;

        // Handle environment error - agent stuck next to a wall
        // if env reports error, agent may be stuck in front of a wall and the
        // simulator isn't allowing him to go through.  Taking a random step might
        // help.
        // Update:  this state is never really reached but leave in just in case
        if (agent.getEnvError()) {
            System.out.println(agent.toString() + "!!!UtilityExploration: Env reports error, taking random step.");
            nextStep = RandomWalk.randomStep(agent);
            agent.setEnvError(false);
            return nextStep;
        }

        // Run correct randomStep function depending on agent state, set nextStep to output
        // Explore is normal exploration, ReturnToParent is relaying information to base
        switch (agent.getState()) {
            case Initial:
                nextStep = takeStep_Initial(timeElapsed);
                break;
            case Explore:
                nextStep = takeStep_Explore(timeElapsed);
                break;
            case ReturnToParent:
                nextStep = takeStep_ReturnToParent(timeElapsed);
                break;
            default:
        }

        // this shouldn't happen, looks like one of takeSteps returned an error
        if (nextStep == null) {
            System.out.println(agent.toString() + "!!!UtilityExploration: nextStep is null, taking random step.");
            nextStep = RandomWalk.randomStep(agent);
        }

        //Increment state timers
        agent.setStateTimer(agent.getStateTimer() + 1);
        //TODO: Should this be in UtilityExploration?
        agent.getRendezvousAgentData().setTimeSinceLastRoleSwitch(agent.getRendezvousAgentData().getTimeSinceLastRoleSwitch() + 1);
        System.out.println(agent.toString() + " takeStep " + agent.getState() + ", took " + (System.currentTimeMillis() - realtimeStart) + "ms.");
        return nextStep;
    }

    private Point takeStep_Initial(int timeElapsed) {
        System.out.println(agent + " takeStep_Initial timeInState: " + agent.getStateTimer());
        // Small number of random steps to get initial range data
        // First 3 steps? Take random step
        if (agent.getStateTimer() < 3) //return RandomWalk.randomStep(agent);
        {
            return agent.getLocation();
        } else {
            // Otherwise? Explorers go into Explore state, others go into GoToChild state.
            // Explorers replan using FrontierExploration, others do nothing.
            agent.setState(RealAgent.AgentState.Explore);
            agent.getStats().setTimeSinceLastPlan(0);
            return takeStep_Explore(timeElapsed);
        }
    }

    private Point takeStep_Explore(int timeElapsed) {
        System.out.println(agent + " takeStep_Explore timeInState: " + agent.getStateTimer());
        Point nextStep;
        // Every CHECK_INTERVAL_TIME_TO_RV steps, check if we're due to change state to return
        int totalNewInfo = agent.getStats().getNewInfo();
        //double infoRatio = (double)agent.getLastContactAreaKnown() /
        //        (double)(agent.getLastContactAreaKnown() + totalNewInfo);
        double infoRatio = (double) agent.getStats().getCurrentBaseKnowledgeBelief()
                / (double) (agent.getStats().getCurrentBaseKnowledgeBelief() + totalNewInfo);

        System.out.println(agent.toString() + " in state Explore. infoRatio = "
                + infoRatio + ", Target = " + simConfig.TARGET_INFO_RATIO + ". newInfo = " + totalNewInfo
                + ", baseInfo = " + agent.getStats().getCurrentBaseKnowledgeBelief());

        if ((!agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInRange()) && (infoRatio < simConfig.TARGET_INFO_RATIO)) //if ((!agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInRange()) && (infoRatio >= simConfig.TARGET_INFO_RATIO))
        {
            System.out.println(agent.toString() + " Decided to return. infoRatio = "
                    + infoRatio + ", Target = " + simConfig.TARGET_INFO_RATIO);
            agent.setState(Agent.AgentState.ReturnToParent);
            //agent.setRole(RobotConfig.roletype.Relay);
            agent.computePathToBaseStation(true);
            agent.setPathToBaseStation();

            if (agent.getPath() == null || agent.getPath().getPoints() == null || agent.getPath().getPoints().size() <= 1) {
                System.out.println(agent.toString() + "Can't find my way home, taking random step.");
                nextStep = RandomWalk.randomStep(agent);
                agent.getStats().setTimeSinceLastPlan(0);
                agent.setCurrentGoal(nextStep);
                return nextStep;
            }

            nextStep = agent.getNextPathPoint();
            if (nextStep.equals(agent.getLocation())) {
                nextStep = agent.getNextPathPoint();
            }
            if (nextStep == null) {
                nextStep = agent.getLocation();
            }

            agent.getStats().setTimeSinceLastPlan(0);
            agent.setCurrentGoal(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation());
            return nextStep;
        }

        //if we reach this point we continue exploring
        //make sure we replan, if we just entered Explore state
        if (agent.getStateTimer() == 0) {
            agent.getStats().setTimeSinceLastPlan(Constants.REPLAN_INTERVAL + 1);
        }
        nextStep = takeStep(timeElapsed);

        // If there are no frontiers to explore, we must be finished.  Return to ComStation.
        if ((frontiers.isEmpty() || (agent.getStats().getPercentageKnown() >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL))) {
            System.out.println(agent + " setting mission complete");
            agent.setMissionComplete(true);
            Point baseLocation = agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation();
            agent.addDirtyCells(agent.getPath().getAllPathPixels());
            agent.setPath(agent.calculatePath(agent.getLocation(), baseLocation, false));
            agent.setState(RealAgent.AgentState.ReturnToParent);
            agent.setStateTimer(0);

            if (agent.getPath().getPoints() != null) {
                agent.setCurrentGoal(baseLocation);
                return ((Point) agent.getPath().getPoints().remove(0));
            } else {
                System.out.println(agent.toString() + "!!!Nothing left to explore, but cannot plan path to parent!!!");
                nextStep = RandomWalk.randomStep(agent);
                agent.setCurrentGoal(nextStep);
                return (nextStep);
            }
        }
        return nextStep;
    }

    private Point takeStep_ReturnToParent(int timeElapsed) {
        System.out.println(agent + " takeStep_ReturnToParent timeInState: " + agent.getStateTimer());
        // If base is in range, go back to exploring
        if (agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).isInRange()) {
            agent.setState(RealAgent.AgentState.Explore);
            agent.setStateTimer(0);
            return takeStep_Explore(timeElapsed);
        }

        // If mission complete and we have tpath, keep going
        if (agent.isMissionComplete() && !agent.getPath().getPoints().isEmpty()) {
            return ((Point) agent.getPath().getPoints().remove(0));
        }

        //If newInfo goes under ratio, go exploring (can happen if we meet a relay)
        int totalNewInfo = agent.getStats().getNewInfo();
        //double infoRatio = (double)agent.getLastContactAreaKnown() /
        //        (double)(agent.getLastContactAreaKnown() + totalNewInfo);
        //double infoRatio = (double)agent.getStats().getCurrentBaseKnowledgeBelief() /
        //        (double)(agent.getStats().getCurrentBaseKnowledgeBelief() + totalNewInfo);

        double infoRatio = (double) totalNewInfo / (double) 57600;

        if ((totalNewInfo == 0) && (infoRatio > simConfig.TARGET_INFO_RATIO)) { //just in case, to avoid returning with 0 new info
            agent.setState(RealAgent.AgentState.Explore);
            agent.setStateTimer(0);
            System.out.println(agent + " switching to takeStep_Explore. infoRatio = "
                    + infoRatio + ", Target = " + simConfig.TARGET_INFO_RATIO);
            return takeStep_Explore(timeElapsed);
        } else {
            System.out.println(agent + " remained in ReturnToParent. infoRatio = "
                    + infoRatio + ", Target = " + simConfig.TARGET_INFO_RATIO);
        }

        Point baseLocation = agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation();
        //Recalculate tpath every PATH_RECALC_PARENT_INTERVAL steps, if fail try A*, if that fails try using existing tpath, if that fails take random step
        Path existingPath = agent.getPath();
        if ((existingPath == null) || (agent.getStateTimer() == 0)
                || (agent.getStateTimer() % Constants.PATH_RECALC_PARENT_INTERVAL) == (Constants.PATH_RECALC_PARENT_INTERVAL - 1)) {
            System.out.println(agent + " replanning path to Base");
            //If tpath already exists, update dirty cells with that tpath
            if (existingPath != null) {
                agent.addDirtyCells(existingPath.getAllPathPixels());
            }

            if (simConfig.getBaseRange()) {
                List<NearRVPoint> generatedPoints
                        = MultiPointRendezvousStrategy.SampleEnvironmentPoints(agent, simConfig.getSamplingDensity());
                NearRVPoint agentPoint = new NearRVPoint(agent.getLocation().x, agent.getLocation().y);
                List<CommLink> connectionsToBase = MultiPointRendezvousStrategy.FindCommLinks(generatedPoints, agent);
                MultiPointRendezvousStrategy.findNearestPointInBaseCommRange(agentPoint, connectionsToBase, agent);
                if (agentPoint.parentPoint != null) {
                    baseLocation = agentPoint.parentPoint.getLocation();
                }
            }

            agent.updateTopologicalMap(false);
            Path tpath = agent.calculatePath(agent.getLocation(), baseLocation, false);
            //If tpath not found, try A*
            if (!tpath.found) {
                System.out.println(agent.toString() + "ERROR!  Could not find full path! Trying pure A*");
                tpath = agent.calculatePath(agent.getLocation(), baseLocation, true);
            }
            //If tpath still not found, try existing tpath. If existing tpath doesn't exist or exhausted, take random step
            if (!tpath.found) {
                System.out.println(agent.toString() + "!!!ERROR!  Could not find full path!");
                if ((existingPath != null) && (existingPath.getPoints().size() > 2)) {
                    agent.setPath(existingPath);
                    agent.setCurrentGoal(existingPath.getGoalPoint());
                } else {
                    agent.setCurrentGoal(agent.getLocation());
                    return RandomWalk.randomStep(agent);
                }
            } else {
                System.out.println(agent + " path to Base found.");
                agent.setPath(tpath);
                agent.setCurrentGoal(baseLocation);
                // Must remove first point in tpath as this is robot's location.
                agent.getPath().getPoints().remove(0);
            }
        }

        if (agent.getPath().found && !agent.getPath().getPoints().isEmpty()) {
            return ((Point) agent.getPath().getPoints().remove(0));
        }

        // If we reach this point, we are not in range of the base and the
        // tpath is empty, so we must have the base.
        // Just check to make sure we are though and if not take random step.
        if (agent.getLocation().distance(baseLocation) > 2 * Constants.STEP_SIZE) {
            System.out.println(agent.toString() + "!!!ERROR! We should have reached parent RV, but we are too far from it! Taking random step");
            agent.setPath(null);
            return RandomWalk.randomStep(agent);
        }
        if (simConfig.getBaseRange()) {
            Point point1 = agent.getLocation();
            Point point2 = agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation();
            Point newPoint = MultiPointRendezvousStrategy.getBetterCommLocation(point1, point2, agent);
            if (!point1.equals(newPoint)) {
                return newPoint;
            } else {
                //still not in range of base. We should really reevaluate our comms model here, but for now, just revert to
                // LOS comms
                simConfig.setBaseRange(false);
                return takeStep_ReturnToParent(timeElapsed);
            }
        } else {

            // If we reach this point, we're at the base station; go back to exploring.
            agent.setState(RealAgent.AgentState.Explore);
            agent.setStateTimer(0);
            return takeStep_Explore(timeElapsed);
        }
    }

    @Override
    public Point replan(int timeElapsed) {
        throw new UnsupportedOperationException("Not supported in this ExplorationType.");
    }

}
