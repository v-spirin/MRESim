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
package exploration.rendezvous;

import agents.Agent;
import agents.RealAgent;
import agents.TeammateAgent;
import communication.CommLink;
import communication.PropModel1;
import config.Constants;
import environment.Frontier;
import environment.OccupancyGrid;
import java.awt.Point;
import java.util.LinkedList;
import java.util.PriorityQueue;
import path.Path;
import java.util.List;
import org.apache.commons.math3.random.SobolSequenceGenerator;

/**
 *
 * @author Victor
 */
public class MultiPointRendezvousStrategy implements IRendezvousStrategy {

    private RealAgent agent;
    private final MultiPointRendezvousStrategyDisplayData displayData;
    private final MultiPointRendezvousStrategySettings settings;

    private List<NearRVPoint> generatedPoints;
    private List<CommLink> connectionsToBase;

    public MultiPointRendezvousStrategy(RealAgent agent, MultiPointRendezvousStrategySettings settings) {
        this.agent = agent;
        displayData = new MultiPointRendezvousStrategyDisplayData();
        this.settings = settings;
    }

    @Override
    public void calculateRendezvousExplorerWithRelay(int timeElapsed) {
        //calculateRendezvousRandomSampling();
        //
        if (settings.attemptExplorationByRelay) {
            calculateRendezvousFrontier(timeElapsed);
        } else {
            calculateRendezvousAAMAS(timeElapsed);
        }
    }

    @Override
    public void calculateRendezvousRelayWithRelay() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void processExplorerCheckDueReturnToRV() {
        //Here we can recalc where we can meet relay (could be, that from our location, it's better to meet relay
        //at another point, i.e. change our part of the RV location pair.
        if (settings.replanOurMeetingPoint) {
            RendezvousAgentData rvd = agent.getRendezvousAgentData();

            int origMeetingTime = rvd.getParentRendezvous().getTimeMeeting();
            int origWaitingTime = rvd.getParentRendezvous().getTimeWait();
            Point relayPoint = rvd.getParentRendezvous().getParentLocation();
            Point explorerPoint = rvd.getParentRendezvous().getChildLocation();
            Rendezvous origParentsRV = rvd.getParentRendezvous().parentsRVLocation;

            //Do same as sampling method, except we already have explorer point
            //need to find nearest point to base's comms range
            System.out.print(Constants.INDENT + "Generating random points ... ");
            generatedPoints = SampleEnvironmentPoints(agent, settings.SamplePointDensity);

            NearRVPoint relayRVPoint = new NearRVPoint(relayPoint.x, relayPoint.y);
            NearRVPoint explorerRVPoint = new NearRVPoint(explorerPoint.x, explorerPoint.y);
            generatedPoints.add(explorerRVPoint);
            generatedPoints.add(relayRVPoint);

            System.out.print(Constants.INDENT + "Finding commlinks ... ");
            connectionsToBase = FindCommLinks(generatedPoints, agent);
            int pathsCalculated = 0;

            double minDistToExplorer = Double.MAX_VALUE;
            NearRVPoint bestRVPoint = explorerRVPoint;
            for (CommLink link : relayRVPoint.commLinks) {
                NearRVPoint connectedPoint = link.getRemotePoint();

                double dist = agent.calculatePath(connectedPoint.getLocation(), agent.getLocation()).getLength();
                if (dist < minDistToExplorer) {
                    bestRVPoint = connectedPoint;
                    minDistToExplorer = dist;
                }
            }
            //End method. Now just set the found points as RV.

            NearRVPoint childPoint = bestRVPoint;
            NearRVPoint parentPoint = relayRVPoint;

            Rendezvous meetingLocation = new Rendezvous(childPoint);
            meetingLocation.setParentLocation(parentPoint);

            /*Rendezvous parentsMeetingLocation = new Rendezvous(parentPoint.parentPoint);
            Point baseLocation = agent.getTeammate(agent.getParentTeammate().getParent()).getLocation();
            System.out.println("    base location: " + baseLocation);
            parentsMeetingLocation.setParentLocation(agent.getTeammate(agent.getParentTeammate().getParent()).getLocation());*/
            meetingLocation.parentsRVLocation = origParentsRV;
            rvd.setParentRendezvous(meetingLocation);

            //Rendezvous backupRV = new Rendezvous(childPoint);
            //rvd.setParentBackupRendezvous(backupRV);
            //calculate timings
            rvd.getParentRendezvous().setTimeMeeting(origMeetingTime);
            rvd.getParentRendezvous().setTimeWait(origWaitingTime);

            //calculateParentTimeToBackupRV();
            displayData.setGeneratedPoints(generatedPoints);

            System.out.print(Constants.INDENT + "Re-evaluating explorer's meeting point complete "
                    + rvd.getParentRendezvous().getChildLocation().x + ","
                    + rvd.getParentRendezvous().getChildLocation().y + ". ");

            if (rvd.getParentRendezvous().parentsRVLocation.getChildLocation() == null) {
                System.err.println("!!! ParentPoint is null!");
            }

        }
    }

    @Override
    public void processExplorerStartsHeadingToRV() {

    }

    @Override
    public void processReturnToParentReplan() {

    }

    @Override
    public Path processGoToChildReplan() {
        if (settings.tryToGetToExplorerRV) {
            RendezvousAgentData rvd = agent.getRendezvousAgentData();
            //if we have enough time, we should actually just go to childLocation - this way the child may travel less
            //as it is more likely it will enter our comms range sooner. This only really makes a difference with multipoint RV
            Path path = agent.calculatePath(agent.getLocation(), rvd.getChildRendezvous().getChildLocation());
            boolean pathWorked = false;
            if (path.found) {
                double timeToChild = path.getLength() / agent.getSpeed() + agent.getTimeElapsed();
                if (timeToChild <= rvd.getChildRendezvous().getTimeMeeting()) {
                    pathWorked = true;
                }
            }
            if (pathWorked) {
                return path;
            }
        }
        return null;
    }

    @Override
    public Point processWaitForParent() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        if (settings.moveToBetterCommsWhileWaiting) {
            //<editor-fold defaultstate="collapsed" desc="Try to move to a cell with better comms">
            Point point1 = agent.getLocation();
            Point point2 = agent.getRendezvousAgentData().getParentRendezvous().getParentLocation();
            Point newPoint = getBetterCommLocation(point1, point2, agent);
            if (agent.getParentTeammate().getID() == Constants.BASE_STATION_AGENT_ID) {
                if (!point1.equals(newPoint)) {
                    return newPoint;
                } else {
                    //let's head back to actual base station
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println("Heading back to actual base station, as we cannot communicate with base station form here.");
                    }
                    Rendezvous meetingLocation = rvd.getParentRendezvous();
                    meetingLocation.setChildLocation(agent.getParentTeammate().getLocation());
                    meetingLocation.setParentLocation(agent.getParentTeammate().getLocation());
                    rvd.setParentRendezvous(meetingLocation);
                    Path newPath = agent.calculatePath(point1, agent.getParentTeammate().getLocation());
                    agent.setPath(newPath);

                    agent.setState(Agent.ExploreState.ReturnToParent);
                    return agent.getLocation();
                }
            } else {
                return newPoint;
            }
        } else {
            return agent.getLocation();
        }
    }

    @Override
    public Point processWaitForChild() {
        if (settings.moveToBetterCommsWhileWaiting) {
            //<editor-fold defaultstate="collapsed" desc="Try to move to a cell with better comms">
            Point point1 = agent.getLocation();
            Point point2 = agent.getRendezvousAgentData().getChildRendezvous().getChildLocation();
            return getBetterCommLocation(point1, point2, agent);
        } else {
            return agent.getLocation();
        }
    }

    //This method "rolls" the agent into a local minima of better signal between points 1 and 2.
    public static Point getBetterCommLocation(Point point1, Point point2, RealAgent ag) {
        double curSignal = PropModel1.signalStrength(ag.getCommRange(), ag.getOccupancyGrid(), point1, point2);
        double origSignal = curSignal;
        Point curPoint = new Point(point1.x, point1.y);
        boolean foundNewPoint = true;
        while (foundNewPoint && (point1.distance(curPoint) < Constants.DEFAULT_SPEED)) {
            foundNewPoint = false;
            int oldX = curPoint.x;
            int oldY = curPoint.y;
            for (int x = oldX - Constants.DEFAULT_SPEED; x <= oldX + Constants.DEFAULT_SPEED; x++) {
                for (int y = oldY - Constants.DEFAULT_SPEED; y <= oldY + Constants.DEFAULT_SPEED; y++) {
                    Point testPoint = new Point(x, y);
                    if (ag.getOccupancyGrid().directLinePossible(point1.x, point1.y, testPoint.x, testPoint.y)) {
                        double newSignal = PropModel1.signalStrength(
                                ag.getCommRange(), ag.getOccupancyGrid(), testPoint, point2);
                        if (newSignal > curSignal) {
                            curPoint = testPoint;
                            curSignal = newSignal;
                            foundNewPoint = true;
                        }
                    }
                }
            }
        }
        //if (!curPoint.equals(point1)) {
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(ag + " getBetterCommLocation(" + point1 + ", " + point2 + "): "
                    + "origSignal: " + origSignal + ", newSignal: " + curSignal + ", newPoint: " + curPoint);
        }
        return curPoint;
        //} else {
        //    return RandomWalk.takeStep(agent);
        //}
    }

    @Override
    public Point processWaitForChildTimeoutNoBackup() {
        return agent.getLocation();
    }

    @Override
    public void processJustGotIntoParentRange(int timeElapsed) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //<editor-fold defaultstate="collapsed" desc="Case 1: Explorer">
        if (agent.isExplorer()) {
            // Second, calculate rendezvous, but stick around for one time step to communicate
            //if(settings.useImprovedRendezvous) {
            calculateRendezvousExplorerWithRelay(timeElapsed);
            /*if (rvThroughWalls) {
                    if (rvd.getTimeSinceLastRVCalc() == 0)
                        rvd.setTimeSinceLastRVCalc(100);
                    rvd.setParentBackupRendezvous(rvd.getParentRendezvous());
                    //calculateRVThroughWalls(agent);
                    calculateRendezvousRandomSampling(agent);
                }*/
 /*}
            else {
                //calculateRendezvous();
                rvd.setParentRendezvous(rvd.getChildRendezvous());
                /*if (rvThroughWalls && timeElapsed > 100) {
                    if (rvd.getTimeSinceLastRVCalc() == 0)
                        rvd.setTimeSinceLastRVCalc(100);
                    rvd.setParentBackupRendezvous(rvd.getParentRendezvous());
                    calculateRVThroughWalls(agent);
                }*/
            //}
        } //</editor-fold>
        //<editor-fold defaultstate="collapsed" desc="Case 2: Relay with another relay as parent">
        else if (agent.getParent() != Constants.BASE_STATION_TEAMMATE_ID) {
            calculateRendezvousRelayWithRelay();
        } //</editor-fold>
        //<editor-fold defaultstate="collapsed" desc="Case 3: Relay with base station as parent, no need to recalculate rv">
        else {

        }
        //</editor-fold>
    }

    @Override
    public void processAfterGiveParentInfoExplorer(int timeElapsed) {
        calculateRVTimings(timeElapsed);
    }

    @Override
    public void processAfterGiveParentInfoRelay() {
        //if exploration by relay enabled
        if (settings.attemptExplorationByRelay) {
            RendezvousAgentData rvd = agent.getRendezvousAgentData();
            Path path = agent.calculatePath(agent.getLocation(), rvd.getChildRendezvous().getParentLocation());
            //Check if we have at least T=15 timesteps to spare.
            int timeMeeting = rvd.getChildRendezvous().getTimeMeeting();
            if ((path.getLength() / Constants.DEFAULT_SPEED) + agent.getTimeElapsed() <= (timeMeeting - 15)) {
                //If we do, calc frontiers and check if we can reach the center of any of them, and get to RV in time
                //if we can, go to that frontier.
                //Adapt explore state / frontier exploration to only go to frontiers that we have time to explore. Then we can simply go to explore state above, and once it's time to go back go back into GoToParent state.
                agent.setState(Agent.ExploreState.Explore);
            }
        }
    }

    @Override
    public void processAfterGetInfoFromChild() {

    }

    @Override
    public IRendezvousDisplayData getRendezvousDisplayData() {
        return displayData;
    }

    @Override
    public RealAgent getAgent() {
        return agent;
    }

    private void calculateParentTimeToBackupRV() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        if (rvd.getParentBackupRendezvous() == null) {
            return;
        }
        //System.out.println(agent.toString() + "Calculating time to next rendezvous...");
        int timeAtStart = rvd.getParentRendezvous().getTimeMeeting() + rvd.getParentRendezvous().getTimeWait();

        Path pathMeToRV2 = agent.calculatePath(rvd.getParentRendezvous().getChildLocation(),
                rvd.getParentBackupRendezvous().getChildLocation());

        Path pathParentToRV2 = agent.calculatePath(rvd.getParentRendezvous().getParentLocation(),
                rvd.getParentBackupRendezvous().getParentLocation());

        if (pathMeToRV2.found && pathParentToRV2.found) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + "Calculating parent backup RV meeting time...");
            }
            rvd.getParentBackupRendezvous().setTimeMeeting(timeAtStart
                    + Math.max((int) pathMeToRV2.getLength(), (int) pathParentToRV2.getLength()) / Constants.DEFAULT_SPEED);
            /*rvd.getParentBackupRendezvous().setMinTimeMeeting(timeAtStart + 
                    Math.max((int)pathMeToRV2.getLength(), (int)pathParentToRV2.getLength())/Constants.DEFAULT_SPEED);*/
            rvd.getParentBackupRendezvous().setTimeWait(Constants.WAIT_AT_RV_BEFORE_REPLAN);
        } else {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + "  !!!FAILED to calculate backup RV times!");
            }
            rvd.getParentBackupRendezvous().setTimeMeeting(Constants.MAX_TIME);
            rvd.getParentBackupRendezvous().setTimeWait(Constants.MAX_TIME);
        }
    }

    private void calculateRVTimings(int timeElapsed) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        Point parentPoint = rvd.getParentRendezvous().getParentLocation();
        Point childPoint = rvd.getParentRendezvous().getChildLocation();
        Point basePoint = rvd.getParentRendezvous().parentsRVLocation.getChildLocation();
        int timeToMeeting = calculateRVTimings(parentPoint, childPoint, basePoint, timeElapsed);
        if (timeToMeeting > 0) {
            rvd.getParentRendezvous().setTimeMeeting(timeToMeeting);
            //rvd.getParentRendezvous().setMinTimeMeeting(timeToMeeting);
            rvd.getParentRendezvous().setTimeWait(Constants.WAIT_AT_RV_BEFORE_REPLAN);
            calculateParentTimeToBackupRV();
        }
    }

    //returns meetingTime
    private int calculateRVTimings(Point parentPoint, Point childPoint, Point basePoint, int timeElapsed) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        TeammateAgent relay = agent.getParentTeammate();
        boolean useSingleMeetingTime = settings.useSingleMeetingTime; //assume that we are using single meeting point to time RV.
        //use this to make sure we do not RV more often that we would have if we were using single point RV.

        Point frontierCentre = getExplorerFrontier();
        //Point parentPoint = rvd.getParentRendezvous().getParentLocation();
        //Point childPoint = rvd.getParentRendezvous().getChildLocation();

        //Point basePoint = rvd.getParentRendezvous().parentsRVLocation.getChildLocation();
        if ((relay == null) || (basePoint == null) || (relay.getLocation() == null)) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("!!! Somehow we don't have a relay?! Not recalcing rv timings...");
            }
            return -1;
        }

        if (useSingleMeetingTime) {
            basePoint = agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation();
            parentPoint = childPoint;
        }

        double timeRelayToBase = agent.calculatePath(relay.getLocation(), basePoint).getLength();
        double timeBaseToRV = agent.calculatePath(basePoint, parentPoint).getLength();
        double timeExpToFrontier = agent.calculatePath(agent.getLocation(), frontierCentre).getLength();
        double timeFrontierToRV = agent.calculatePath(frontierCentre, childPoint).getLength();

        double timeToMeetingR = timeRelayToBase + timeBaseToRV;
        timeToMeetingR = timeElapsed + timeToMeetingR / Constants.DEFAULT_SPEED;
        double timeToMeetingE = timeExpToFrontier + timeFrontierToRV;
        timeToMeetingE = timeElapsed + Constants.FRONTIER_MIN_EXPLORE_TIME + timeToMeetingE / Constants.DEFAULT_SPEED;
        int timeToMeeting = (int) Math.ceil(Math.max(timeToMeetingR, timeToMeetingE));

        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " timeToMeetingR: " + timeToMeetingR + ", timeToMeetingE: " + timeToMeetingE
                    + ", timeToMeeting: " + timeToMeeting + ", timeRelayToBase: " + timeRelayToBase
                    + ", timeBaseToRV: " + timeBaseToRV + ", basePoint: " + basePoint
                    + ", relay loc: " + relay.getLocation() + ", parentPoint: " + parentPoint);
        }

        return timeToMeeting;
    }

    public static LinkedList<NearRVPoint> generateSobolPoints(OccupancyGrid grid, double density) {
        SobolSequenceGenerator sobolGen = new SobolSequenceGenerator(2);

        //int numPointsToGenerate = grid.getNumFreeCells() * 150 / 432000;
        int numPointsToGenerate = (int) (grid.getNumFreeCells() / density); //roughly every 20 sq. cells
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("Generating " + numPointsToGenerate + " Sobol points");
        }

        LinkedList<NearRVPoint> genPoints = new LinkedList<NearRVPoint>();

        for (int i = 0; i < numPointsToGenerate; i++) {
            int x = 0;
            int y = 0;
            double[] vector;
            do {
                vector = sobolGen.nextVector();
                x = (int) (vector[0] * grid.width);
                y = (int) (vector[1] * grid.height);
            } while (!grid.freeSpaceAt(x, y));

            NearRVPoint pd = new NearRVPoint(x, y);

            /*simConfig.getEnv().setPathStart(pd.point);
            simConfig.getEnv().setPathGoal(expLocation);
            simConfig.getEnv().getTopologicalPath(false);
            pd.distance1 = simConfig.getEnv().getPath().getLength();
            
            simConfig.getEnv().setPathStart(pd.point);
            simConfig.getEnv().setPathGoal(relLocation);
            simConfig.getEnv().getTopologicalPath(false);
            pd.distance2 = simConfig.getEnv().getPath().getLength();*/
            genPoints.add(pd);
            //freeSpace.remove(index);
        }

        return genPoints;
    }

    public static List<NearRVPoint> SampleEnvironmentPoints(RealAgent ag, double density) {
        LinkedList<NearRVPoint> genPoints = generateSobolPoints(ag.getOccupancyGrid(), density);
        //add base station to it. Could also add any special points here as well
        NearRVPoint base = new NearRVPoint(ag.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getX(),
                ag.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getY());
        genPoints.add(base);

        return genPoints;
    }

    //This method finds comm connections between generatedPoints, and returns the subset of points within comm range
    //of base station
    public static List<CommLink> FindCommLinks(List<NearRVPoint> generatedPoints, RealAgent ag) {
        //LinkedList<CommLink> commLinks = new LinkedList<CommLink>();
        LinkedList<CommLink> connsToBase = new LinkedList<CommLink>();
        NearRVPoint base = new NearRVPoint(ag.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getX(),
                ag.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getY());

        for (NearRVPoint p1 : generatedPoints) {
            for (NearRVPoint p2 : generatedPoints) {
                if (p1.distance(p2) <= PropModel1.getMaxRange(ag.getCommRange())) {
                    //TODO: range should be min of ours and our teammate's
                    if (PropModel1.isConnected(ag.getOccupancyGrid(), ag.getCommRange(), (Point) p1, (Point) p2)) {
                        //check if connection is line of sight?
                        int numWalls = ag.getOccupancyGrid().numObstaclesOnLine(p1.x, p1.y, p2.x, p2.y);

                        CommLink link = new CommLink(p1, p2);
                        link.numObstacles = numWalls;
                        //commLinks.add(link);
                        p1.commLinks.add(link);
                        if (p1.equals(base)) {
                            if (Constants.DEBUG_OUTPUT) {
                                System.out.println(Constants.INDENT + "Base is " + p1 + ", adding connected point " + p2);
                            }
                            connsToBase.add(link);
                        }
                        //commLinks.add(new CommLink(p2, p1, null, null));
                    }
                }
            }
        }

        return connsToBase;
    }

    private Point getExplorerFrontier() {
        Point frontierCentre = agent.getLocation();
        if (agent.getLastFrontier() != null) {
            frontierCentre = agent.getLastFrontier().getCentre();//getClosestPoint(agent.getLocation(), agent.getOccupancyGrid());
        } else if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " !!!! getLastFrontier returned null, setting frontierCentre to " + agent.getLocation());
        } //frontierCentre = agent.getLocation();

        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontierCentre is " + frontierCentre);
        }
        return frontierCentre;
    }

    private PriorityQueue<NearRVPoint> GetPointsWithinDistOfFrontier(List<NearRVPoint> generatedPoints, double maxDist) {
        PriorityQueue<NearRVPoint> pointsNearFrontier = new PriorityQueue<NearRVPoint>();

        Point frontierCentre = getExplorerFrontier();

        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontierCentre is " + frontierCentre);
        }
        // create priority queue of all potential rvpoints within given straight line distance
        for (NearRVPoint p : generatedPoints) {
            double dist = p.distance(frontierCentre);
            if (dist > maxDist) {
                continue;
            }
            p.setDistanceToFrontier(dist);
            pointsNearFrontier.add(p);
        }

        return pointsNearFrontier;
    }

    /**
     * This method finds a point among connectionsToBase (that is in comm range
     * of Base Station) that is closest to origPoint. That is, it's an estimate
     * of the shortest distance we need to travel from origPoint to get into
     * comm range of the Base station
     *
     * @param origPoint
     * @param connectionsToBase
     * @param ag
     * @return
     */
    public static int findNearestPointInBaseCommRange(NearRVPoint origPoint, List<CommLink> connectionsToBase, RealAgent ag) {
        int pathsCalculated = 0;
        // only calculate nearest base point for connectedPoint if we haven't already.
        if (origPoint.distanceToParent == Double.MAX_VALUE) {
            PriorityQueue<NearRVPoint> lineOfSightBasePoints = new PriorityQueue<NearRVPoint>();
            PriorityQueue<NearRVPoint> nonLOSBasePoints = new PriorityQueue<NearRVPoint>();
            for (CommLink baseLink : connectionsToBase) {
                NearRVPoint basePoint = new NearRVPoint(baseLink.getRemotePoint().x, baseLink.getRemotePoint().y);
                double approxPathLen = basePoint.distance(origPoint);
                basePoint.setDistanceToFrontier(approxPathLen);
                if (baseLink.numObstacles == 0) {
                    lineOfSightBasePoints.add(basePoint);
                } else {
                    nonLOSBasePoints.add(basePoint);
                }
            }

            LinkedList<NearRVPoint> pointsConnectedToBase = new LinkedList<NearRVPoint>();

            for (int j = 0; (j < 5) && !lineOfSightBasePoints.isEmpty(); j++) {
                pointsConnectedToBase.add(lineOfSightBasePoints.poll());
            }

            for (int j = 0; (j < 20) && !nonLOSBasePoints.isEmpty(); j++) {
                pointsConnectedToBase.add(nonLOSBasePoints.poll());
            }

            for (NearRVPoint basePoint : pointsConnectedToBase) {
                pathsCalculated++;
                Path pathToBase = ag.calculatePath(origPoint, basePoint);
                double pathLen = Double.MAX_VALUE;
                if (pathToBase.found) {
                    pathLen = pathToBase.getLength();
                }
                if (pathLen < origPoint.distanceToParent) {
                    origPoint.distanceToParent = pathLen;
                    origPoint.parentPoint = basePoint;
                }
            }
        }
        return pathsCalculated;
    }

    //sets display data to that of the underlying single point rendezvous strategy and returns conventional RV point
    private Point getExplorerRVPoint() {
        SinglePointRendezvousStrategy str = RendezvousStrategyFactory.createSinglePointImprovedRendezvousStrategy(agent);
        Point result = str.calculateRVPoint(agent);
        displayData.setSPRDisplayData((SinglePointRendezvousStrategyDisplayData) str.getRendezvousDisplayData());
        return result;
    }

    //Explorer selects point as in DeHoog's AAMAS2010 paper, relay selects point such that it can explore nearby frontiers
    private void calculateRendezvousFrontier(int timeElapsed) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        // Only calculate rv every several time steps at most
        if (rvd.getTimeSinceLastRVCalc() < Constants.RV_REPLAN_INTERVAL) {
            return;
        } else {
            rvd.setTimeSinceLastRVCalc(0);
        }

        Point explorerPoint = getExplorerRVPoint();
        TeammateAgent relay = agent.getParentTeammate();

        //Do same as sampling method, except we already have explorer point
        //need to find nearest point to base's comms range
        System.out.print(Constants.INDENT + "Generating random points ... ");
        generatedPoints = SampleEnvironmentPoints(agent, settings.SamplePointDensity);

        NearRVPoint explorerRVPoint = new NearRVPoint(explorerPoint.x, explorerPoint.y);
        generatedPoints.add(explorerRVPoint);

        System.out.print(Constants.INDENT + "Finding commlinks ... ");
        connectionsToBase = FindCommLinks(generatedPoints, agent);
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " connectionsToBase count is " + connectionsToBase.size());
        }
        int pathsCalculated = 0;

        int meetingTime
                = calculateRVTimings(explorerPoint, explorerPoint, agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation(), timeElapsed);

        PriorityQueue<Frontier> frontiers = agent.getFrontiers();

        int maxFrontierExploreTime = 0;

        Point currentRelayBasePoint = agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation();

        if ((rvd.getParentRendezvous() != null) && (rvd.getParentRendezvous().parentsRVLocation != null)
                && (rvd.getParentRendezvous().parentsRVLocation.getChildLocation() != null)) {
            currentRelayBasePoint = rvd.getParentRendezvous().parentsRVLocation.getChildLocation();
        }

        if (Constants.DEBUG_OUTPUT) {
            System.out.println("Current relay base point is " + currentRelayBasePoint);
        }

        Frontier bestFrontier = null;
        CommLink bestLink = null;

        for (Frontier f : frontiers) {
            if (!f.equals(agent.getLastFrontier())) { //potential frontier for the relay to explore
                //can relay even get to frontier in time, if the meeting point was at frontier centre?
                double timeToFrontier = 0;
                timeToFrontier += agent.calculatePath(relay.getLocation(), currentRelayBasePoint).getLength();
                timeToFrontier += agent.calculatePath(currentRelayBasePoint, f.getCentre()).getLength();
                double hereToFrontier = timeToFrontier;
                double delta = explorerPoint.distance(f.getCentre()) - agent.getCommRange();
                if (delta < 0) {
                    delta = 0;
                }
                timeToFrontier += delta;
                timeToFrontier = timeToFrontier / agent.getSpeed();
                timeToFrontier += agent.getTimeElapsed();
                if (timeToFrontier > meetingTime) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println("Skipping frontier test1 at " + f.getCentre() + "; timeToFrontier is " + timeToFrontier + ", meetingTime is " + meetingTime);
                    }
                    continue;
                } //cannot possibly reach frontier in time.
                if (meetingTime - timeToFrontier < maxFrontierExploreTime) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println("Skipping frontier test1 at " + f.getCentre() + "; timeToFrontier is " + timeToFrontier + ", meetingTime is " + meetingTime + ", maxFrontierExploreTime is " + maxFrontierExploreTime);
                    }
                    continue;
                } //we already have a frontier we can explore for longer

                double minDistToBase = Double.MAX_VALUE;
                for (CommLink link : explorerRVPoint.commLinks) {
                    NearRVPoint connectedPoint = link.getRemotePoint();

                    Path frontierToMeeting = agent.calculatePath(f.getCentre(), connectedPoint.getLocation());
                    double totalTime = (hereToFrontier + frontierToMeeting.getLength()) / agent.getSpeed();
                    totalTime += agent.getTimeElapsed();

                    if (!frontierToMeeting.found) {
                        if (Constants.DEBUG_OUTPUT) {
                            System.out.println("Skipping frontier test2 at " + f.getCentre() + "; path not found! (between " + f.getCentre() + " and " + connectedPoint.getLocation());
                        }
                        continue;
                    }

                    if (totalTime > meetingTime) {
                        if (Constants.DEBUG_OUTPUT) {
                            System.out.println("Skipping frontier test2 at " + f.getCentre() + "; timeToFrontier is " + timeToFrontier + ", meetingTime is " + meetingTime + ", totalTime is " + totalTime);
                        }
                        continue;
                    } //cannot make it to meeting point in time
                    if (meetingTime - totalTime < maxFrontierExploreTime) {
                        if (Constants.DEBUG_OUTPUT) {
                            System.out.println("Skipping frontier test2 at " + f.getCentre() + "; timeToFrontier is " + timeToFrontier + ", meetingTime is " + meetingTime + ", maxFrontierExploreTime is " + maxFrontierExploreTime);
                        }
                        continue;
                    } //we already have a better point

                    maxFrontierExploreTime = (int) (meetingTime - totalTime);
                    bestFrontier = f;
                    bestLink = link;

                    /*pathsCalculated = findNearestPointInBaseCommRange(connectedPoint, connectionsToBase, agent);

                    if (connectedPoint.distanceToParent < minDistToBase) {
                        minDistToBase = connectedPoint.distanceToParent;
                        explorerRVPoint.commLinkClosestToBase = link;
                    }*/
                }
            }
        }

        NearRVPoint parentPoint;
        NearRVPoint childPoint = explorerRVPoint;
        if (bestFrontier != null && bestLink != null) {
            pathsCalculated = findNearestPointInBaseCommRange(bestLink.getRemotePoint(), connectionsToBase, agent);

            explorerRVPoint.commLinkClosestToBase = bestLink;
            parentPoint = childPoint.commLinkClosestToBase.getRemotePoint();
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Ended up selecting frontier " + bestFrontier.getCentre() + ", parentPoint is " + parentPoint.toString() + ", basePoint is " + parentPoint.parentPoint.toString());
            }
        } else {
            parentPoint = explorerRVPoint;
            pathsCalculated = findNearestPointInBaseCommRange(parentPoint, connectionsToBase, agent);
            CommLink selfLink = new CommLink(parentPoint, parentPoint);
            explorerRVPoint.commLinkClosestToBase = selfLink;
        }

        //At this point, for explorerRVPoint, we know:
        //  1. Connected point explorerRVPoint' that is nearest to comm range of Base
        //  2. Distance from explorerRVPoint' to comm range of Base
        //  3. Nearest point from explorerRVPoint' that is within comm range of Base
        if (explorerRVPoint.commLinkClosestToBase == null || explorerRVPoint.commLinkClosestToBase.getRemotePoint() == null) {
            //something went wrong, set RV to backup and return
            Rendezvous meetingLocation = new Rendezvous(explorerPoint);
            meetingLocation.setParentLocation(explorerPoint);
            Point baseLocation = agent.getTeammate(agent.getParentTeammate().getParent()).getLocation();
            Rendezvous parentsMeetingLocation = new Rendezvous(baseLocation);
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("    base location: " + baseLocation);
            }
            parentsMeetingLocation.setParentLocation(baseLocation);
            meetingLocation.parentsRVLocation = parentsMeetingLocation;
            rvd.setParentRendezvous(meetingLocation);
            Rendezvous backupRV = new Rendezvous(explorerPoint);
            rvd.setParentBackupRendezvous(backupRV);
            calculateRVTimings(timeElapsed);
            return;
        }
        //End method. Now just set the found points as RV.

        Rendezvous meetingLocation = new Rendezvous(childPoint);
        meetingLocation.setParentLocation(parentPoint);

        Rendezvous parentsMeetingLocation = new Rendezvous(parentPoint.parentPoint);
        Point baseLocation = agent.getTeammate(agent.getParentTeammate().getParent()).getLocation();
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("    base location: " + baseLocation);
        }
        parentsMeetingLocation.setParentLocation(agent.getTeammate(agent.getParentTeammate().getParent()).getLocation());

        meetingLocation.parentsRVLocation = parentsMeetingLocation;
        rvd.setParentRendezvous(meetingLocation);

        Rendezvous backupRV = new Rendezvous(childPoint);
        rvd.setParentBackupRendezvous(backupRV);

        calculateRVTimings(timeElapsed);

        displayData.setGeneratedPoints(generatedPoints);

        System.out.print(Constants.INDENT + "Choosing advanced RV complete, chose "
                + rvd.getParentRendezvous().getChildLocation().x + ","
                + rvd.getParentRendezvous().getChildLocation().y + ". ");
    }

    //Explorer selects point as in DeHoog's AAMAS2010 paper, relay selects point nearest to base in range of explorer's point
    private void calculateRendezvousAAMAS(int timeElapsed) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        // Only calculate rv every several time steps at most
        if (rvd.getTimeSinceLastRVCalc() < Constants.RV_REPLAN_INTERVAL) {
            return;
        } else {
            rvd.setTimeSinceLastRVCalc(0);
        }

        Point explorerPoint = getExplorerRVPoint();

        //Do same as sampling method, except we already have explorer point
        //need to find nearest point to base's comms range
        System.out.print(Constants.INDENT + "Generating random points ... ");
        generatedPoints = SampleEnvironmentPoints(agent, settings.SamplePointDensity);

        NearRVPoint explorerRVPoint = new NearRVPoint(explorerPoint.x, explorerPoint.y);
        generatedPoints.add(explorerRVPoint);

        System.out.print(Constants.INDENT + "Finding commlinks ... ");
        connectionsToBase = FindCommLinks(generatedPoints, agent);
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " connectionsToBase count is " + connectionsToBase.size());
        }
        int pathsCalculated = 0;

        double minDistToBase = Double.MAX_VALUE;
        for (CommLink link : explorerRVPoint.commLinks) {
            NearRVPoint connectedPoint = link.getRemotePoint();

            pathsCalculated = findNearestPointInBaseCommRange(connectedPoint, connectionsToBase, agent);

            if (connectedPoint.distanceToParent < minDistToBase) {
                minDistToBase = connectedPoint.distanceToParent;
                explorerRVPoint.commLinkClosestToBase = link;
            }
        }
        //At this point, for explorerRVPoint, we know:
        //  1. Connected point explorerRVPoint' that is nearest to comm range of Base
        //  2. Distance from explorerRVPoint' to comm range of Base
        //  3. Nearest point from explorerRVPoint' that is within comm range of Base
        if (explorerRVPoint.commLinkClosestToBase == null || explorerRVPoint.commLinkClosestToBase.getRemotePoint() == null) {
            //something went wrong, set RV to backup and return
            Rendezvous meetingLocation = new Rendezvous(explorerPoint);
            meetingLocation.setParentLocation(explorerPoint);
            Point baseLocation = agent.getTeammate(agent.getParentTeammate().getParent()).getLocation();
            Rendezvous parentsMeetingLocation = new Rendezvous(baseLocation);
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("    base location: " + baseLocation);
            }
            parentsMeetingLocation.setParentLocation(baseLocation);
            meetingLocation.parentsRVLocation = parentsMeetingLocation;
            rvd.setParentRendezvous(meetingLocation);
            Rendezvous backupRV = new Rendezvous(explorerPoint);
            rvd.setParentBackupRendezvous(backupRV);
            calculateRVTimings(timeElapsed);
            return;
        }
        //End method. Now just set the found points as RV.

        NearRVPoint childPoint = explorerRVPoint;
        NearRVPoint parentPoint = childPoint.commLinkClosestToBase.getRemotePoint();

        Rendezvous meetingLocation = new Rendezvous(childPoint);
        meetingLocation.setParentLocation(parentPoint);

        Rendezvous parentsMeetingLocation = new Rendezvous(parentPoint.parentPoint);
        Point baseLocation = agent.getTeammate(agent.getParentTeammate().getParent()).getLocation();
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("    base location: " + baseLocation);
        }
        parentsMeetingLocation.setParentLocation(agent.getTeammate(agent.getParentTeammate().getParent()).getLocation());

        meetingLocation.parentsRVLocation = parentsMeetingLocation;
        rvd.setParentRendezvous(meetingLocation);

        Rendezvous backupRV = new Rendezvous(childPoint);
        rvd.setParentBackupRendezvous(backupRV);

        calculateRVTimings(timeElapsed);

        displayData.setGeneratedPoints(generatedPoints);

        System.out.print(Constants.INDENT + "Choosing AAMAS complete, chose "
                + rvd.getParentRendezvous().getChildLocation().x + ","
                + rvd.getParentRendezvous().getChildLocation().y + ". ");
    }

    private void calculateRendezvousRandomSampling(int timeElapsed) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        // Only calculate rv every several time steps at most
        if (rvd.getTimeSinceLastRVCalc() < Constants.RV_REPLAN_INTERVAL) {
            return;
        } else {
            rvd.setTimeSinceLastRVCalc(0);
        }

        long realtimeStart = System.currentTimeMillis();

        TeammateAgent relay = agent.getParentTeammate();

        System.out.print(Constants.INDENT + "Generating random points ... ");

        generatedPoints = SampleEnvironmentPoints(agent, settings.SamplePointDensity);

        System.out.print(Constants.INDENT + "Finding commlinks ... ");

        connectionsToBase = FindCommLinks(generatedPoints, agent);

        if (Constants.DEBUG_OUTPUT) {
            System.out.println(Constants.INDENT + "Choosing specific RV point ... ");
        }

        PriorityQueue<NearRVPoint> pointsNearFrontier = GetPointsWithinDistOfFrontier(generatedPoints, 100);

        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " connectionsToBase count is " + connectionsToBase.size());
        }

        int pathsCalculated = 0;

        //<editor-fold defaultstate="collapsed" desc="Now for top K points, let's calculate p' distances to base, and find the nearest point connected to base">
        PriorityQueue<NearRVPoint> pointsNearFrontierReal = new PriorityQueue<NearRVPoint>();
        for (int k = 0; (k < 50) && !pointsNearFrontier.isEmpty(); k++) {
            NearRVPoint p = pointsNearFrontier.poll();
            double minDistToBase = Double.MAX_VALUE;

            for (CommLink link : p.commLinks) {
                NearRVPoint connectedPoint = link.getRemotePoint();

                pathsCalculated = findNearestPointInBaseCommRange(connectedPoint, connectionsToBase, agent);

                if (connectedPoint.distanceToParent < minDistToBase) {
                    minDistToBase = connectedPoint.distanceToParent;
                    p.commLinkClosestToBase = link;
                }
            }
            //At this point, for p, we know:
            //  1. Connected point p' that is nearest to comm range of Base
            //  2. Distance from p' to comm range of Base
            //  3. Nearest point from p' that is within comm range of Base
            //So we know how long each point p will have to wait for relay, and so can estimate
            //where explorer will be at the time, to calculate regret accurately.

            //For now, just calculate accurate distance to next frontier:
            Path pathToFrontier = agent.calculatePath(p, getExplorerFrontier());
            double distToFrontier = Double.MAX_VALUE;
            if (pathToFrontier.found) {
                distToFrontier = pathToFrontier.getLength();
            }
            pathsCalculated++;
            p.setDistanceToFrontier(distToFrontier);

            if (p.commLinkClosestToBase == null || p.commLinkClosestToBase.getRemotePoint() == null) {
                //something went wrong, set RV to our current location and return
                Rendezvous meetingLocation = new Rendezvous(agent.getLocation());
                Point baseLocation = agent.getTeammate(agent.getParentTeammate().getParent()).getLocation();
                meetingLocation.parentsRVLocation = new Rendezvous(baseLocation);
                rvd.setParentRendezvous(meetingLocation);
                calculateRVTimings(timeElapsed);
                return;
            }

            p.utility = NearRVPoint.getFullRVUtility(p.distanceToFrontier,
                    p.commLinkClosestToBase.getRemotePoint().distanceToParent, p.commLinkClosestToBase.numObstacles);
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + " utility is " + p.utility + " for point " + p + " linked to "
                        + p.commLinkClosestToBase.getRemotePoint() + "; distToFrontier: " + p.distanceToFrontier
                        + ", distToParent: " + p.commLinkClosestToBase.getRemotePoint().distanceToParent);
            }
            pointsNearFrontierReal.add(p);
        }
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + "complete, took " + (System.currentTimeMillis())
                    + " ms., paths calculated: " + pathsCalculated);
        }
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="Now just need to retrieve the best point">
        NearRVPoint childPoint = pointsNearFrontierReal.peek();
        NearRVPoint parentPoint = childPoint.commLinkClosestToBase.getRemotePoint();

        Rendezvous meetingLocation = new Rendezvous(childPoint);
        meetingLocation.setParentLocation(parentPoint);

        Rendezvous parentsMeetingLocation = new Rendezvous(parentPoint.parentPoint);
        Point baseLocation = agent.getTeammate(agent.getParentTeammate().getParent()).getLocation();
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("    base location: " + baseLocation);
        }
        parentsMeetingLocation.setParentLocation(agent.getTeammate(agent.getParentTeammate().getParent()).getLocation());

        meetingLocation.parentsRVLocation = parentsMeetingLocation;
        rvd.setParentRendezvous(meetingLocation);
        //</editor-fold>

        Rendezvous backupRV = new Rendezvous(childPoint);
        rvd.setParentBackupRendezvous(backupRV);

        calculateRVTimings(timeElapsed);

        displayData.setGeneratedPoints(generatedPoints);
        displayData.setPointsNearFrontier(pointsNearFrontier);

        if (Constants.DEBUG_OUTPUT) {
            System.out.print(Constants.INDENT + "Choosing complete, chose "
                    + rvd.getParentRendezvous().getChildLocation().x + ","
                    + rvd.getParentRendezvous().getChildLocation().y + ". ");
        }
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(Constants.INDENT + "Complete RV calculation process took "
                    + (System.currentTimeMillis() - realtimeStart) + "ms.");
        }
    }

    @Override
    public void setAgent(RealAgent ag) {
        agent = ag;
    }
}
