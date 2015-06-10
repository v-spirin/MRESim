/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package exploration.rendezvous;

import agents.RealAgent;
import agents.TeammateAgent;
import communication.CommLink;
import communication.PropModel1;
import config.Constants;
import environment.OccupancyGrid;
import exploration.NearRVPoint;
import java.awt.Point;
import java.util.LinkedList;
import java.util.PriorityQueue;
import org.apache.commons.math3.random.SobolSequenceGenerator;
import path.Path;
import static exploration.RoleBasedExploration.timeElapsed;
import java.util.List;

/**
 *
 * @author Victor
 */
public class MultiPointRendezvousStrategy implements IRendezvousStrategy {
    private RealAgent agent;
    private final MultiPointRendezvousStrategyDisplayData displayData;
    private final MultiPointRendezvousStrategySettings settings;
    
    private Point frontierCentre;
    
    private List<NearRVPoint> generatedPoints;
    private List<CommLink> connectionsToBase;
    
    public MultiPointRendezvousStrategy(RealAgent agent, MultiPointRendezvousStrategySettings settings) {
        this.agent = agent;
        displayData = new MultiPointRendezvousStrategyDisplayData();
        this.settings = settings;
    }
    
    public void calculateRendezvousExplorerWithRelay() {
        calculateRendezvousRandomSampling();
    }

    public void calculateRendezvousRelayWithRelay() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }
    
    public void processExplorerCheckDueReturnToRV() {
        //Here we can recalc where we can meet relay (could be, that from our location, it's better to meet relay
        //at another point, i.e. change our part of the RV location pair.
        if (settings.replanOurMeetingPoint) {
            generatedPoints = SampleEnvironmentPoints();
            
            
            calculateRVTimings();
        }
    }

    public void processExplorerStartsHeadingToRV() {
        
    }

    public void processReturnToParentReplan() {
        
    }

    public void processGoToChildReplan() {
        
    }

    public Point processWaitForParent() {
        if (settings.moveToBetterCommsWhileWaiting) {
            //<editor-fold defaultstate="collapsed" desc="Try to move to a cell with better comms">
            Point point1 = agent.getLocation();
            Point point2 = agent.getRendezvousAgentData().getParentRendezvous().getParentLocation();
            return getBetterCommLocation(point1, point2);
        } else
            return agent.getLocation();
    }

    public Point processWaitForChild() {
        if (settings.moveToBetterCommsWhileWaiting) {
            //<editor-fold defaultstate="collapsed" desc="Try to move to a cell with better comms">
            Point point1 = agent.getLocation();
            Point point2 = agent.getRendezvousAgentData().getChildRendezvous().getChildLocation();
            return getBetterCommLocation(point1, point2);
        } else
            return agent.getLocation();
    }
    
    //This method "rolls" the agent into a local minima of better signal between points 1 and 2.
    private Point getBetterCommLocation(Point point1, Point point2) {
        double curSignal = PropModel1.signalStrength(agent.getCommRange(), agent.getOccupancyGrid(), point1, point2);
        double origSignal = curSignal;
        Point curPoint = new Point(point1.x, point1.y);
        boolean foundNewPoint = true;
        while (foundNewPoint && (point1.distance(curPoint) < Constants.DEFAULT_SPEED)) {
            foundNewPoint = false;
            int oldX = curPoint.x; int oldY = curPoint.y;
            for (int x = oldX-1; x <= oldX+1; x++) {
                for (int y = oldY-1; y <= oldY+1; y++) {                        
                    Point testPoint = new Point(x,y);
                    if (agent.getOccupancyGrid().directLinePossible(point1.x, point1.y, testPoint.x, testPoint.y)) {
                        double newSignal = PropModel1.signalStrength(
                                agent.getCommRange(), agent.getOccupancyGrid(), testPoint, point2);
                        if (newSignal > curSignal) {
                            curPoint = testPoint;
                            curSignal = newSignal;
                            foundNewPoint = true;
                        }
                    }
                }
            }
        }
        
        System.out.println(agent + " getBetterCommLocation(" + point1 + ", " + point2 + "): " + 
                "origSignal: " + origSignal + ", newSignal: " + curSignal + ", newPoint: " + curPoint);
        //</editor-fold>
        return curPoint;
    }

    public Point processWaitForChildTimeoutNoBackup() {
        return agent.getLocation();
    }

    public void processJustGotIntoParentRange() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //<editor-fold defaultstate="collapsed" desc="Case 1: Explorer">
        if(agent.isExplorer()) {
            // Second, calculate rendezvous, but stick around for one time step to communicate
            //if(settings.useImprovedRendezvous) {
                calculateRendezvousExplorerWithRelay();                    
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
        }
        //</editor-fold>
            
        //<editor-fold defaultstate="collapsed" desc="Case 2: Relay with another relay as parent">
        else if(agent.getParent() != Constants.BASE_STATION_TEAMMATE_ID) {
            calculateRendezvousRelayWithRelay();
        }
        //</editor-fold>
            
        //<editor-fold defaultstate="collapsed" desc="Case 3: Relay with base station as parent, no need to recalculate rv">
        else {            
            
        }
        //</editor-fold>
    }

    public void processAfterGiveParentInfoExplorer() {
        calculateRVTimings();
    }

    public void processAfterGiveParentInfoRelay() {
        
    }

    public void processAfterGetInfoFromChild() {
        
    }

    public IRendezvousDisplayData getRendezvousDisplayData() {
        return displayData;
    }

    public RealAgent getAgent() {
        return agent;
    }
    
    private void calculateParentTimeToBackupRV()
    {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        if (rvd.getParentBackupRendezvous() == null) return;
        //System.out.println(agent.toString() + "Calculating time to next rendezvous...");
        int timeAtStart = rvd.getParentRendezvous().getTimeMeeting() + rvd.getParentRendezvous().getTimeWait();
        
        Path pathMeToRV2 = agent.calculatePath(rvd.getParentRendezvous().getChildLocation(),
                rvd.getParentBackupRendezvous().getChildLocation());
        
        Path pathParentToRV2 = agent.calculatePath(rvd.getParentRendezvous().getParentLocation(),
                rvd.getParentBackupRendezvous().getParentLocation());
        
        if (pathMeToRV2.found && pathParentToRV2.found)
        {
            System.out.println(agent + "Calculating parent backup RV meeting time...");
            rvd.getParentBackupRendezvous().setTimeMeeting(timeAtStart + 
                    Math.max((int)pathMeToRV2.getLength(), (int)pathParentToRV2.getLength())/Constants.DEFAULT_SPEED);
            /*rvd.getParentBackupRendezvous().setMinTimeMeeting(timeAtStart + 
                    Math.max((int)pathMeToRV2.getLength(), (int)pathParentToRV2.getLength())/Constants.DEFAULT_SPEED);*/
            rvd.getParentBackupRendezvous().setTimeWait(Constants.WAIT_AT_RV_BEFORE_REPLAN);
        } else
        {
            System.out.println(agent + "  !!!FAILED to calculate backup RV times!");
            rvd.getParentBackupRendezvous().setTimeMeeting(Constants.MAX_TIME);
            rvd.getParentBackupRendezvous().setTimeWait(Constants.MAX_TIME);
        }
    }
    
    private void calculateRVTimings() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        TeammateAgent relay = agent.getParentTeammate();
        
        Point parentPoint = rvd.getParentRendezvous().getParentLocation();
        Point childPoint = rvd.getParentRendezvous().getChildLocation();
        
        Point basePoint = rvd.getParentRendezvous().parentsRVLocation.getChildLocation();
        double timeRelayToBase = agent.calculatePath(relay.getLocation(), basePoint).getLength();
        double timeBaseToRV = agent.calculatePath(basePoint, parentPoint).getLength();
        double timeExpToFrontier = agent.calculatePath(agent.getLocation(), frontierCentre).getLength();
        double timeFrontierToRV = agent.calculatePath(frontierCentre, childPoint).getLength();
        
        double timeToMeetingR = timeRelayToBase + timeBaseToRV;
        timeToMeetingR = timeElapsed + timeToMeetingR / Constants.DEFAULT_SPEED;
        double timeToMeetingE = timeExpToFrontier + Constants.FRONTIER_MIN_EXPLORE_TIME + timeFrontierToRV;
        timeToMeetingE = timeElapsed + timeToMeetingE / Constants.DEFAULT_SPEED;
        int timeToMeeting = (int)Math.ceil(Math.max(timeToMeetingR, timeToMeetingE));
        rvd.getParentRendezvous().setTimeMeeting(timeToMeeting);
        //rvd.getParentRendezvous().setMinTimeMeeting(timeToMeeting);
        rvd.getParentRendezvous().setTimeWait(Constants.WAIT_AT_RV_BEFORE_REPLAN);
        
        calculateParentTimeToBackupRV();
        
        System.out.println(agent + " timeToMeetingR: " + timeToMeetingR + ", timeToMeetingE: " + timeToMeetingE + 
                ", timeToMeeting: " + timeToMeeting + ", timeRelayToBase: " + timeRelayToBase + 
                ", timeBaseToRV: " + timeBaseToRV + ", basePoint: " + basePoint + 
                ", relay loc: " + relay.getLocation() + ", parentPoint: " + parentPoint);
    }
    
    
    private LinkedList<NearRVPoint> generateSobolPoints(OccupancyGrid grid) {
        SobolSequenceGenerator sobolGen = new SobolSequenceGenerator(2);
        
        //int numPointsToGenerate = grid.getNumFreeCells() * 150 / 432000;
        int numPointsToGenerate = (int)(grid.getNumFreeCells() / settings.SamplePointDensity); //roughly every 20 sq. cells
        System.out.println("Generating " + numPointsToGenerate + " Sobol points");
        
        LinkedList<NearRVPoint> generatedPoints = new LinkedList<NearRVPoint>();
        
        for (int i = 0; i < numPointsToGenerate; i++) {
            int x = 0;
            int y = 0;
            double[] vector;
            do {
                vector = sobolGen.nextVector();
                x = (int)(vector[0] * grid.width);
                y = (int)(vector[1] * grid.height);
            } while(!grid.freeSpaceAt(x, y));
            
            NearRVPoint pd = new NearRVPoint(x, y);
            
            /*simConfig.getEnv().setPathStart(pd.point);
            simConfig.getEnv().setPathGoal(expLocation);
            simConfig.getEnv().getTopologicalPath(false);
            pd.distance1 = simConfig.getEnv().getPath().getLength();
            
            simConfig.getEnv().setPathStart(pd.point);
            simConfig.getEnv().setPathGoal(relLocation);
            simConfig.getEnv().getTopologicalPath(false);
            pd.distance2 = simConfig.getEnv().getPath().getLength();*/
            
            generatedPoints.add(pd);
            //freeSpace.remove(index);
        }
        
        return generatedPoints;
    }
    
    private List<NearRVPoint> SampleEnvironmentPoints() {
        LinkedList<NearRVPoint> generatedPoints = generateSobolPoints(agent.getOccupancyGrid());
        //add base station to it. Could also add any special points here as well
        NearRVPoint base = new NearRVPoint(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getX(), 
                agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getY());
        generatedPoints.add(base);
        
        return generatedPoints;
    }
    
    //This method finds comm connections between generatedPoints, and returns the subset of points within comm range
    //of base station
    private List<CommLink> FindCommLinks(List<NearRVPoint> generatedPoints) {
        //LinkedList<CommLink> commLinks = new LinkedList<CommLink>();
        LinkedList<CommLink> connectionsToBase = new LinkedList<CommLink>();
        NearRVPoint base = new NearRVPoint(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getX(), 
                agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getY());
        
        for (NearRVPoint p1: generatedPoints) {
            for (NearRVPoint p2: generatedPoints) {
                if (p1.distance(p2) <= PropModel1.getMaxRange(agent.getCommRange())) {
                    //TODO: range should be min of ours and our teammate's
                    if (PropModel1.isConnected(agent.getOccupancyGrid(), agent.getCommRange(), (Point)p1, (Point)p2)) {
                        //check if connection is line of sight?
                        int numWalls = agent.getOccupancyGrid().numObstaclesOnLine(p1.x, p1.y, p2.x, p2.y);
                        
                        CommLink link = new CommLink(p1, p2);
                        link.numObstacles = numWalls;
                        //commLinks.add(link);
                        p1.commLinks.add(link);
                        if (p1.equals(base)) {
                            System.out.println(Constants.INDENT + "Base is " + p1 + ", adding connected point " + p2);                            
                            connectionsToBase.add(link);
                        }
                        //commLinks.add(new CommLink(p2, p1, null, null));
                    }
                }
            }
        }
        
        return connectionsToBase;
    }
    
    private PriorityQueue<NearRVPoint> GetPointsWithinDistOfFrontier(List<NearRVPoint> generatedPoints, double maxDist) {
        PriorityQueue<NearRVPoint> pointsNearFrontier = new PriorityQueue<NearRVPoint>();
        
        if(agent.getLastFrontier() != null)
            frontierCentre = agent.getLastFrontier().getCentre();//getClosestPoint(agent.getLocation(), agent.getOccupancyGrid());
        else
        {
            System.out.println(agent + " !!!! getLastFrontier returned null, setting frontierCentre to " + agent.getLocation());
            frontierCentre = agent.getLocation();
        }
            
        System.out.println(agent + " frontierCentre is " + frontierCentre);
        // create priority queue of all potential rvpoints within given straight line distance
        for(NearRVPoint p: generatedPoints) {
            double dist = p.distance(frontierCentre);
            if(dist > maxDist)
                continue;
            p.setDistanceToFrontier(dist);
            pointsNearFrontier.add(p);
        }
        
        return pointsNearFrontier;
    }
    
    /**
    * This method finds a point among connectionsToBase (that is in comm range of Base Station) that is closest to
    * origPoint. That is, it's an estimate of the shortest distance we need to travel from origPoint to get into
    * comm range of the Base station
    * @param origPoint  
    * @param connectionsToBase
    */
    private int findNearestPointInBaseCommRange(NearRVPoint origPoint, List<CommLink> connectionsToBase) {
        int pathsCalculated = 0;
        // only calculate nearest base point for connectedPoint if we haven't already.
        if (origPoint.distanceToParent == Double.MAX_VALUE) {
            PriorityQueue<NearRVPoint> lineOfSightBasePoints = new PriorityQueue<NearRVPoint>();
            PriorityQueue<NearRVPoint> nonLOSBasePoints = new PriorityQueue<NearRVPoint>();
            for(CommLink baseLink: connectionsToBase) {
                NearRVPoint basePoint = new NearRVPoint(baseLink.getRemotePoint().x, baseLink.getRemotePoint().y);
                double approxPathLen = basePoint.distance(origPoint);
                basePoint.setDistanceToFrontier(approxPathLen);
                if (baseLink.numObstacles == 0)
                    lineOfSightBasePoints.add(basePoint);
                else
                    nonLOSBasePoints.add(basePoint);
            }

            LinkedList<NearRVPoint> pointsConnectedToBase = new LinkedList<NearRVPoint>();

            for (int j = 0; (j < 5) && !lineOfSightBasePoints.isEmpty(); j++) {
                pointsConnectedToBase.add(lineOfSightBasePoints.poll());
            }

            for (int j = 0; (j < 5) && !nonLOSBasePoints.isEmpty(); j++) {
                pointsConnectedToBase.add(nonLOSBasePoints.poll());
            }

            for(NearRVPoint basePoint: pointsConnectedToBase) {
                pathsCalculated++;
                Path pathToBase = agent.calculatePath(origPoint, basePoint);
                double pathLen = Double.MAX_VALUE;
                if (pathToBase.found)
                    pathLen = pathToBase.getLength();
                if (pathLen < origPoint.distanceToParent) {
                    origPoint.distanceToParent = pathLen;
                    origPoint.parentPoint = basePoint;
                }
            }
        }
        return pathsCalculated;
    }
    
    private void calculateRendezvousRandomSampling() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        // Only calculate rv every several time steps at most
        if(rvd.getTimeSinceLastRVCalc() < Constants.RV_REPLAN_INTERVAL)
            return;
        else
            rvd.setTimeSinceLastRVCalc(0);

        long realtimeStart = System.currentTimeMillis();
        
        TeammateAgent relay = agent.getParentTeammate();
        
        System.out.print(Constants.INDENT + "Generating random points ... ");

        generatedPoints = SampleEnvironmentPoints();

        System.out.print(Constants.INDENT + "Finding commlinks ... ");
        
        connectionsToBase = FindCommLinks(generatedPoints);
        
        System.out.println(Constants.INDENT + "Choosing specific RV point ... ");
        
        PriorityQueue<NearRVPoint> pointsNearFrontier = GetPointsWithinDistOfFrontier(generatedPoints, 100);        
        
        System.out.println(agent + " connectionsToBase count is " + connectionsToBase.size());
        
        int pathsCalculated = 0;
        
        //<editor-fold defaultstate="collapsed" desc="Now for top K points, let's calculate p' distances to base, and find the nearest point connected to base">
        PriorityQueue<NearRVPoint> pointsNearFrontierReal = new PriorityQueue<NearRVPoint>();
        for (int k = 0; (k < 50) && !pointsNearFrontier.isEmpty(); k++) {
            NearRVPoint p = pointsNearFrontier.poll();
            double minDistToBase = Double.MAX_VALUE;
            
            for(CommLink link: p.commLinks) {
                NearRVPoint connectedPoint = link.getRemotePoint();
                
                pathsCalculated = findNearestPointInBaseCommRange(connectedPoint, connectionsToBase);
                
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
            Path pathToFrontier = agent.calculatePath(p, frontierCentre);
            double distToFrontier = Double.MAX_VALUE;
            if (pathToFrontier.found)
                distToFrontier = pathToFrontier.getLength();
            pathsCalculated++;
            p.setDistanceToFrontier(distToFrontier);
            
            if (p.commLinkClosestToBase == null || p.commLinkClosestToBase.getRemotePoint() == null) {
                //something went wrong, set RV to our current location and return
                Rendezvous meetingLocation = new Rendezvous(agent.getLocation());
                Point baseLocation = agent.getTeammate(agent.getParentTeammate().getParent()).getLocation();
                meetingLocation.parentsRVLocation = new Rendezvous(baseLocation);
                rvd.setParentRendezvous(meetingLocation);
                calculateRVTimings();
                return;
            }
            
            p.utility = NearRVPoint.getFullRVUtility(p.distanceToFrontier,
                    p.commLinkClosestToBase.getRemotePoint().distanceToParent, p.commLinkClosestToBase.numObstacles);
            System.out.println(agent + " utility is " + p.utility + " for point " + p + " linked to " +
                    p.commLinkClosestToBase.getRemotePoint() + "; distToFrontier: " + p.distanceToFrontier +
                    ", distToParent: " + p.commLinkClosestToBase.getRemotePoint().distanceToParent);
            pointsNearFrontierReal.add(p);
        }
        System.out.println(agent + "complete, took " + (System.currentTimeMillis()) +
                " ms., paths calculated: " + pathsCalculated);
        //</editor-fold>

        //<editor-fold defaultstate="collapsed" desc="Now just need to retrieve the best point">
        NearRVPoint childPoint = pointsNearFrontierReal.peek();
        NearRVPoint parentPoint = childPoint.commLinkClosestToBase.getRemotePoint();
        
        Rendezvous meetingLocation = new Rendezvous(childPoint);
        meetingLocation.setParentLocation(parentPoint);
        
        Rendezvous parentsMeetingLocation = new Rendezvous(parentPoint.parentPoint);
        Point baseLocation = agent.getTeammate(agent.getParentTeammate().getParent()).getLocation();
        System.out.println("    base location: " + baseLocation);
        parentsMeetingLocation.setParentLocation(agent.getTeammate(agent.getParentTeammate().getParent()).getLocation());
        
        meetingLocation.parentsRVLocation = parentsMeetingLocation;
        rvd.setParentRendezvous(meetingLocation);
        //</editor-fold>
        
        Rendezvous backupRV = new Rendezvous(childPoint);
        rvd.setParentBackupRendezvous(backupRV);
        
        calculateRVTimings();
        
        displayData.setGeneratedPoints(generatedPoints);
        displayData.setPointsNearFrontier(pointsNearFrontier);
        
        System.out.print(Constants.INDENT + "Choosing complete, chose " + 
                rvd.getParentRendezvous().getChildLocation().x + "," + 
                rvd.getParentRendezvous().getChildLocation().y + ". ");
        System.out.println(Constants.INDENT + "Complete RV calculation process took " + 
                (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
    
    public void setAgent(RealAgent ag) {
        agent = ag;
    }
}
