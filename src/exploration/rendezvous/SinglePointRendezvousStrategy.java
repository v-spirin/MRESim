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
import config.Constants;
import environment.OccupancyGrid;
import environment.Skeleton;
import static environment.Skeleton.gridToList;
import static environment.Skeleton.neighborTraversal;
import static environment.Skeleton.numNonzeroNeighbors;
import exploration.NearRVPoint;
import static exploration.RoleBasedExploration.timeElapsed;
import java.awt.Point;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import path.Path;

/**
 *
 * @author Victor
 */
public class SinglePointRendezvousStrategy implements IRendezvousStrategy {

    private RealAgent agent;
    private SinglePointRendezvousStrategyDisplayData displayData;

    private final int MAX_DISTANCE_TO_FRONTIER_CENTER = 600;
    private final int POINTS_NEAR_FRONTIER_TO_CONSIDER = 4;

    private int[][] skeletonGrid;
    private Point frontierCentre;

    private SinglePointRendezvousStrategySettings settings;

    public SinglePointRendezvousStrategy(RealAgent agent, SinglePointRendezvousStrategySettings settings) {
        this.agent = agent;
        displayData = new SinglePointRendezvousStrategyDisplayData();
        this.settings = settings;
    }

    private static LinkedList<Point> findRendezvousPoints(int[][] skeleton, OccupancyGrid occGrid) {
        LinkedList<Point> rvPts = new LinkedList<Point>();

        // Pass 1:  find key points (junctions)
        for (int i = 2; i < skeleton.length - 2; i++) {
            for (int j = 2; j < skeleton[0].length - 2; j++) {
                if (numNonzeroNeighbors(skeleton, i, j) >= 3 && neighborTraversal(skeleton, i, j) >= 3) {
                    rvPts.add(new Point(i, j));
                }
            }
        }

        // Pass 2:  fill in gaps
        LinkedList<Point> pts = gridToList(skeleton);
        boolean addToRVlist;
        for (Point p : pts) {
            // First check if it's an endpoint
            if (numNonzeroNeighbors(skeleton, p.x, p.y) < 2) {
                rvPts.add(p);
                continue;
            }

            // Second check if it's far away from all other rv points
            addToRVlist = true;
            for (Point q : rvPts) {
                if (p.distance(q) < 50) {
                    addToRVlist = false;
                    break;
                }
            }
            if (addToRVlist) {
                rvPts.add(p);
            }
        }

        Point p;
        // Pass 3:  prune points too close to another rv point or too close to an obstacle
        for (int i = rvPts.size() - 1; i >= 0; i--) {
            p = rvPts.get(i);
            if (occGrid.obstacleWithinDistance(p.x, p.y, Constants.WALL_DISTANCE)) {
                rvPts.remove(i);
                continue;
            }
            for (int j = rvPts.size() - 1; j >= 0; j--) {
                if (p.distance(rvPts.get(j)) < 20 && i != j) {
                    rvPts.remove(i);
                    break;
                }
            }
        }

        //System.out.println("---  Removed " + counter + "rvPts that were too close");
        return rvPts;
    }

    // Get a subset of occupancy grid free space points to consider
    private List<Point> SampleEnvironmentPoints() {
        skeletonGrid = Skeleton.findSkeleton(agent.getOccupancyGrid());
        LinkedList<Point> allSkeletonPoints = Skeleton.gridToList(skeletonGrid);
        LinkedList<Point> pts = findRendezvousPoints(skeletonGrid, agent.getOccupancyGrid());

        displayData.setSkeleton(allSkeletonPoints);
        displayData.setRVPoints(pts);

        return pts;
    }

    //calculate simple utility by distance
    private double UtilityDist(double dist) {
        if (dist == 0) {
            return Integer.MAX_VALUE;
        }
        return 10000 / dist;
    }

    //calculate utility by skeleton vertex degree (1 for endpoint, 2 for corridor, 3+ for junction)
    private double UtilityDegree(double dist, double degree) {
        return Math.pow(degree, 2) * UtilityDist(dist);
    }

    private PriorityQueue<NearRVPoint> PruneByFrontierProximity(List<Point> pts) {
        LinkedList<Point> rvPtsCopy = new LinkedList();
        pts.stream().forEach((p) -> {
            rvPtsCopy.add(new Point(p.x, p.y));
        });

        PriorityQueue<NearRVPoint> nearRVPoints = new PriorityQueue<NearRVPoint>();
        if (agent.getLastFrontier() != null) {
            frontierCentre = agent.getLastFrontier().getCentre();//getClosestPoint(agent.getLocation(), agent.getOccupancyGrid());
        } else {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + " !!!! getLastFrontier returned null, setting frontierCentre to " + agent.getLocation());
            }
            frontierCentre = agent.getLocation();
        }
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent + " frontierCentre is " + frontierCentre);
        }
        // create priority queue of all potential rvpoints within given straight line distance
        for (Point p : rvPtsCopy) {
            if (p.distance(frontierCentre) > MAX_DISTANCE_TO_FRONTIER_CENTER) {
                continue;
            }

            nearRVPoints.add(new NearRVPoint(p.x, p.y, UtilityDist(p.distance(frontierCentre))));
        }
        return nearRVPoints;
    }

    private PriorityQueue<NearRVPoint> PruneByUtility(PriorityQueue<NearRVPoint> pts) {
        // reduce to up to 4 points closest to next frontier, calculate exact path cost
        NearRVPoint tempPoint;
        double pathCost;
        int degree;
        PriorityQueue<NearRVPoint> prunedNearRvPts = new PriorityQueue();
        for (int i = 0; i < POINTS_NEAR_FRONTIER_TO_CONSIDER; i++) {
            if (pts.size() > 0) {
                tempPoint = pts.remove();
                Path p = agent.calculatePath(tempPoint, frontierCentre);
                if (p.found) {
                    pathCost = p.getLength();
                    degree = Skeleton.neighborTraversal(skeletonGrid, tempPoint.x, tempPoint.y);
                    prunedNearRvPts.add(new NearRVPoint(tempPoint.x, tempPoint.y, UtilityDegree(pathCost, degree)));
                }
            }
        }
        return prunedNearRvPts;
    }

    //Calculate time to next RV with parent, taking parent communication range into account (using simple circle model)
    private void calculateParentTimeToRV() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        Point baseLoc = agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation();
        Point relayLoc = agent.getParentTeammate().getLocation();
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent.toString() + "Calculating time to next rendezvous...");
        }
        Path pathParentToCS = agent.calculatePath(relayLoc, baseLoc);
        Path pathCSToRendezvous = agent.calculatePath(baseLoc, rvd.getParentRendezvous().getParentLocation());
        //<editor-fold defaultstate="collapsed" desc="Couldn't find pathCSToRV - approximate">
        if ((pathCSToRendezvous.getLength() == 0)
                && (!rvd.getParentRendezvous().getParentLocation().equals(
                        agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation()))) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Could not calculate pathCSToRendezvous!!!!");
            }
            //let's at least set it to a rough approximation - better than setting it to 0!
            pathCSToRendezvous = pathParentToCS;
        }
        //</editor-fold>
        double totalPathLength = pathParentToCS.getLength() + pathCSToRendezvous.getLength();
        if (settings.useSimpleCircleCommModelForBaseRange) {
            totalPathLength = totalPathLength - 2 * Math.min(agent.getParentTeammate().getCommRange(),
                    agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getCommRange());
        }
        rvd.setTimeUntilRendezvous(Math.max((int) ((totalPathLength) / Constants.DEFAULT_SPEED), 60));
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("\nP2CS " + pathParentToCS.getLength() + "; "
                    + " CS2R " + pathCSToRendezvous.getLength() + "; "
                    + totalPathLength + "; "
                    + Constants.DEFAULT_SPEED);
        }

        if (settings.giveExplorerMinTimeNearFrontier) {
            //<editor-fold defaultstate="collapsed" desc="Check time for explorer to reach frontier, to make sure he has time to explore before returning">
            Point frontierLoc;
            if (agent.getLastFrontier() != null) {
                frontierLoc = agent.getLastFrontier().getCentre();
            } else {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(agent + " Setting frontierCentre to agent location");
                }
                frontierLoc = agent.getLocation();
            }
            if (frontierLoc != null) {
                Path here2Frontier = agent.calculatePath(agent.getLocation(), frontierLoc);
                Path front2rv = agent.calculatePath(frontierLoc, rvd.getParentRendezvous().getChildLocation());
                int expTime = (int) (here2Frontier.getLength() + front2rv.getLength()) / Constants.DEFAULT_SPEED;
                expTime += Constants.FRONTIER_MIN_EXPLORE_TIME;
                rvd.setTimeUntilRendezvous(Math.max(rvd.getTimeUntilRendezvous(), expTime));
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(agent + " here2Frontier: " + here2Frontier.getLength() + " front2rv " + front2rv.getLength() + " minExplore " + Constants.FRONTIER_MIN_EXPLORE_TIME);
                }
            } else if (Constants.DEBUG_OUTPUT) {
                System.out.println(agent + " frontier is null");
            }
            //</editor-fold>
        }

        rvd.getParentRendezvous().setTimeMeeting(timeElapsed + rvd.getTimeUntilRendezvous());
        rvd.getParentRendezvous().setTimeWait(Constants.WAIT_AT_RV_BEFORE_REPLAN);

        if (Constants.DEBUG_OUTPUT) {
            System.out.println(Constants.INDENT + "Assume that parent will take " + rvd.getTimeUntilRendezvous()
                    + " time steps until rendezvous.");
        }
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

    public Point calculateRVPoint(RealAgent explorer) {
        List<Point> pts = SampleEnvironmentPoints();

        if (pts == null) {
            return explorer.getLocation();
        } else {
            PriorityQueue<NearRVPoint> nearFrontier = PruneByFrontierProximity(pts);
            PriorityQueue<NearRVPoint> utilityPts = PruneByUtility(nearFrontier);

            Point bestPoint;
            if (utilityPts.size() > 0) {
                bestPoint = utilityPts.remove();
            } else {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(agent + " !!!!! error pruning - bestpoint is set to " + agent.getLocation());
                }
                bestPoint = agent.getLocation();
            }

            return bestPoint;
        }
    }

    @Override
    public void calculateRendezvousExplorerWithRelay() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        // Only calculate rv every several time steps at most
        if (rvd.getTimeSinceLastRVCalc() < Constants.RV_REPLAN_INTERVAL) {
            return;
        } else {
            rvd.setTimeSinceLastRVCalc(0);
        }

        long realtimeStart = System.currentTimeMillis();
        if (Constants.DEBUG_OUTPUT) {
            System.out.println(agent.toString() + "Calculating next rendezvous ... ");
        }

        rvd.setParentRendezvous(new Rendezvous(calculateRVPoint(agent)));

        //required by the interface contract - where our relay will meet its parent
        Rendezvous parentsRVLocation = new Rendezvous(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation());
        rvd.getParentRendezvous().parentsRVLocation = parentsRVLocation;

        System.out.print(Constants.INDENT + agent + "Choosing complete, chose "
                + rvd.getParentRendezvous().getChildLocation().x + ","
                + rvd.getParentRendezvous().getChildLocation().y + ". ");

        calculateParentTimeToRV();
        calculateParentTimeToBackupRV();

        if (Constants.DEBUG_OUTPUT) {
            System.out.println("Took " + (System.currentTimeMillis() - realtimeStart) + "ms.");
        }
    }

    //For the case of Relay having an RV with another relay
    @Override
    public void calculateRendezvousRelayWithRelay() {
        throw new UnsupportedOperationException("Not supported yet.");
        /*long realtimeStart = System.currentTimeMillis();
        System.out.println(agent.toString() + "Calculating next rendezvous2 ... ");

        int[][] skeletonGrid = Skeleton.findSkeleton(agent.getOccupancyGrid());
        LinkedList<Point> allSkeletonPoints = Skeleton.gridToList(skeletonGrid);
        LinkedList<Point> pts = Skeleton.findRendezvousPoints(skeletonGrid, agent.getOccupancyGrid());
        
        System.out.println("          Initial functions took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        
        if(pts == null) 
            agent.setParentRendezvous(new Rendezvous(agent.getLocation()));

        else {
            agent.setSkeleton(allSkeletonPoints);
            agent.setRVPoints(pts);
            
            // calculate path from base station to own childrendezvous
            // NOTE this is for branch depth 3, greater depth needs to calculate
            // path from parent's parentrendezvous (must be communicated) to own childrendezvous
            Path pathToChild = agent.calculatePath(agent.getTeammate(Constants.BASE_STATION_TEAMMATE_ID).getLocation(), 
                    agent.getChildRendezvous().getParentLocation());
            Point middle = (Point)pathToChild.getPoints().get((int)(pathToChild.getPoints().size()/2));
            double pathCost;
            double bestPathCost = Double.MAX_VALUE;
            Point bestPoint = agent.getLocation();
            
            for(Point p: pts) {
                //skip points that are too far anyway
                if(p.distance(middle) > 100)
                    continue;
                pathCost = agent.calculatePath(p, middle).getLength();
                if(pathCost < bestPathCost) {
                    bestPathCost = pathCost;
                    bestPoint = p;
                }
            }
            
            agent.setParentRendezvous(new Rendezvous(bestPoint));
        }
        
        System.out.print(" -Chose RV at " + agent.getParentRendezvous().getChildLocation().x + "," + 
                agent.getParentRendezvous().getChildLocation().y + ". ");
        System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");*/
    }

    @Override
    public void processExplorerStartsHeadingToRV() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        if (!settings.useImprovedRendezvous) {
            //if we are not using improved RV, then next RV point is the point at which the Explorer 
            //turns back to RV with the parent
            rvd.setChildRendezvous(new Rendezvous(agent.getLocation()));
        }
    }

    @Override
    public void processExplorerCheckDueReturnToRV() {

    }

    @Override
    public void processReturnToParentReplan() {

    }

    @Override
    public Path processGoToChildReplan() {
        return null;
    }

    @Override
    public Point processWaitForParent() {
        /*if (agent.getParentTeammate().getID() == Constants.BASE_STATION_AGENT_ID) {
            //if we are returning to base station, we should never get into this state, because base station doesn't move.
            //So what must have happened is that we picked a spot that we thought is in the range of the base station, but
            //actually it is not. So we should head directly to the base station.
            agent.getSimConfig().setBaseRange(false);
            return RoleBasedExploration.takeStep_ReturnToParent(agent);
        }*/
        return new Point(agent.getX(), agent.getY());
    }

    @Override
    public void processJustGotIntoParentRange() {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        //<editor-fold defaultstate="collapsed" desc="Case 1: Explorer">
        if (agent.isExplorer()) {
            // Second, calculate rendezvous, but stick around for one time step to communicate
            if (settings.useImprovedRendezvous) {
                calculateRendezvousExplorerWithRelay();
                /*if (rvThroughWalls) {
                    if (rvd.getTimeSinceLastRVCalc() == 0)
                        rvd.setTimeSinceLastRVCalc(100);
                    rvd.setParentBackupRendezvous(rvd.getParentRendezvous());
                    //calculateRVThroughWalls(agent);
                    calculateRendezvousRandomSampling(agent);
                }*/
            } else {
                //calculateRendezvous();
                //set RV to next destination
                //if we have a current goal that is not right next to where we are
                if (agent.getCurrentGoal().distance(agent.getLocation()) > 25) {
                    rvd.setChildRendezvous(new Rendezvous(agent.getCurrentGoal()));
                }
                rvd.setParentRendezvous(rvd.getChildRendezvous());
                /*if (rvThroughWalls && timeElapsed > 100) {
                    if (rvd.getTimeSinceLastRVCalc() == 0)
                        rvd.setTimeSinceLastRVCalc(100);
                    rvd.setParentBackupRendezvous(rvd.getParentRendezvous());
                    calculateRVThroughWalls(agent);
                }*/
            }
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
    public void processAfterGiveParentInfoExplorer() {
        calculateParentTimeToRV();
        calculateParentTimeToBackupRV();
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
    public Point processWaitForChild() {
        return agent.getLocation();
    }

    @Override
    public Point processWaitForChildTimeoutNoBackup() {
        return agent.getLocation();
    }

    @Override
    public IRendezvousDisplayData getRendezvousDisplayData() {
        return displayData;
    }

    @Override
    public RealAgent getAgent() {
        return agent;
    }

    @Override
    public void setAgent(RealAgent ag) {
        agent = ag;
    }
}
