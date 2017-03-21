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

package agents;

import communication.CommLink;
import communication.DataMessage;
import config.Constants;
import config.RobotConfig;
import config.RobotConfig.roletype;
import config.SimulatorConfig;
import environment.Frontier;
import environment.OccupancyGrid;
import environment.TopologicalMap;
import exploration.FrontierExploration;
import exploration.LeaderFollower;
import exploration.NearRVPoint;
import exploration.RelayFrontierExploration;
import exploration.RoleBasedExploration;
import exploration.RunFromLog;
import exploration.SimulationFramework;
import exploration.UtilityExploration;
import exploration.rendezvous.IRendezvousStrategy;
import exploration.rendezvous.MultiPointRendezvousStrategy;
import exploration.rendezvous.RendezvousAgentData;
import exploration.rendezvous.RendezvousStrategyFactory;
import java.awt.Point;
import java.awt.Polygon;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import path.Path;

/**
 *
 * @author Julian de Hoog
 */
public class RealAgent extends Agent {
    
    AgentStats stats;
    private boolean missionComplete; // true only when mission complete (env fully explored)

    int prevX, prevY;             // Previous position in environment
    public int periodicReturnInterval;   // how long to wait until going back to BS
    // bit of a hack: state of periodic return frontier exp robots (0=exploring; 1=returning)
    public int frontierPeriodicState;   
    int timeElapsed;
    boolean envError;             // set to true by env when RealAgent's step is not legal
    OccupancyGrid occGrid;
    // List of cells changed since last step (For faster update of image)
    LinkedList<Point> dirtyCells;
    public LinkedList<Point> pathTaken;    // For display where robot has gone

    // Frontiers
    PriorityQueue<Frontier> frontiers;
    Frontier lastFrontier;          // Keep track of last frontier of interest
    //Frontiers that are impossible to reach, so should be discarded
    HashMap<Frontier, Boolean> badFrontiers;
    public int totalSpareTime; //total time this agent was not used for exploration

    // Path
    Path path;
    Path pathToBase;
    //location in range of base station that is nearest to us
    private Point nearestBaseCommunicationPoint;

    private final TopologicalMap topologicalMap;
    int timeTopologicalMapUpdated;
    
    // Teammates
    HashMap<Integer, TeammateAgent> teammates;
    private Agent baseStation;

    // Role-based Exploration
    private RendezvousAgentData rendezvousAgentData;
    private IRendezvousStrategy rendezvousStrategy;

    // dynamic behavior
    private Point currentGoal;  // needed for calculating dynamic role switch

    private SimulatorConfig simConfig;
    
    // Used only for logging - direct reference to other agents. DO NOT use this for anything else
    private SimulationFramework simFramework;
    private int oldTimeElapsed;

    public RealAgent(int envWidth, int envHeight, RobotConfig robot, SimulatorConfig simConfig, Agent baseStation) {
        super(robot.getRobotNumber(),
                robot.getName(),
                robot.getRobotNumber(),
                robot.getStartX(),
                robot.getStartY(),
                robot.getStartHeading(),
                robot.getSensingRange(),
                robot.getCommRange(),
                robot.getBatteryLife(),
                robot.getRole(),
                robot.getParent(),
                robot.getChild(),
                robot.getSpeed(),
                robot.getAbility(),
                robot.getComStationLimit()
        );

        stats = new AgentStats();

        prevX = x;
        prevY = y;

        periodicReturnInterval = 10;
        frontierPeriodicState = 0;
        envError = false;
        timeTopologicalMapUpdated = -1;
        timeElapsed = 0;

        totalSpareTime = 0;

        occGrid = new OccupancyGrid(envWidth, envHeight);
        topologicalMap = new TopologicalMap(null);
        dirtyCells = new LinkedList<Point>();
        pathTaken = new LinkedList<Point>();
        badFrontiers = new HashMap<Frontier, Boolean>();

        frontiers = new PriorityQueue();

        path = new Path();

        teammates = new HashMap<Integer, TeammateAgent>();

        setState(RealAgent.ExploreState.Initial);

        missionComplete = false;

        this.simConfig = simConfig;
        rendezvousAgentData = new RendezvousAgentData(this);
        rendezvousStrategy = null;
        currentGoal = new Point(x, y);

        nearestBaseCommunicationPoint = null;

        this.baseStation = baseStation;
    }
  
// <editor-fold defaultstate="collapsed" desc="Get and Set">
    public AgentStats getStats() {
        return stats;
    }

    public int getTimeElapsed() {
        return timeElapsed;
    }

    public void resetBadFrontiers() {
        badFrontiers.clear();
    }

    public Set<Frontier> getBadFrontiers() {
        return badFrontiers.keySet();
    }

    public void addBadFrontier(Frontier f) {
        badFrontiers.put(f, true);
    }

    public boolean isBadFrontier(Frontier f) {
        return badFrontiers.containsKey(f);
    }

    public int getPrevX() {
        return this.prevX;
    }

    public int getPrevY() {
        return this.prevY;
    }

    public boolean getEnvError() {
        return envError;
    }

    public void setEnvError(boolean err) {
        envError = err;
    }

    public SimulatorConfig getSimConfig() {
        return simConfig;
    }

    public OccupancyGrid getOccupancyGrid() {
        return occGrid;
    }

    public LinkedList<Point> getDirtyCells() {
        if (dirtyCells == null) {
            dirtyCells = new LinkedList<Point>();
        }
        return dirtyCells;
    }

    public void setDirtyCells(LinkedList<Point> list) {
        dirtyCells = list;
    }

    public void addDirtyCells(LinkedList<Point> newDirt) {
        setDirtyCells(mergeLists(getDirtyCells(), newDirt));
    }

    public PriorityQueue<Frontier> getFrontiers() {
        return (this.frontiers);
    }

    public void setFrontiers(PriorityQueue<Frontier> newFrontierList) {
        this.frontiers = newFrontierList;
    }

    public Frontier getLastFrontier() {
        return this.lastFrontier;
    }

    public void setLastFrontier(Frontier f) {
        this.lastFrontier = f;
    }


    public Path getPath() {
        return path;
    }

    public void setPath(Path newPath) {
        if (path != null) {
            this.setDirtyCells(mergeLists(dirtyCells, path.getAllPathPixels()));
        }
        path = newPath;
    }

    public void setSimFramework(SimulationFramework simFramework) {
        this.simFramework = simFramework;
    }
    
    public Point getNextPathPoint() {
        if (path != null) {
            if (path.getPoints() != null) {
                if (!path.getPoints().isEmpty()) {
                    return ((Point) path.getPoints().remove(0));
                }
            }
        }

        return null;
    }

    public void setPathToBaseStation() {
        setPath(getPathToBaseStation());
    }

    public void resetPathToBaseStation() {
        pathToBase = null;
    }


    public Path getPathToBaseStation() {
        if ((pathToBase != null) && ((pathToBase.getPoints() == null) || pathToBase.getPoints().isEmpty())) {
            pathToBase = null;
        }
        if ((pathToBase == null) || (pathToBase.getStartPoint().distance(this.getLocation()) > (Constants.DEFAULT_SPEED * 2))) {
            computePathToBaseStation(true);
        }
        return pathToBase;
    }

    public boolean isMissionComplete() {
        return missionComplete;
    }

    public void setMissionComplete(boolean missionComplete) {
        this.missionComplete = missionComplete;
    }

    public Point getCurrentGoal() {
        return currentGoal;
    }

    public void setCurrentGoal(Point cg) {
        currentGoal = cg;
    }

    public TopologicalMap getTopologicalMap() {
        return topologicalMap;
    }


    public RendezvousAgentData getRendezvousAgentData() {
        return rendezvousAgentData;
    }

    public void setRendezvousAgentData(RendezvousAgentData data) {
        this.rendezvousAgentData = data;
    }

    public IRendezvousStrategy getRendezvousStrategy() {
        if (rendezvousStrategy == null) {
            rendezvousStrategy = RendezvousStrategyFactory.createRendezvousStrategy(simConfig, this);
        }
        return rendezvousStrategy;
    }

    public void setRendezvousStrategy(IRendezvousStrategy strategy) {
        rendezvousStrategy = strategy;
    }
    public TeammateAgent getParentTeammate() {
        return getTeammate(parent);
    }

    public void setParent(int p) {
        parent = p;
    }

    public TeammateAgent getChildTeammate() {
        return getTeammate(child);
    }

    public void setChild(int c) {
        child = c;
    }

    public boolean isExplorer() {
        return (role == roletype.Explorer);
    }   

    public void addTeammate(TeammateAgent teammate) {
        teammates.put(teammate.getID(), teammate);
    }

    public TeammateAgent getTeammate(int n) {
        return teammates.get(n);
    }

    public TeammateAgent getTeammateByNumber(int n) {
        for (TeammateAgent teammate : teammates.values()) {
            if (teammate.getRobotNumber() == n) {
                return teammate;
            }
        }
        return null;
    }
    // necessary when swapping roles with another agent
    public TeammateAgent removeTeammate(int n) {
        return teammates.remove(n);
    }

    public HashMap<Integer, TeammateAgent> getAllTeammates() {
        return teammates;
    }
   
// </editor-fold>    

    public void updatePathDirt(Path p) {
        java.util.List pts = p.getPoints();
        if (pts != null) {
            for (int i = 0; i < pts.size() - 1; i++) {
                addDirtyCells(pointsAlongSegment(((Point) pts.get(i)).x, ((Point) pts.get(i)).y, ((Point) pts.get(i + 1)).x, ((Point) pts.get(i + 1)).y));
            }
        }
    }

    public void computePathToBaseStation(boolean comRangeSufficient) {
        long realtimeStartAgentCycle = System.currentTimeMillis();
        Point baseLocation = baseStation.getLocation();
        if (nearestBaseCommunicationPoint == null) {
            nearestBaseCommunicationPoint = baseLocation;
        }
        if (simConfig.getBaseRange() && (this.getTimeElapsed() % 10 == 1)) {
            List<NearRVPoint> generatedPoints
                    = MultiPointRendezvousStrategy.SampleEnvironmentPoints(this, simConfig.getSamplingDensity());
            NearRVPoint agentPoint = new NearRVPoint(this.getLocation().x, this.getLocation().y);
            List<CommLink> connectionsToBase = MultiPointRendezvousStrategy.FindCommLinks(generatedPoints, this);
            MultiPointRendezvousStrategy.findNearestPointInBaseCommRange(agentPoint, connectionsToBase, this);
            if (agentPoint.parentPoint != null) {
                baseLocation = agentPoint.parentPoint.getLocation();
                nearestBaseCommunicationPoint = baseLocation;
            }
        }
        pathToBase = calculatePath(getLocation(), nearestBaseCommunicationPoint);
        if (Constants.DEBUG_OUTPUT || Constants.PROFILING) {
            System.out.println(this.toString() + "Path to base computation took " + (System.currentTimeMillis() - realtimeStartAgentCycle) + "ms.");
        }
    }
    
    /**
     * This method checks if occupancy grid has changed since last update of topological map,
     * and rebuilds the map if necessary.
     * @param force update even if 'occGrid.hasMapChanged()' is false
     */
    public void updateTopologicalMap(boolean force) {
        if (occGrid.hasMapChanged() || force) {
            //System.out.println(this + " Updating topological map");
            long timeStart = System.currentTimeMillis();
            topologicalMap.setGrid(occGrid);
            //System.out.println(toString() + "setGrid, " + (System.currentTimeMillis()-timeStart) + "ms.");
            topologicalMap.generateSkeleton();
            //System.out.println(toString() + "generateSkeleton, " + (System.currentTimeMillis()-timeStart) + "ms.");
            topologicalMap.findKeyPoints();
            //System.out.println(toString() + "findKeyPoints, " + (System.currentTimeMillis()-timeStart) + "ms.");
            topologicalMap.generateKeyAreas();
            timeTopologicalMapUpdated = timeElapsed;
            if (Constants.DEBUG_OUTPUT || Constants.PROFILING) {
                System.out.println(toString() + "generated topological map, " + (System.currentTimeMillis() - timeStart) + "ms.");
            }
            occGrid.setMapHasChangedToFalse();
        } else if (Constants.DEBUG_OUTPUT) {
            System.out.println(this + " Occupancy Grid not changed since last update, skipping topological map update");
        }
    }

// <editor-fold defaultstate="collapsed" desc="Flush, take step, write step">
    public void flushComms() {
        teammates.values().stream().forEach((teammate) -> {
            teammate.setInRange(false);
        });
    }

    // Overwrite any useless data from previous step
    @Override
    public void flush() {
        prevX = x;
        prevY = y;
        this.oldTimeElapsed = this.timeElapsed;

        // Only for testing, uncommenting the line below leads to massive slow down
        //occGrid.initializeTestBits();
    }

    /**
     * Calculates and returns the next step to go, need to be called several times 
     * until distance (based on agents speed) is reached. 
     * Only the last step neds to be written, but a high speed can cause sensor-flaws, 
     * as if the agent only scans after traveling
     * @param timeElapsed in which cycle are we? Needed for several Exploration-algos
     * @return Point (step) to go
     */
    @Override
    public Point takeStep(int timeElapsed) {
        long realtimeStartAgentStep = System.currentTimeMillis();
        Point nextStep = null;

        //previous time elapsed, used to check if we advanced to a new time cycle
        this.oldTimeElapsed = this.timeElapsed;
        this.timeElapsed = timeElapsed;

        //shall we go out of service?
        if (Math.random() < Constants.PROB_OUT_OF_SERVICE) {
            if ((timeElapsed > (robotNumber * 150)) && (robotNumber > 0)) {
                setState(ExploreState.OutOfService);
            }
        }
        
        //if we are out of service, don't move, act as relay
        if (getState() == ExploreState.OutOfService) {
            setSpeed(0);
            return getLocation();
        }

        if (timeElapsed == 0) {
            oldTimeElapsed = -1; //hack for initial time step
        }
        if (oldTimeElapsed != timeElapsed) {
            // First call in cycle
            //TODO Only needed for Util and RoleBased, but needs to be done on request!!!
            //setDistanceToBase(getPathToBaseStation().getLength());
            switch (simConfig.getExpAlgorithm()) {
                case RunFromLog:
                    nextStep = RunFromLog.takeStep(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber);
                    setState(RunFromLog.getState(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber));
                    setRole(RunFromLog.getRole(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber));
                    //<editor-fold defaultstate="collapsed" desc="Make sure the GUI can display a path estimate">
                    Path straightLine = new Path();
                    straightLine.setStartPoint(getLocation());
                    straightLine.setGoalPoint(RunFromLog.getGoal(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber));
                    straightLine.getPoints().add(straightLine.getStartPoint());
                    straightLine.getPoints().add(straightLine.getGoalPoint());
                    setPath(straightLine);
                    //</editor-fold>
                    break;
                case LeaderFollower:
                    nextStep = LeaderFollower.takeStep(this, timeElapsed);
                    break;
                case FrontierExploration:
                    if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.UtilReturn)) {
                        nextStep = UtilityExploration.takeStep(this, timeElapsed, simConfig);
                    } else {
                        nextStep = FrontierExploration.takeStep(this, timeElapsed, simConfig.getFrontierAlgorithm());
                    }
                    break;
                case RoleBasedExploration:
                    nextStep = RoleBasedExploration.takeStep(this, timeElapsed, this.getRendezvousStrategy());
                    break;
                case Testing:
                    nextStep = RelayFrontierExploration.takeStep(this, timeElapsed, simConfig.getFrontierAlgorithm(), baseStation, simConfig.useComStations(), simConfig.getComStationDropChance());
                    break;
                default:
                    break;
            }
        } else {
            // further call in cycle, just give next points of path
            switch (simConfig.getExpAlgorithm()) {
                case RunFromLog:
                    break;
                case LeaderFollower:
                    nextStep = this.getNextPathPoint();
                    break;
                case FrontierExploration:
                    nextStep = this.getNextPathPoint();
                    break;
                case RoleBasedExploration:
                    if ((this.getState() != ExploreState.GiveParentInfo)
                            && (this.getState() != ExploreState.GetInfoFromChild) 
                            && (this.getState() != ExploreState.WaitForChild)
                            && (this.getState() != ExploreState.WaitForParent)) {
                        nextStep = this.getNextPathPoint();
                    }
                    break;
                case Testing:
                    nextStep = this.getNextPathPoint();
                    break;
                default:
                    break;
            }
        }

        if (Constants.DEBUG_OUTPUT || Constants.PROFILING) {
            String nxt = nextStep == null ? "NOTHING" : ((int) nextStep.getX() + "," + (int) nextStep.getY());
            System.out.println(this.toString() + "Taking step complete, moving from (" + (int) getLocation().getX() + "," + (int) getLocation().getY() + ") to (" + nxt + "), took " + (System.currentTimeMillis() - realtimeStartAgentStep) + "ms.");
        }

        return nextStep;
    }

    /**
     * Go the step and update Agents data (including sensordata).
     * @param nextLoc Step to go
     * @param sensorData sensordata to update the agents map with
     * @param updateSensorData should the sensordata be updated?
     */
    @Override
    public void writeStep(Point nextLoc, double[] sensorData, boolean updateSensorData) {
        long realtimeStart = System.currentTimeMillis();

        //int safeRange = (int) (sensRange * Constants.SAFE_RANGE / 100);
        Polygon newFreeSpace;//, newSafeSpace;

        //Points between our old location and new location are definitely safe, as we are moving through them now!
        LinkedList<Point> safePoints = getOccupancyGrid().pointsAlongSegment(x, y, nextLoc.x, nextLoc.y);
        for (Point p : safePoints) {
            getOccupancyGrid().setFreeSpaceAt(p.x, p.y);
            //getOccupancyGrid().setSafeSpaceAt(p.x, p.y);
            getOccupancyGrid().setNoObstacleAt(p.x, p.y);
        }

        x = nextLoc.x;
        y = nextLoc.y;

        pathTaken.add(new Point(x, y));

        if (!(prevX == x && prevY == y)) {
            heading = Math.atan2(y - prevY, x - prevX);
        }
        stats.addDistanceTraveled(Math.sqrt(Math.pow(y - prevY, 2) + Math.pow(x - prevX, 2)));

        // OLD METHOD RAY TRACING
        // Safe space slightly narrower than free space to make sure frontiers
        // are created along farthest edges of sensor radial polygon
        if (updateSensorData) {
            newFreeSpace = findRadialPolygon(sensorData, sensRange, 0, 180);
            //newSafeSpace = findRadialPolygon(sensorData, safeRange, 0, 180);
            updateObstacles(newFreeSpace);
            updateFreeAndSafeSpace(newFreeSpace, null);
        }

        // NEW METHOD FLOOD FILL
        //updateGrid(sensorData);
        batteryPower--;

        if (Constants.PROFILING){
            System.out.println(this.toString() + "WriteStep complete, took " + 
                    (System.currentTimeMillis()-realtimeStart) + "ms.");
        }
    }

// </editor-fold>     
// <editor-fold defaultstate="collapsed" desc="Calculate paths">
    public Path calculatePath(Point startPoint, Point goalPoint) {
        return calculatePath(startPoint, goalPoint, false);
    }

    public Path calculatePath(Point startPoint, Point goalPoint, boolean pureAStar) {

        if (timeTopologicalMapUpdated < 0) {
            timeTopologicalMapUpdated
                    = timeElapsed - Constants.MUST_REBUILD_TOPOLOGICAL_MAP_INTERVAL;
        }
        if (timeElapsed - timeTopologicalMapUpdated >= Constants.REBUILD_TOPOLOGICAL_MAP_INTERVAL) {
            updateTopologicalMap(false);
        }
        if (timeElapsed - timeTopologicalMapUpdated >= Constants.MUST_REBUILD_TOPOLOGICAL_MAP_INTERVAL) {
            updateTopologicalMap(true);
        }
        boolean topologicalMapUpdated = (timeTopologicalMapUpdated == timeElapsed);
        topologicalMap.setPathStart(startPoint);
        topologicalMap.setPathGoal(goalPoint);
        if (!pureAStar) {
            topologicalMap.calculateTopologicalPath();
            if (!topologicalMap.getPath().found && !topologicalMapUpdated) {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(this + "Trying to rebuild topological map and replan path " 
                            + startPoint + " to " + goalPoint);
                }
                timeTopologicalMapUpdated = -1;
                return calculatePath(startPoint, goalPoint);
            } else if (!topologicalMap.getPath().found) {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(this + "at location (" + (int) getLocation().getX() + "," + (int) getLocation().getY() + ") failed to plan path (" + (int) startPoint.getX() + "," + (int) startPoint.getY() + ") to (" + (int) goalPoint.getX() + "," + (int) goalPoint.getY() + "), not retrying; "
                            + "time topologicalMapUpdated: " + timeTopologicalMapUpdated + ", curTime: " + timeElapsed
                            + ", mapCellsChanged: " + occGrid.getMapCellsChanged() + "/" + Constants.MAP_CHANGED_THRESHOLD);
                }
            }
        } else {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(this + "calculating jump path from " 
                        + startPoint + " to " + goalPoint);
            }
            topologicalMap.calculateJumpPath();
            if (topologicalMap.getPath() == null) {
                System.err.println("!!!! CATASTROPHIC FAILURE !!!!!");
            }
        }
        return topologicalMap.getPath();
    }

    private LinkedList<Point> pointsAlongSegment(int x1, int y1, int x2, int y2) {
        LinkedList<Point> pts = new LinkedList<Point>();

        for (int i = Math.min(x1, x2); i <= Math.max(x1, x2); i++) {
            for (int j = Math.min(y1, y2); j <= Math.max(y1, y2); j++) {
                if (occGrid.distPointToLine(x1, y1, x2, y2, i, j) < 0.5) {
                    pts.add(new Point(i, j));
                }
            }
        }

        return pts;
    }
// </editor-fold>

    // 

    /**
     * update stats of what we know about the environment.
     * TODO: we shouldn't call this every time step, this is a performance bottleneck and 
     * can be made more efficient.
     */
    public void updateAreaKnown() {
        //<editor-fold defaultstate="collapsed" desc="Commented out - brute force">
        // brute force" way of calculating map stats">
        /*int counter = 0;
        int new_counter = 0;
        int gotRelayed = 0;
        int baseCounter = 0;
        for(int i=0; i<occGrid.width; i++)
        for(int j=0; j<occGrid.height; j++)
        if (occGrid.freeSpaceAt(i,j))
        {
        counter++;
        if ((!occGrid.isKnownAtBase(i, j)) && (!occGrid.isGotRelayed(i, j)))
        new_counter++;
        if ((!occGrid.isKnownAtBase(i, j)) && (occGrid.isGotRelayed(i, j)))
        gotRelayed++;
        if (occGrid.isKnownAtBase(i, j))// || occGrid.isGotRelayed(i, j))
        baseCounter++;
        }
        
        areaKnown = counter;
        newInfo = new_counter;
        percentageKnown = (double)areaKnown / (double)areaGoal;
        
        if (baseCounter != occGrid.getNumFreeCellsKnownAtBase())
        System.out.println("@@@@@@@@@@@ OccGrid baseCounter corrupted, expected " + baseCounter +
        " got " + occGrid.getNumFreeCellsKnownAtBase() + " @@@@@@@@@");
        if (gotRelayed != occGrid.getNumFreeRelayedCells())
        System.out.println("@@@@@@@@@@@ OccGrid gotRelayed counter corrupted, expected " + gotRelayed +
        " got " + occGrid.getNumFreeRelayedCells() + " @@@@@@@@@");
        if (areaKnown != occGrid.getNumFreeCells())
        System.out.println("@@@@@@@@@@@ OccGrid freeCells counter corrupted, expected " + areaKnown +
        " got " + occGrid.getNumFreeCells() + " @@@@@@@@@");
        if (newInfo != (occGrid.getNumFreeCells() - occGrid.getNumFreeCellsKnownAtBase() - occGrid.getNumFreeRelayedCells()))
        System.out.println("@@@@@@@@@@@ OccGrid newInfo calculation wrong, expected " + newInfo +
        " got " + (occGrid.getNumFreeCells() - occGrid.getNumFreeCellsKnownAtBase() - occGrid.getNumFreeRelayedCells()) + " @@@@@@@@@");*/
//</editor-fold>

        stats.setAreaKnown(occGrid.getNumFreeCells());
        stats.setNewInfo(occGrid.getNumFreeCells() - occGrid.getNumFreeCellsKnownAtBase() - occGrid.getNumFreeRelayedCells());
        stats.setPercentageKnown((double) stats.getAreaKnown() / (double) stats.getGoalArea());

        stats.setCurrentBaseKnowledgeBelief(occGrid.getNumFreeCellsKnownAtBase() + occGrid.getNumFreeRelayedCells());
        // ^^^ can add them up, as they are disjoint;
        // may be a good idea to add a discounted value for gotRelayed, as we are not sure it is going to be delivered
        // to base soon. The reason we incorporate gotRelayed to reduce the probability of agents trying to go back to base
        // before the ratio is hit, and then having to go back to exploring when they learn that base station knows more
        // information than they thought, resulting in wasted effort.
        stats.setCurrentTotalKnowledgeBelief(stats.getCurrentBaseKnowledgeBelief() + stats.getNewInfo() * (teammates.size() - 1));

        if (timeElapsed > 0) {
            double rateOfInfoGatheringBelief = (double) stats.getAreaKnown() / (double) timeElapsed;
            if (rateOfInfoGatheringBelief > stats.getMaxRateOfInfoGatheringBelief()) {
                stats.setMaxRateOfInfoGatheringBelief(rateOfInfoGatheringBelief);
            }
        }
    }

    // TODO: Should be able to make this more efficient.
    public void updateAreaRelayed(TeammateAgent ag) {
        //long timer = System.currentTimeMillis();
        if (ag.robotNumber == robotNumber) //we are the same as ag, nothing to do here
        {
            return;
        }
        if (getID() == Constants.BASE_STATION_TEAMMATE_ID) //base station
        {
            return;
        }
        if (ag.timeToBase() == timeToBase()) { //robots overlapping in sim; couldn't happen in real life.
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(toString() + " timeToBase same as " + ag.name + " (" + timeToBase() + " vs " + ag.timeToBase() + ")");
            }
            return;                          //anyway, this means there is symmetry so could potentially lose new data
        }

        if (ag.getNewInfo() == 0 || stats.getNewInfo() == 0) { //only at most one agent is responsible for data
            if (Math.abs(ag.timeToBase() - timeToBase()) < 3) //then only 'swap' data if significant advantage is offered
            {
                return;                                     //to prevent oscillations
            }
        }
        int new_counter = 0;
        boolean iAmCloserToBase = (ag.timeToBase() > timeToBase()); //buffer to prevent oscillations
        if (iAmCloserToBase) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(toString() + " relaying for " + ag.name + " (" + timeToBase() + " vs " + ag.timeToBase() + ")");
            } else if (Constants.DEBUG_OUTPUT) {
                System.out.println(ag.name + " relaying for " + toString() + " (" + ag.timeToBase() + " vs " + timeToBase() + ")");
            }
        }

        if (iAmCloserToBase) {
            for (Point point : ag.occGrid.getOwnedCells()) {
                if (occGrid.isGotRelayed(point.x, point.y)) {
                    occGrid.setGotUnrelayed(point.x, point.y);
                    new_counter++;
                }
            }

            stats.setNewInfo(occGrid.getNumFreeCells() - occGrid.getNumFreeCellsKnownAtBase() - occGrid.getNumFreeRelayedCells());

            if (new_counter > 0) {
                //Set new info to 1 to prevent oscillations - this get updated properly in updateAreaKnown
                if (stats.getNewInfo() == 0) {
                    stats.setNewInfo(1);
                }
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println(toString() + "setGotUnrelayed: " + new_counter);
                }
            }

            if ((simConfig.getExpAlgorithm() == SimulatorConfig.exptype.FrontierExploration)
                    && (simConfig.getFrontierAlgorithm() == SimulatorConfig.frontiertype.UtilReturn)) {
                if (((getState() == ExploreState.ReturnToParent) || ag.getState() == ExploreState.ReturnToParent)
                        && (ag.getNewInfo() > 1 || stats.getNewInfo() > 1)) {
                    setState(ExploreState.ReturnToParent);
                } else {
                    setState(ExploreState.Explore);
                }
            }

        } else {
            if ((simConfig.getExpAlgorithm() == SimulatorConfig.exptype.FrontierExploration)
                    && (simConfig.getFrontierAlgorithm() == SimulatorConfig.frontiertype.UtilReturn)) {
                setState(ExploreState.Explore);
            }
            if (stats.getNewInfo() > 0) {
                // Need to iterate over a copy of getOwnedCells list, as the list gets changed by setGotRelayed.
                new_counter = occGrid.setOwnedCellsRelayed();
                if (new_counter > 0) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println(toString() + "setGotRelayed: " + new_counter);
                    }
                }
            }
        }

        if ((getState() != ExploreState.Initial) && (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.FrontierExploration)) {
            setStateTimer(0); //replan
        }        //System.out.println(this.toString() + "updateAreaRelayed took " + (System.currentTimeMillis()-timer) + "ms.\n");
    }

    protected void updateGrid(double sensorData[]) {
        Point first, second;
        double dist, angle;
        int currX, currY;
        double currRayAngle;
        int currRayX, currRayY;

        //If there's no sensor data, done
        if (sensorData == null) {
            return;
        }

        int safeRange = (int) (sensRange * Constants.SAFE_RANGE / 100);

        // To create a polygon, add robot at start and end
        Polygon polygon = new Polygon();
        polygon.addPoint(x, y);

        //For every degree
        for (int i = 0; i <= 180; i += 1) {
            currRayAngle = heading - Math.PI / 2 + Math.PI / 180 * i;

            if (sensorData[i] >= sensRange) {
                currRayX = x + (int) Math.round(sensRange * (Math.cos(currRayAngle)));
                currRayY = y + (int) Math.round(sensRange * (Math.sin(currRayAngle)));
            } else {
                currRayX = x + (int) Math.round(sensorData[i] * (Math.cos(currRayAngle)));
                currRayY = y + (int) Math.round(sensorData[i] * (Math.sin(currRayAngle)));
            }

            polygon.addPoint(currRayX, currRayY);
        }

        polygon.addPoint(x, y);

        // determine mins and maxes of polygon
        int xmin = polygon.getBounds().x;
        int ymin = polygon.getBounds().y;

        // Create temp grid 
        int[][] tempGrid = new int[polygon.getBounds().width + 1][polygon.getBounds().height + 1];
        for (int i = 0; i < polygon.getBounds().width + 1; i++) {
            for (int j = 0; j < polygon.getBounds().height + 1; j++) {
                tempGrid[i][j] = 0;
            }
        }

        /*
         * ******************************************
         */
        //   I   Make outline of polygon
        // go through all points in scan, set lines between them to obstacle
        for (int i = 0; i < polygon.npoints - 1; i++) {
            first = new Point(polygon.xpoints[i], polygon.ypoints[i]);
            second = new Point(polygon.xpoints[i + 1], polygon.ypoints[i + 1]);
            dist = first.distance(second);
            angle = Math.atan2(second.y - first.y, second.x - first.x);
            for (int j = 0; j < dist; j++) {
                currX = (int) (first.x + j * Math.cos(angle));
                currY = (int) (first.y + j * Math.sin(angle));
                tempGrid[currX - xmin][currY - ymin] = 1;
            }
        }

        /*
         * ******************************************
         */
        //   II  Fill free space from inside
        // first, create
        LinkedList<Point> toFill = new LinkedList<Point>();
        Point start = new Point((int) (x + 1 * Math.cos(heading) - xmin), (int) (y + 1 * Math.sin(heading) - ymin));

        // if start is obstacle (robot right in front of wall), don't bother updating anything
        if (occGrid.obstacleAt(start.x, start.y)) {
            return;
        }

        Point curr;
        toFill.add(start);

        while (!toFill.isEmpty()) {
            curr = toFill.pop();
            tempGrid[curr.x][curr.y] = 2;

            // add neighbours
            if (curr.x - 1 >= 0 && tempGrid[curr.x - 1][curr.y] == 0) {
                tempGrid[curr.x - 1][curr.y] = -1;
                toFill.add(new Point(curr.x - 1, curr.y));
            }
            if (curr.x + 1 <= polygon.getBounds().width && tempGrid[curr.x + 1][curr.y] == 0) {
                tempGrid[curr.x + 1][curr.y] = -1;
                toFill.add(new Point(curr.x + 1, curr.y));
            }
            if (curr.y - 1 >= 0 && tempGrid[curr.x][curr.y - 1] == 0) {
                tempGrid[curr.x][curr.y - 1] = -1;
                toFill.add(new Point(curr.x, curr.y - 1));
            }
            if (curr.y + 1 <= polygon.getBounds().height && tempGrid[curr.x][curr.y + 1] == 0) {
                tempGrid[curr.x][curr.y + 1] = -1;
                toFill.add(new Point(curr.x, curr.y + 1));
            }
        }

        /*
         * ******************************************
         */
        //   III Fill in obstacles
        // go through all points in polygon (avoid first and last because it's the robot)
        for (int i = 1; i < polygon.npoints - 2; i++) {
            first = new Point(polygon.xpoints[i], polygon.ypoints[i]);
            second = new Point(polygon.xpoints[i + 1], polygon.ypoints[i + 1]);

            // if they are close enough, fill line between them
            if (first.distance(x, y) < (sensRange - 2) && second.distance(x, y) < (sensRange - 2)
                    && first.distance(second) < 5) {
                angle = Math.atan2(second.y - first.y, second.x - first.x);
                for (double j = 0; j < first.distance(second); j++) {
                    currX = (int) (first.x + j * Math.cos(angle));
                    currY = (int) (first.y + j * Math.sin(angle));
                    tempGrid[currX - xmin][currY - ymin] = 4;
                }
            }
        }

        /*
         * ******************************************
         */
        //   IV  Update real grid
        for (int i = 0; i < polygon.getBounds().width; i++) {
            for (int j = 0; j < polygon.getBounds().height; j++) {
                if (tempGrid[i][j] == 4) {
                    occGrid.setObstacleAt(i + xmin, j + ymin);
                    dirtyCells.add(new Point(i + xmin, j + ymin));
                } else if (tempGrid[i][j] == 2) {
                    if ((new Point(x, y)).distance(new Point(i + xmin, j + ymin)) < safeRange) { // &&
                        //angleDiff(Math.atan2((j+ymin)-y, (i+xmin)-x), heading) < 80)
                        occGrid.setSafeSpaceAt(i + xmin, j + ymin);
                        dirtyCells.add(new Point(i + xmin, j + ymin));
                    } else {
                        occGrid.setFreeSpaceAt(i + xmin, j + ymin);
                        dirtyCells.add(new Point(i + xmin, j + ymin));

                    }
                }
            }
        }
    }

    protected int angleDiff(double theta1, double theta2) {
        //System.out.println(theta1 + " " + theta2);
        int angle1 = (int) (180 / Math.PI * theta1 + 360) % 360;
        int angle2 = (int) (180 / Math.PI * theta2 + 360) % 360;
        int diff = Math.abs(angle1 - angle2);
        return (Math.min(diff, 360 - diff));
    }

    protected void updateFreeAndSafeSpace(Polygon newFreeSpace, Polygon newSafeSpace) {
        // May be possible to do some optimization here, I think a lot of cells are checked unnecessarily
        boolean sensedNew = false;
        boolean doubleSensed = false;
        for (int i = newFreeSpace.getBounds().x; i <= newFreeSpace.getBounds().x + newFreeSpace.getBounds().width; i++) {
            innerloop:
            for (int j = newFreeSpace.getBounds().y; j <= newFreeSpace.getBounds().y + newFreeSpace.getBounds().height; j++) {
                if (occGrid.locationExists(i, j)) {
                    if (newFreeSpace.contains(i, j) && !occGrid.freeSpaceAt(i, j)) {
                        if (!occGrid.obstacleAt(i, j)) {
                            sensedNew = true;
                            //need to check if it was new sensing or double-sensing
                            //note that agent itself has no way of knowing this. So we do a dirty hack here and check the
                            //occupancy grids of other agents directly. This is fine though as we only do this for logging.
                            if (simFramework.hasCellBeenSensedByAnyAgent(i, j)) {
                                doubleSensed = true;
                            }
                            occGrid.setFreeSpaceAt(i, j);
                            dirtyCells.add(new Point(i, j));
                        }
                    }
                    if (newSafeSpace != null && newSafeSpace.contains(i, j)) {
                        // double for loop to prevent empty-safe boundary (which
                        // would not qualify as a frontier)
                        for (int m = i - 1; m <= i + 1; m++) {
                            for (int n = j - 1; n <= j + 1; n++) {
                                if (occGrid.locationExists(m, n) && occGrid.emptyAt(m, n)) {
                                    continue innerloop;
                                }
                            }
                        }
                        if (!occGrid.safeSpaceAt(i, j)) {
                            occGrid.setSafeSpaceAt(i, j);
                            occGrid.setNoObstacleAt(i, j);
                            dirtyCells.add(new Point(i, j));
                        }
                    }
                }
            }
        }

        //update stats for reporting/logging
        if (sensedNew) {
            if (!doubleSensed) {
                stats.incrementTimeSensing(timeElapsed);
            } else {
                stats.incrementTimeDoubleSensing(timeElapsed);
            }
        }
        if (getState().equals(ExploreState.ReturnToParent)
                || getState().equals(ExploreState.WaitForParent)
                || getState().equals(ExploreState.WaitForChild)
                || //getState().equals(ExploreState.GetInfoFromChild) ||
                getState().equals(ExploreState.GiveParentInfo)) //don't include GoToChild here, as in UtilityExploration
        //agents end up going to child in Explore state as
        //they don't know they are going to child at the time!
        {
            stats.incrementTimeReturning(timeElapsed);
        }
    }

    protected void updateObstacles(Polygon newFreeSpace) {
        // Update obstacles -- all those bits for which radial polygon < sensRange
        // Ignore first point in radial polygon as this is the robot itself.
        Point first;
        Point second;
        double angle;
        int currX;
        int currY;

        for (int i = 1; i < newFreeSpace.npoints - 1; i++) {
            first = new Point(newFreeSpace.xpoints[i], newFreeSpace.ypoints[i]);
            second = new Point(newFreeSpace.xpoints[i + 1], newFreeSpace.ypoints[i + 1]);

            // if two subsequent points are close and both hit an obstacle, assume there is a line between them.
            if (first.distance(x, y) < (sensRange - 2) && second.distance(x, y) < (sensRange - 2)
                    && first.distance(second) < 5) { // used to be 10
                angle = Math.atan2(second.y - first.y, second.x - first.x);
                for (int j = 0; j < first.distance(second); j++) {
                    currX = first.x + (int) (j * (Math.cos(angle)));
                    currY = first.y + (int) (j * (Math.sin(angle)));
                    double angleDepth = Math.atan2(currY - y, currX - x);
                    for (int k = 0; k < Constants.WALL_THICKNESS; k++) {
                        int newX = currX + (int) (k * Math.cos(angleDepth));
                        int newY = currY + (int) (k * Math.sin(angleDepth));
                        //free space is final,
                        //otherwise, we can get inaccessible pockets of free space in our map that are actually
                        //accessible. This can lead to frontiers or RV points we cannot plan a path to, which can lead
                        //to all sorts of tricky problems.
                        if (!occGrid.safeSpaceAt(newX, newY)) {
                            occGrid.setObstacleAt(newX, newY);
                            //mark obstacles that we know for sure are there
                            //if (k == 0) occGrid.setSafeSpaceAt(newX, newY); 
                            dirtyCells.add(new Point(currX, currY));
                        }
                    }
                }
            }
            if (first.distance(x, y) < (sensRange - 2)) {
                if (!occGrid.safeSpaceAt(first.x, first.y)) {
                    occGrid.setObstacleAt(first.x, first.y);
                    occGrid.setSafeSpaceAt(first.x, first.y);
                    dirtyCells.add(new Point(first.x, first.y));
                }
            }
        }
        /*if (second.distance(x,y) < (sensRange-2)) {
            if (!occGrid.safeSpaceAt(second.x, second.y)) {
                occGrid.setObstacleAt(second.x, second.y);
                dirtyCells.add(new Point(second.x, second.y));
            }
        }*/
    }

    protected Polygon findRadialPolygon(double sensorData[], int maxRange, int startAngle, int finishAngle) {
        double currRayAngle;
        int currRayX, currRayY;

        Polygon radialPolygon = new Polygon();
        radialPolygon.addPoint(x, y);

        //If there's no sensor data, done
        if (sensorData == null) {
            return radialPolygon;
        }

        //For every degree
        for (int i = startAngle; i <= finishAngle; i += 1) {
            currRayAngle = heading - Math.PI / 2 + Math.PI / 180 * i;
            //sensorData[i]--;
            //maxRange--;
            if (sensorData[i] >= maxRange) {
                currRayX = x + (int) Math.round(maxRange * (Math.cos(currRayAngle)));
                currRayY = y + (int) Math.round(maxRange * (Math.sin(currRayAngle)));
            } else {
                currRayX = x + (int) Math.round(sensorData[i] * (Math.cos(currRayAngle)));
                currRayY = y + (int) Math.round(sensorData[i] * (Math.sin(currRayAngle)));
            }

            radialPolygon.addPoint(currRayX, currRayY);
        }

        return radialPolygon;
    }

    // </editor-fold>     
// <editor-fold defaultstate="collapsed" desc="Communicate">
    public void receiveMessage(DataMessage msg) {
        TeammateAgent teammate = getTeammateByNumber(msg.ID);

        msg.receiveMessage(this, teammate);

        boolean isBaseStation = false;
        if ((teammate.getRobotNumber() == Constants.BASE_STATION_TEAMMATE_ID)
                || (this.getRobotNumber() == Constants.BASE_STATION_TEAMMATE_ID)) {
            isBaseStation = true;
        }

        //merge the occupancy grids, and add affected cells to dirty cell list to be repainted in the GUI
        dirtyCells.addAll(
                occGrid.mergeGrid(teammate.getOccupancyGrid(), isBaseStation));

        if ((simConfig != null)
                && ((simConfig.getExpAlgorithm() == SimulatorConfig.exptype.FrontierExploration)
                && (simConfig.getFrontierAlgorithm() == SimulatorConfig.frontiertype.UtilReturn))
                || (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.RoleBasedExploration)) //this line just so we can compare logs with UtilReturn exploration
        {
            updateAreaRelayed(teammate);
        }

        //replan?
        //stats.setTimeSinceLastPlan(Integer.MAX_VALUE);
    }

    public boolean isCommunicating() {
        return teammates.values().stream().anyMatch((teammate) -> (teammate.isInRange()));
    }

    public void updateAfterCommunication() {
        teammates.values().stream().filter((teammate) -> (!teammate.isInRange())).forEach((teammate) -> {
            teammate.setTimeSinceLastComm(teammate.getTimeSinceLastComm() + 1);
        }); //processRelayMarks();
    }
// </editor-fold>     

    public void dropComStation() {
        if (comStations.size() > 0) {
            ComStation comStation = this.comStations.remove(0);
            comStation.setState(ExploreState.RELAY);
            comStation.setX(this.x);
            comStation.setY(this.y);
        }
    }
}
