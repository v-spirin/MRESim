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

import Logging.AgentStats;
import communication.DataMessage;
import config.RobotConfig;
import config.RobotConfig.roletype;
import config.SimConstants;
import config.SimulatorConfig;
import environment.Frontier;
import environment.OccupancyGrid;
import environment.TopologicalMap;
import exploration.Exploration;
import exploration.FrontierExploration;
import exploration.LeaderFollower;
import exploration.RandomExploration;
import exploration.RoleBasedExploration;
import exploration.RunFromLog;
import exploration.WallFollowExploration;
import exploration.rendezvous.IRendezvousStrategy;
import exploration.rendezvous.RendezvousAgentData;
import exploration.rendezvous.RendezvousStrategyFactory;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.PriorityQueue;
import path.Path;
import simulator.SimulationFramework;

/**
 *
 * @author Julian de Hoog
 */
public class RealAgent extends Agent {

    AgentStats stats;
    private boolean missionComplete; // true only when mission complete (env fully explored)

    int prevX, prevY;             // Previous position in environment
    int timeElapsed;
    private boolean envError;             // set to true by env when RealAgent's step is not legal
    OccupancyGrid occGrid;
    // List of cells changed since last step (For faster update of image)
    LinkedList<Point> dirtyCells;

    // Frontiers
    PriorityQueue<Frontier> frontiers;
    Frontier frontier;          // Keep track of last frontier of interest
    //Frontiers that are impossible to reach, so should be discarded
    LinkedList<Frontier> badFrontiers;
    public int totalSpareTime; //total time this agent was not used for exploration

    private Path path;
    //location in range of base station that is nearest to us
    private Point nearestBaseCommunicationPoint;

    private final TopologicalMap topologicalMap;
    int timeTopologicalMapUpdated;

    // Teammates
    HashMap<Integer, TeammateAgent> teammates;
    private RealAgent baseStation;

    // Role-based Exploration
    private RendezvousAgentData rendezvousAgentData;
    private IRendezvousStrategy rendezvousStrategy;

    private SimulatorConfig simConfig;

    // Used only for logging - direct reference to other agents. DO NOT use this for anything else
    private SimulationFramework simFramework;
    private int oldTimeElapsed;
    private boolean stepFinished;

    /**
     * Indicates that the agend did something using occupied cycles (cycles left until available
     * again).
     */
    private int occupied = 0;
    Exploration exploration;
    private String dynamicInfo;
    private int originalChild;
    private int originalParent;

    public String command = "";
    public Integer command_data = -1;

    private boolean hasDualComLink = false;

    public RealAgent(int envWidth, int envHeight, RobotConfig robot, SimulatorConfig simConfig, RealAgent baseStation) {
        super(robot);

        stats = new AgentStats();

        prevX = x;
        prevY = y;

        envError = false;
        stepFinished = false;
        timeTopologicalMapUpdated = -1;
        timeElapsed = 0;

        totalSpareTime = 0;

        occGrid = new OccupancyGrid(envWidth, envHeight);
        topologicalMap = new TopologicalMap(occGrid);
        dirtyCells = new LinkedList<Point>();
        badFrontiers = new LinkedList<Frontier>();

        frontiers = new PriorityQueue();

        teammates = new HashMap<Integer, TeammateAgent>();

        setState(RealAgent.AgentState.Initial);

        missionComplete = false;

        this.simConfig = simConfig;
        rendezvousAgentData = new RendezvousAgentData(this);
        rendezvousStrategy = null;

        nearestBaseCommunicationPoint = null;

        this.baseStation = baseStation;
        this.oldTimeElapsed = -1;
        this.timeElapsed = -1;
        this.originalChild = this.child;
        this.originalParent = this.parent;
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

    public LinkedList<Frontier> getBadFrontiers() {
        return badFrontiers;
    }

    public void addBadFrontier(Frontier f) {
        badFrontiers.add(f);
    }

    public boolean isBadFrontier(Frontier f) {
        return badFrontiers.contains(f);
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

    public void resetDirtyCells() {
        dirtyCells = new LinkedList<Point>();
    }

    public void addDirtyCells(LinkedList<Point> newDirt) {
        dirtyCells = mergeLists(getDirtyCells(), newDirt);
    }

    public PriorityQueue<Frontier> getFrontiers() {
        if (exploration instanceof FrontierExploration) {
            return ((FrontierExploration) this.exploration).getFrontiers();
        } else {
            return null;
        }
    }

    public Frontier getFrontier() {
        return this.frontier;
    }

    public void setFrontier(Frontier f) {
        this.frontier = f;
    }

    public LinkedList<Point> getPathTaken() {
        if (this.path == null) {
            return new LinkedList<Point>();
        }
        return this.path.getTakenPathPixels();
    }

    public LinkedList<Point> getPathComming() {
        if (this.path == null) {
            return new LinkedList<Point>();
        }
        return this.path.getCommingPathPixels();
    }

    public Path getPath() {
        return path;
    }

    public void setPath(Path newPath) {
        if (path != null) {
            this.addDirtyCells(path.getAllPathPixels());
        }
        path = newPath;
        if (path == null) {
            System.err.println("Path is null");
        } else {
            path.start();
        }
    }

    public void setExactPath(Path newPath) {
        if (path != null) {
            this.addDirtyCells(path.getAllPathPixels());
        }
        path = calculatePath(newPath.getGoalPoint(), true);
        if (path == null) {
            System.err.println("Path is null");
        } else {
            path.start();
        }
    }

    public void setSimFramework(SimulationFramework simFramework) {
        this.simFramework = simFramework;
    }

    public Point getNextPathPoint() {
        return path.nextPoint();
    }

    public void setPathToBaseStation(boolean exact) {
        setPath(calculatePath(this.getAllTeammates().get(SimConstants.BASE_STATION_TEAMMATE_ID).getLocation(), exact));
    }

    public boolean isMissionComplete() {
        return missionComplete;
    }

    public void setMissionComplete(boolean missionComplete) {
        this.missionComplete = missionComplete;
    }

    public Point getCurrentGoal() {
        return path.getGoalPoint();
    }

    public TopologicalMap getTopologicalMap() {
        return topologicalMap;
    }

    public RendezvousAgentData getRendezvousAgentData() {
        return rendezvousAgentData;
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

    public TeammateAgent getOriginalChildTeammate() {
        return getTeammate(originalChild);
    }

    public void setChild(int c) {
        child = c;
    }

    public boolean isExplorer() {
        return (role == roletype.Explorer);
    }

    public void addTeammate(TeammateAgent teammate) {
        if (teammate.getID() == this.ID) {
            return;
        }
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

    public boolean isStepFinished() {
        return stepFinished;
    }

    public void setStepFinished(boolean stepFinished) {
        this.stepFinished = stepFinished;
    }

// </editor-fold>
    public Point stay() {
        this.stepFinished = true;
        return this.getLocation();
    }

    public void updatePathDirt() {
        if (path == null) {
            return;
        }
        addDirtyCells(path.getAllPathPixels());
        /*        java.util.List pts = path.getPoints();
        if (pts != null) {
            for (int i = 0; i < pts.size() - 1; i++) {
                addDirtyCells(pointsAlongSegment(((Point) pts.get(i)).x, ((Point) pts.get(i)).y, ((Point) pts.get(i + 1)).x, ((Point) pts.get(i + 1)).y));
            }
        }*/
    }

    /**
     * This method checks if occupancy grid has changed since last update of topological map, and
     * rebuilds the map if necessary.
     *
     * @param force update even if 'occGrid.hasMapChanged()' is false
     */
    final public void updateTopologicalMap(boolean force) {
        if (occGrid.hasMapChanged() || force) {
            //System.out.println(this + " Updating topological map");
            topologicalMap.setGrid(occGrid);
            topologicalMap.update(force);
            timeTopologicalMapUpdated = timeElapsed;
            occGrid.setMapHasChangedToFalse();
        }
        //In this situation it might be a good idea to reset bad frontiers
        resetBadFrontiers();
    }

// Flush, take step, write step
    public void flushComms() {
        teammates.values().stream().forEach((teammate) -> {
            teammate.setCommunicationLink(false);
            teammate.setDirectComLink(0);
            //teammate.setBaseComLink(false);
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
     * Calculates and returns the next step to go, need to be called several times until distance
     * (based on agents speed) is reached. Only the last step neds to be written, but a high speed
     * can cause sensor-flaws, as if the agent only scans after traveling
     *
     * @param timeElapsed in which cycle are we? Needed for several Exploration-algos
     * @return Point (step) to go
     */
    @Override
    public Point takeStep(int timeElapsed) {
        Point nextStep = null;

        //previous time elapsed, used to check if we advanced to a new time cycle
        this.oldTimeElapsed = this.timeElapsed;
        this.timeElapsed = timeElapsed;
        if (oldTimeElapsed != timeElapsed) {
            if (occupied != 0) {
                occupied -= 1;
            }
        }
        if (occupied > 0) {
            this.setState(AgentState.OCCUPIED);
            return stay();
        } else {
            setState(AgentState.AKTIVE);
        }
        //shall we go out of service?
        if (Math.random() < SimConstants.PROB_OUT_OF_SERVICE) {
            if ((timeElapsed > (robotNumber * 150)) && (robotNumber > 0)) {
                setState(AgentState.OutOfService);
            }
        }

        //if we are out of service, don't move, act as relay
        if (getState() == AgentState.OutOfService) {
            setSpeed(0);
            return getLocation();
        }

        if (oldTimeElapsed != timeElapsed) {
            // First call in cycle
            if (exploration == null) {
                switch (simConfig.getExpAlgorithm()) {
                    case RunFromLog:
                        exploration = new RunFromLog(simConfig.getRunFromLogFilename(), this.robotNumber);
                        setState(((RunFromLog) exploration).getState(timeElapsed));
                        setRole(((RunFromLog) exploration).getRole(timeElapsed));
                        break;

                    case LeaderFollower:
                        exploration = new LeaderFollower(this, simConfig, baseStation);
                        break;
                    case FrontierExploration:
                        exploration = new FrontierExploration(this, simConfig, baseStation);
                        break;
                    case RoleBasedExploration:
                        exploration = new RoleBasedExploration(timeElapsed, this, simConfig, this.getRendezvousStrategy(), baseStation);
                        break;
                    case Testing:
                    case Random:
                        exploration = new RandomExploration(this, simConfig);
                        break;
                    case WallFollow:
                        exploration = new WallFollowExploration(this, simConfig, occGrid);
                        break;
                    default:
                        exploration = new RandomExploration(this, simConfig);
                        break;
                }
            }

            nextStep = exploration.takeStep(timeElapsed);
            if (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.RunFromLog) {
                //Make sure the GUI can display a path estimate
                Path straightLine = new Path(occGrid, getLocation(), ((RunFromLog) exploration).getGoal(timeElapsed), true, true, false);
                setPath(straightLine);
            }
            incrementStateTimer();
        } else if (getEnvError() || path == null || !path.isValid()) {
            //enverror handling, just stay
            nextStep = stay();
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
                    nextStep = this.getNextPathPoint();
                    break;
                case Testing:
                    nextStep = this.getNextPathPoint();
                    break;
                case Random:
                    nextStep = exploration.takeStep(timeElapsed);
                    break;
                case WallFollow:
                    ((WallFollowExploration) exploration).updateGrid(occGrid);
                    nextStep = exploration.takeStep(timeElapsed);

                default:
                    break;
            }
        }

        return nextStep;
    }

    /**
     * Go the step and update Agents data (including sensordata).
     *
     * @param nextLoc Step to go
     * @param sensorData sensordata to update the agents map with
     * @param updateSensorData should the sensordata be updated?
     */
    @Override
    public void writeStep(Point nextLoc, double[] sensorData, boolean updateSensorData
    ) {
        long realtimeStart = System.currentTimeMillis();

        //int safeRange = (int) (sensRange * SimConstants.SAFE_RANGE / 100);
        Polygon newFreeSpace;//, newSafeSpace;

        //Points between our old location and new location are definitely safe, as we are moving through them now!
        /*LinkedList<Point> safePoints = getOccupancyGrid().pointsAlongSegment(x, y, nextLoc.x, nextLoc.y);
        for (Point p : safePoints) {
            getOccupancyGrid().setFreeSpaceAt(p.x, p.y);
            //getOccupancyGrid().setSafeSpaceAt(p.x, p.y);
            getOccupancyGrid().setNoObstacleAt(p.x, p.y);
        }*/
        this.x = nextLoc.x;
        this.y = nextLoc.y;

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
        batteryPower -= energyCunsumption;
        this.getStats().incrementEnergyConsumption(energyCunsumption);
    }

    /**
     * Calculates the standard-path from where the agent is to the given goal with optimizations
     *
     * @param goalPoint
     * @param exact
     * @return
     */
    public Path calculatePath(Point goalPoint, boolean exact) {
        return calculatePath(this.getLocation(), goalPoint, false, exact);
    }

    public Path calculatePath(Point startPoint, Point goalPoint, boolean pureAStar, boolean exact) {
        if (timeElapsed - timeTopologicalMapUpdated >= SimConstants.REBUILD_TOPOLOGICAL_MAP_INTERVAL) {
            if (timeElapsed - timeTopologicalMapUpdated >= SimConstants.MUST_REBUILD_TOPOLOGICAL_MAP_INTERVAL) {
                updateTopologicalMap(true);
            } else {
                updateTopologicalMap(false);
            }
        }

        Path tpath;
        try {
            tpath = new Path(occGrid, topologicalMap, startPoint, goalPoint, false, !pureAStar, exact);
        } catch (IllegalStateException e) {
            tpath = new Path(occGrid, startPoint, goalPoint, false, !pureAStar, exact);
        }

        if (!tpath.found && !(timeTopologicalMapUpdated == timeElapsed)) {
            //Update topological map and retry
            updateTopologicalMap(true);
            return calculatePath(startPoint, goalPoint, false, exact);
        } else if (!tpath.found) {
            System.err.println(this + "at location (" + (int) getLocation().getX() + "," + (int) getLocation().getY() + ") failed to plan path (" + (int) startPoint.getX() + "," + (int) startPoint.getY() + ") to (" + (int) goalPoint.getX() + "," + (int) goalPoint.getY() + "), not retrying; "
                    + "time topologicalMapUpdated: " + timeTopologicalMapUpdated + ", curTime: " + timeElapsed
                    + ", mapCellsChanged: " + occGrid.getMapCellsChanged() + "/" + SimConstants.MAP_CHANGED_THRESHOLD);

        }

        return tpath;
    }

    /**
     * update stats of what we know about the environment.
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

    /**
     * // TODO: Should be able to make this more efficient.
     *
     * @param ag
     */
    public void updateAreaRelayed(TeammateAgent ag) {
        //long timer = System.currentTimeMillis();
        if (ag.robotNumber == robotNumber) //we are the same as ag, nothing to do here
        {
            return;
        }
        if (getID() == SimConstants.BASE_STATION_TEAMMATE_ID) //base station
        {
            return;
        }
        /*if (ag.timeToBase() == timeToBase()) { //robots overlapping in sim; couldn't happen in real life.
            if (SimConstants.DEBUG_OUTPUT) {
                System.out.println(toString() + " timeToBase same as " + ag.name + " (" + timeToBase() + " vs " + ag.timeToBase() + ")");
            }
            return;                          //anyway, this means there is symmetry so could potentially lose new data
        }*/

        if (ag.getNewInfo() == 0 || stats.getNewInfo() == 0) { //only at most one agent is responsible for data
            if (Math.abs(ag.timeToBase() - timeToBase()) < 3) //then only 'swap' data if significant advantage is offered
            {
                return;                                     //to prevent oscillations
            }
        }
        int new_counter = 0;
        boolean iAmCloserToBase = (ag.timeToBase() > timeToBase()); //buffer to prevent oscillations

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
                if (SimConstants.DEBUG_OUTPUT) {
                    System.out.println(toString() + "setGotUnrelayed: " + new_counter);
                }
            }

            /*if ((simConfig.getExpAlgorithm() == SimulatorConfig.exptype.FrontierExploration)
                    && (simConfig.getFrontierAlgorithm() == SimulatorConfig.frontiertype.UtilReturn)) {
                if (((getState() == AgentState.ReturnToBaseStation) || ag.getState() == AgentState.ReturnToBaseStation)
                        && (ag.getNewInfo() > 1 || stats.getNewInfo() > 1)) {
                    setState(AgentState.ReturnToBaseStation);
                } else {
                    setState(AgentState.Explore);
                }
            }*/
        } else {
            if ((simConfig.getExpAlgorithm() == SimulatorConfig.exptype.FrontierExploration)
                    && (simConfig.getFrontierAlgorithm() == SimulatorConfig.frontiertype.UtilReturn)) {
                setState(AgentState.Explore);
            }
            if (stats.getNewInfo() > 0) {
                // Need to iterate over a copy of getOwnedCells list, as the list gets changed by setGotRelayed.
                new_counter = occGrid.setOwnedCellsRelayed();
                if (new_counter > 0) {
                    if (SimConstants.DEBUG_OUTPUT) {
                        System.out.println(toString() + "setGotRelayed: " + new_counter);
                    }
                }
            }
        }

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

        int safeRange = (int) (sensRange * SimConstants.SAFE_RANGE / 100);

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
        Rectangle bounds = newFreeSpace.getBounds();
        int width = Math.min(bounds.x + bounds.width, occGrid.width - 1);
        int height = Math.min(bounds.y + bounds.height, occGrid.height - 1);
        for (int i = Math.max(bounds.x, 0); i <= width; i++) {
            for (int j = Math.max(bounds.y, 0); j <= height; j++) {
                try {
                    if (newFreeSpace.contains(i, j) && !occGrid.freeSpaceAt(i, j)) {
                        if (!occGrid.obstacleAt(i, j)) {
                            sensedNew = true;
                            occGrid.setFreeSpaceAt(i, j);
                            dirtyCells.add(new Point(i, j));
                        }
                    }
                } catch (ArrayIndexOutOfBoundsException e) {
                    //not interresting... but should not happen (at least not often)
                }
            }
        }

        //update stats for reporting/logging
        if (sensedNew) {
            stats.incrementTimeSensing(timeElapsed);
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
                    && first.distance(second) < 7) { // used to be 10 then 5
                angle = Math.atan2(second.y - first.y, second.x - first.x);
                for (int j = 0; j < first.distance(second); j++) {
                    currX = first.x + (int) (j * (Math.cos(angle)));
                    currY = first.y + (int) (j * (Math.sin(angle)));
                    double angleDepth = Math.atan2(currY - y, currX - x);
                    for (int k = 0; k < SimConstants.WALL_THICKNESS; k++) {
                        int newX = currX + (int) (k * Math.cos(angleDepth));
                        int newY = currY + (int) (k * Math.sin(angleDepth));
                        if (newX < 0) {
                            newX = 0;
                        }
                        if (newY < 0) {
                            newY = 0;
                        }
                        occGrid.setObstacleAt(newX, newY);
                        dirtyCells.add(new Point(newX, newY));
                    }
                }
            }
        }
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
    public void receiveMessage(DataMessage msg) {
        stats.incrementCommunications();
        TeammateAgent teammate = getTeammateByNumber(msg.ID);

        msg.receiveMessage(this, teammate);

        boolean isBaseStation = false;
        if ((teammate.getRobotNumber() == SimConstants.BASE_STATION_TEAMMATE_ID)
                || (this.getRobotNumber() == SimConstants.BASE_STATION_TEAMMATE_ID)) {
            isBaseStation = true;
        }

        //merge the occupancy grids, and add affected cells to dirty cell list to be repainted in the GUI
        dirtyCells.addAll(
                occGrid.mergeGrid(teammate.getOccupancyGrid(), isBaseStation));

        updateAreaRelayed(teammate);

    }

    public boolean isCommunicating() {
        return teammates.values().stream().anyMatch((teammate) -> (teammate.hasCommunicationLink()));
    }

    public void updateAfterCommunication() {
        teammates.values().stream().filter((teammate) -> (!teammate.hasCommunicationLink())).forEach((teammate) -> {
            teammate.setTimeSinceLastComm(teammate.getTimeSinceLastComm() + 1);
        });
    }

    public int dropComStation() {
        if (comStations.size() > 0) {
            ComStation comStation = this.comStations.remove(0);
            comStation.setState(AgentState.RELAY);
            comStation.setX(this.x);
            comStation.setY(this.y);
            this.getStats().incrementDroppedComStations();
            baseStation.getStats().incrementDroppedComStations();
            occupied = SimConstants.COM_STATION_DROP_TIME; // use X cycles to drop ComStation (including current one
            return comStation.getID();
        }
        return -1;
    }

    /**
     * takes the first ComStation found in the range of an agent in one step (agent.speed). Not
     * nececcary the nearest one!
     *
     * @return Point of the ComStation
     */
    public boolean liftComStation() {
        ComStation comStation;
        if (comStations.size() < getComStationLimit()) {
            TeammateAgent mate = findNearComStation(2);
            if (mate == null) {
                return false;
            }
            comStation = mate.getReference();
            this.comStations.add(comStation);
            comStation.setState(AgentState.INACTIVE);
            comStation.setX(1);
            comStation.setY(1);
            this.getStats().incrementDroppedComStations();
            this.getStats().decrementDroppedComStations();
            baseStation.getStats().decrementDroppedComStations();
            occupied = SimConstants.COM_STATION_LIFT_TIME; // use X cycles to drop ComStation (including current one
            return true;
        }
        return false;
    }

    /**
     * REturns the first comstation found in the given distance, not nececcary the nearest!
     *
     * @param distance
     * @return ComStation
     */
    public TeammateAgent findNearComStation(int distance) {
        for (TeammateAgent mate : teammates.values()) {
            if (mate.isStationary() && mate.getRole() == roletype.RelayStation && this.getLocation().distance(mate.getLocation()) < distance) {
                return mate; //This works as ComStations are nearly only Agents
            }
        }
        return null;
    }

    public String getDynamicInfoText() {
        return this.dynamicInfo;
    }

    public void setDynamicInfoText(String info) {
        this.dynamicInfo = info;
    }

    public int getOriginalChild() {
        return this.originalChild;
    }

    public void setCommand(String command) {
        if (exploration != null) {
            this.exploration.setCommand(command);
        }
    }

    public void setCommandData(Integer command_data) {
        if (exploration != null) {
            this.exploration.setCommandData(command_data);
        }
    }

    public Integer getCommandData() {
        return this.command_data;
    }

    public String getCommand() {
        return this.command;
    }

    public int getOriginalParent() {
        return this.originalParent;
    }

    public TeammateAgent getOriginalParentTeammate() {
        return this.getTeammate(this.originalParent);
    }

    public boolean hasDualComLink() {
        return this.hasDualComLink;
    }

    public void setHasDualComLink(boolean hasDualComLink) {
        this.hasDualComLink = hasDualComLink;
    }

}
