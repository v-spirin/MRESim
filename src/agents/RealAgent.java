/*
 *     Copyright 2010, 2014 Julian de Hoog (julian@dehoog.ca), Victor Spirin (victor.spirin@cs.ox.ac.uk)
 *
 *     This file is part of MRESim 2.2, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our paper:
 *
 *     @inproceedings{deHoog2009,
 *         title = "Role-Based Autonomous Multi-Robot Exploration",
 *         author = "Julian de Hoog, Stephen Cameron and Arnoud Visser",
 *         year = "2009",
 *         booktitle = "International Conference on Advanced Cognitive Technologies and Applications (COGNITIVE)",
 *         location = "Athens, Greece",
 *         month = "November",
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

import communication.DataMessage;
import exploration.*;
import environment.*;
import config.*;

import config.RobotConfig.roletype;
import exploration.rendezvous.IRendezvousStrategy;
import exploration.rendezvous.RendezvousAgentData;
import exploration.rendezvous.SinglePointRendezvousStrategy;
import exploration.rendezvous.SinglePointRendezvousStrategySettings;
import java.util.*;
import java.awt.*;
import path.Path;


/**
 *
 * @author Julian de Hoog
 */


public class RealAgent extends BasicAgent implements Agent {

// <editor-fold defaultstate="collapsed" desc="Class variables and Constructors">    
    // Data
    int timeLastCentralCommand;   /* units of time elapsed since command 
                                     received from ComStation */
    double distanceTraveled;      // Distance traveled
    int areaKnown;                // How much area this agent knows about
    int areaGoal;                 // How much free space the agent needs to explore in this mission
    int lastContactAreaKnown;     // How much area this agent knew at the time of last contact with base
    int newInfo;                  // How much area that we know we think is not known at base station
    int prevX, prevY;             // Previous position in environment
    public int timeLastDirectContactCS; // time since last contact with base station
    public int periodicReturnInterval;   // how long to wait until going back to BS
    public int frontierPeriodicState;   // bit of a hack: state of periodic return frontier exp robots (0=exploring; 1=returning)
    int timeElapsed;
    
    int relayID; //id of the agent acting as relay for this agent

    boolean envError;             // set to true by env when RealAgent's step is not legal
    public int timeSinceLastPlan;        // time passed since last plan was made
    
    boolean needUpdatingAreaKnown;

    // Occupancy Grid
    OccupancyGrid occGrid;
    LinkedList<Point> dirtyCells;   /* List of cells changed since last step
                                       (For faster update of image) */

    public LinkedList<Point> pathTaken;    // For display where robot has gone
    
    // Frontiers
    PriorityQueue<Frontier> frontiers;
    Frontier lastFrontier;          // Keep track of last frontier of interest
    
    //Frontiers that are impossible to reach, so should be discarded
    HashMap<Frontier, Boolean> badFrontiers;
    
    int relayMarks;

    // Path
    Path path;
    Path pathToBase;
    
    // Teammates
    HashMap<Integer,TeammateAgent> teammates;
    
    double maxRateOfInfoGatheringBelief;
    double currentTotalKnowledgeBelief;    
    double currentBaseKnowledgeBelief;  
    
    // Role-based Exploration
    private RendezvousAgentData rendezvousAgentData;
    private IRendezvousStrategy rendezvousStrategy;
    
    private boolean missionComplete; // true only when mission complete (env fully explored)
    private double percentageKnown;

    // rendezvous Points and Skeleton
    
    
    private final TopologicalMap topologicalMap;
    int timeTopologicalMapUpdated;
    

    // dynamic behavior
    private Point currentGoal;  // needed for calculating dynamic role switch
    
    private SimulatorConfig simConfig;

    
    public RealAgent(int envWidth, int envHeight, RobotConfig robot) {
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
              Constants.DEFAULT_SPEED);

        timeLastCentralCommand = 0;
        distanceTraveled = 0;
        areaKnown = 0;
        newInfo = 0;
        lastContactAreaKnown = 0;
        prevX = x;
        prevY = y;
        timeLastDirectContactCS = 0;
        periodicReturnInterval = 10;
        frontierPeriodicState = 0;
        envError = false;
        timeSinceLastPlan = 0;
        timeTopologicalMapUpdated = -1;
        timeElapsed = 0;
        relayID = -1;
        
        maxRateOfInfoGatheringBelief = 0;
        currentTotalKnowledgeBelief = 0;
        currentBaseKnowledgeBelief = 0;
        

        occGrid = new OccupancyGrid(envWidth, envHeight);
        topologicalMap = new TopologicalMap(null);
        dirtyCells = new LinkedList<Point>();
        pathTaken = new LinkedList<Point>();
        badFrontiers = new HashMap<Frontier, Boolean>();
        
        
        frontiers = new PriorityQueue();
        
        path = new Path();
        
        teammates = new HashMap<Integer,TeammateAgent>();

        setState(RealAgent.ExploreState.Initial);
        
        missionComplete = false;
        
        rendezvousAgentData = new RendezvousAgentData(this);
        SinglePointRendezvousStrategySettings rvSettings = new SinglePointRendezvousStrategySettings();
        rvSettings.allowReplanning = simConfig.replanningAllowed();
        rvSettings.useImprovedRendezvous = simConfig.useImprovedRendezvous();
        rendezvousStrategy = new SinglePointRendezvousStrategy(this, rvSettings);
        
        currentGoal = new Point(x,y);
    }
  
// </editor-fold>     

    
// <editor-fold defaultstate="collapsed" desc="Get and Set">
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
    
    public int getTimeLastCentralCommand() {
        return this.timeLastCentralCommand;
    }
    
    public void incrementTimeLastCentralCommand() {
        timeLastCentralCommand++;
    }
    
    public double getDistanceTraveled() {
        return this.distanceTraveled;
    }     
    
    public int getAreaKnown() {
        return this.areaKnown;
    }
    
    public int getNewInfo() {
        return this.newInfo;
    }
    
    public int getLastContactAreaKnown() {
        return this.lastContactAreaKnown;
    }
    
    public void setLastContactAreaKnown(int val) {
        this.lastContactAreaKnown = val;
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

    public int getTimeSinceLastPlan() {
        return timeSinceLastPlan;
    }

    public void setTimeSinceLastPlan(int t) {
        timeSinceLastPlan = t;
    }

    public OccupancyGrid getOccupancyGrid() {
        return occGrid;
    }
    
    public LinkedList<Point> getDirtyCells() {
        return dirtyCells;
    }

    public void setDirtyCells(LinkedList<Point> list) {
        dirtyCells = list;
    }
    
    public void addDirtyCells(LinkedList<Point> newDirt) {
        setDirtyCells(mergeLists(getDirtyCells(), newDirt));
    }

    public PriorityQueue<Frontier> getFrontiers() {
        return(this.frontiers);
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
    
    public void updatePathDirt(Path p)
    {
        java.util.List pts = p.getPoints();
        if (pts != null)
        {
            for(int i=0; i<pts.size()-1; i++)
            {
                addDirtyCells(pointsAlongSegment(((Point)pts.get(i)).x, ((Point)pts.get(i)).y, ((Point)pts.get(i+1)).x, ((Point)pts.get(i+1)).y));
            }
        }
    }
    
    public Path getPath() {        
        return path;
    }
    
    public void setPath(Path newPath) {
        this.setDirtyCells(mergeLists(dirtyCells, path.getAllPathPixels()));
        path = newPath;
    }

    public Point getNextPathPoint() {
        if(path!=null)
            if(path.getPoints()!=null)
                if(!path.getPoints().isEmpty())
                    return((Point)path.getPoints().remove(0));

        return null;
    }

    public void setPathToBaseStation() {
        setPath(getPathToBaseStation());
    }
    
    public void resetPathToBaseStation() {
        pathToBase = null;
    }
    
    public void computePathToBaseStation()
    {
        long realtimeStartAgentCycle = System.currentTimeMillis();
        pathToBase = calculatePath(getLocation(), getTeammate(1).getLocation());
        System.out.println(this.toString() + "Path to base computation took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
    }
    
    public Path getPathToBaseStation() {
        if ((pathToBase != null) && ((pathToBase.getPoints() == null) || pathToBase.getPoints().isEmpty()))
            pathToBase = null;
        if ((pathToBase == null) || (pathToBase.getStartPoint().distance(this.getLocation()) > (Constants.DEFAULT_SPEED * 2))) 
            computePathToBaseStation();
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
    
    public TopologicalMap getTopologicalMap()
    {
        return topologicalMap;
    }
    
    public double getMaxRateOfInfoGatheringBelief()
    {
        return maxRateOfInfoGatheringBelief;
    }
    
    public void setMaxRateOfInfoGatheringBelief(double rate)
    {
        maxRateOfInfoGatheringBelief = rate;
    }
    
    public double getCurrentTotalKnowledgeBelief()
    {
        return currentTotalKnowledgeBelief;
    }
    
    // used in utility exploration
    // returns: how much information/utility we assume the base station will already know
    // by the time we deliver out information.
    public double getCurrentBaseKnowledgeBelief()
    {
        return currentBaseKnowledgeBelief;        
    }
    
    //This method checks if occupancy grid has changed since last update of topological map,
    //and rebuilds the map if necessary
    public void forceUpdateTopologicalMap(boolean mustUpdate)
    {
        if (occGrid.hasMapChanged() || mustUpdate) {
            System.out.println(this + " Updating topological map");
            long timeStart = System.currentTimeMillis();
            topologicalMap.setGrid(occGrid);   
            System.out.println(toString() + "setGrid, " + (System.currentTimeMillis()-timeStart) + "ms.");
            topologicalMap.generateSkeleton();
            System.out.println(toString() + "generateSkeleton, " + (System.currentTimeMillis()-timeStart) + "ms.");
            topologicalMap.findKeyPoints();
            System.out.println(toString() + "findKeyPoints, " + (System.currentTimeMillis()-timeStart) + "ms.");
            topologicalMap.generateKeyAreas();
            timeTopologicalMapUpdated = timeElapsed;
            System.out.println(toString() + "generated topological map, " + (System.currentTimeMillis()-timeStart) + "ms.");
            occGrid.setMapHasChangedToFalse();
        } else {
            System.out.println(this + " Occupancy Grid not changed since last update, skipping topological map update");
        }
    }

    public RendezvousAgentData getRendezvousAgentData() {
        return rendezvousAgentData;
    }
    
    public IRendezvousStrategy getRendezvousStrategy() {
        return rendezvousStrategy;
    }
// </editor-fold>    
    
// <editor-fold defaultstate="collapsed" desc="Parent and Child">
    public TeammateAgent getParentTeammate() {
        return getTeammate(parent);
    }
    
    public void setParent(int p){
        parent = p;
    }
 
    public TeammateAgent getChildTeammate() {
        return getTeammate(child);
    }
    
    public void setChild(int c){
        child = c;
    }

    public boolean isExplorer() {
        return (role==roletype.Explorer);
    }
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Teammates">
    
    public void addTeammate(TeammateAgent teammate) {
        teammates.put(teammate.getRobotNumber(), teammate);
    }
    
    public TeammateAgent getTeammate(int n) {
        return teammates.get(n);
    }

    // necessary when swapping roles with another agent
    public TeammateAgent removeTeammate(int n) {
        return teammates.remove(n);
    }
    
    public HashMap<Integer, TeammateAgent> getAllTeammates() {
        return teammates;
    }
    
// </editor-fold>     
    
// <editor-fold defaultstate="collapsed" desc="Flush, take step, write step">

    // Overwrite any useless data from previous step
    public void flush() {
        prevX = x;
        prevY = y;
        for(TeammateAgent teammate : teammates.values())
            teammate.setInRange(false);

        // Only for testing, uncommenting the line below leads to massive slow down
        //occGrid.initializeTestBits();
    }

    public Point takeStep(int timeElapsed, SimulatorConfig simConfig) {
        this.simConfig = simConfig;
        long realtimeStartAgentStep = System.currentTimeMillis(); 
        Point nextStep = new Point(0,0);
        
        //previous time elapsed, used to check if we advanced to a new time cycle or are still in the old one
        int oldTimeElapsed = this.timeElapsed; 
        this.timeElapsed = timeElapsed;
        
        //shall we go out of service?
        //if (Math.random() < Constants.PROB_OUT_OF_SERVICE)
        //if ((timeElapsed > (robotNumber * 150)) && (robotNumber > 0))
        //    setState(ExploreState.OutOfService);
        
        //if we are out of service, don't move, act as relay
        if (getState() == ExploreState.OutOfService)
        {
            setSpeed(0);
            return getLocation();                
        }
        //System.out.println(this.toString() + "I am at location " + this.x + "," + this.y + ". Taking step ... ");
        
        if (timeElapsed == 0) oldTimeElapsed = -1; //hack for initial time step
        if (oldTimeElapsed != timeElapsed)
        {
            setDistanceToBase(getPathToBaseStation().getLength());
            switch(simConfig.getExpAlgorithm()) {
                case RunFromLog:             nextStep = RunFromLog.takeStep(timeElapsed, simConfig.getRunFromLogFilename(), this.robotNumber);
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
                                             if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.ReturnWhenComplete))
                                                nextStep = FrontierExploration.takeStep(this, 
                                                        timeElapsed, simConfig.getFrontierAlgorithm());
                                             if (simConfig.getFrontierAlgorithm().equals(SimulatorConfig.frontiertype.UtilReturn))
                                                nextStep = UtilityExploration.takeStep(this, timeElapsed, simConfig);
                                             break;
                case RoleBasedExploration:   
                                             nextStep = RoleBasedExploration.takeStep(this, timeElapsed, rendezvousStrategy);
                                             break;
                default:                     break;
            }
        } else
        {            
            switch(simConfig.getExpAlgorithm()) {
                case RunFromLog:             break;
                case LeaderFollower:         nextStep = this.getNextPathPoint();
                                             break;
                case FrontierExploration:    nextStep = this.getNextPathPoint();
                                             break;
                case RoleBasedExploration:   if ((this.getState() != ExploreState.GiveParentInfo)
                        && (this.getState() != ExploreState.GetInfoFromChild) && (this.getState() != ExploreState.WaitForChild)
                        && (this.getState() != ExploreState.WaitForParent))
                                                nextStep = this.getNextPathPoint();
                                             break;
                default:                     break;
            }
        }
        
        //pruneUnexploredSpace();
        
        System.out.println(this.toString() +  "Taking step complete, moving from " + getLocation() + " to " + nextStep + ", took " + (System.currentTimeMillis()-realtimeStartAgentStep) + "ms.");

        return nextStep;
    }
    
    public void writeStep(Point nextLoc, double[] sensorData) {
        writeStep(nextLoc, sensorData, true);
    }
    
    public void writeStep(Point nextLoc, double[] sensorData, boolean updateSensorData) {
        long realtimeStart = System.currentTimeMillis();
        //System.out.print(this.toString() + "Writing step ... ");
        
        int safeRange = (int)(sensRange * Constants.SAFE_RANGE / 100);
        Polygon newFreeSpace, newSafeSpace;


        x = nextLoc.x;
        y = nextLoc.y;

        pathTaken.add(new Point(x,y));
        
        if(!(prevX == x  &&  prevY == y))
            heading = Math.atan2(y - prevY, x - prevX);
        distanceTraveled += Math.sqrt(Math.pow(y-prevY,2) + Math.pow(x-prevX,2));
        //System.out.println("Agent " + this.ID + " distance travelled: " + distanceTraveled);

        // OLD METHOD RAY TRACING
        // Safe space slightly narrower than free space to make sure frontiers
        // are created along farthest edges of sensor radial polygon
        if (updateSensorData) {
            realtimeStart = System.currentTimeMillis();
            newFreeSpace = findRadialPolygon(sensorData, sensRange, 0, 180);
            newSafeSpace = findRadialPolygon(sensorData, safeRange, 0, 180);
            //System.out.println(this.toString() + "findRadialPolygon(x2) took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
            realtimeStart = System.currentTimeMillis();
            updateObstacles(newFreeSpace);
            //System.out.println(this.toString() + "updateObstacles took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
            realtimeStart = System.currentTimeMillis();
            updateFreeAndSafeSpace(newFreeSpace, newSafeSpace);
            //System.out.println(this.toString() + "updateFreeAndSafeSpace took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        }
        
        // NEW METHOD FLOOD FILL
        //updateGrid(sensorData);

        //batteryPower--;
        batteryPower = newInfo;
        //timeLastCentralCommand++;
        
        //System.out.println(this.toString() + "WriteStep complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
    
// </editor-fold>     
    
// <editor-fold defaultstate="collapsed" desc="Calculate paths">
    public Path calculatePath(Point startPoint, Point goalPoint)
    {
        return calculatePath(startPoint, goalPoint, false);
    }
    public Path calculatePath(Point startPoint, Point goalPoint, boolean pureAStar)
    {
        
        //topologicalMap = new TopologicalMap(occGrid);
        int rebuild_topological_map_interval = Constants.REBUILD_TOPOLOGICAL_MAP_INTERVAL;
        if (timeTopologicalMapUpdated < 0) timeTopologicalMapUpdated = timeElapsed - rebuild_topological_map_interval;
        if (timeElapsed - timeTopologicalMapUpdated >= rebuild_topological_map_interval)
        {
            forceUpdateTopologicalMap(false);
        }
        if (timeElapsed - timeTopologicalMapUpdated >= Constants.MUST_REBUILD_TOPOLOGICAL_MAP_INTERVAL)
        {
            forceUpdateTopologicalMap(true);
        }
        topologicalMap.setPathStart(startPoint);
        topologicalMap.setPathGoal(goalPoint);
        if (!pureAStar)
        {
            //System.out.println(this + "calculating topological path from " + startPoint + " to " + goalPoint);
            topologicalMap.getTopologicalPath();
        } else
        {
            //System.out.println(this + "calculating jump path from " + startPoint + " to " + goalPoint);
            topologicalMap.getJumpPath();
            if (topologicalMap.getPath() == null)
                System.out.println("!!!! CATASTROPHIC FAILURE !!!!!");
        }
        return topologicalMap.getPath();
        //return new Path(occGrid, startPoint, goalPoint, false);
    }
    
    private LinkedList<Point> pointsAlongSegment(int x1, int y1, int x2, int y2) {
        LinkedList<Point> pts = new LinkedList<Point>();
        
        for(int i=Math.min(x1, x2); i<=Math.max(x1, x2); i++)
            for(int j=Math.min(y1, y2); j<=Math.max(y1, y2); j++)
                if(occGrid.distPointToLine(x1, y1, x2, y2, i, j) < 0.5)
                    pts.add(new Point(i,j));
                   
        return pts;
    }
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Updating">
    
    // <editor-fold defaultstate="collapsed" desc="DELETE">
    /*public boolean isNeedUpdatePaths()
    {
        return ((areaKnown != prevAreaKnown) || (newInfo != prevNewInfo) || 
                ((getState() == ExploreState.ReturnToParent) && (newInfo == 0)));
    }
    
    public boolean isNeedUpdateAreaKnown()
    {
        return needUpdatingAreaKnown;
    }
    
    public double getLastTotalKnowledgeBelief()
    {
        return lastTotalKnowledgeBelief;
    }
    
    public void setLastTotalKnowledgeBelief(double val)
    {
        lastTotalKnowledgeBelief = val;
    }
    
    public double getLastBaseKnowledgeBelief()
    {
        return lastBaseKnowledgeBelief;        
    }
    
    public void setLastBaseKnowledgeBelief(double val)
    {
        lastBaseKnowledgeBelief = val;        
    }
    
    public int getLastNewInfo()
    {
        return lastNewInfo;        
    }
    
    public void setLastNewInfo(int val)
    {
        lastNewInfo = val;        
    }
    */
    // </editor-fold>
    
    
    
    public double getPercentageKnown ()
    {
        return percentageKnown;
    }
    
    public void setGoalArea(int goalArea)
    {
        areaGoal = goalArea;
    }
    
    // update stats of what we know about the environment
    // TODO: we shouldn't call this every time step, this is a performance bottleneck and can be made more efficient.
    public void updateAreaKnown () {
        //<editor-fold defaultstate="collapsed" desc="Commented out - "brute force" way of calculating map stats">
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
        
        areaKnown = occGrid.getNumFreeCells();
        newInfo = (occGrid.getNumFreeCells() - occGrid.getNumFreeCellsKnownAtBase() - occGrid.getNumFreeRelayedCells());
        percentageKnown = (double)areaKnown / (double)areaGoal;
        
        currentBaseKnowledgeBelief = occGrid.getNumFreeCellsKnownAtBase() + occGrid.getNumFreeRelayedCells(); 
        // ^^^ can add them up, as they are disjoint;
        // may be a good idea to add a discounted value for gotRelayed, as we are not sure it is going to be delivered
        // to base soon. The reason we incorporate gotRelayed to reduce the probability of agents trying to go back to base
        // before the ratio is hit, and then having to go back to exploring when they learn that base station knows more
        // information than they thought, resulting in wasted effort.
        currentTotalKnowledgeBelief = getCurrentBaseKnowledgeBelief() + newInfo * (teammates.size()-1);
        
        if (timeElapsed > 0)
        {
            double rateOfInfoGatheringBelief = (double)areaKnown / (double)timeElapsed;
            if (rateOfInfoGatheringBelief > maxRateOfInfoGatheringBelief)
                maxRateOfInfoGatheringBelief = rateOfInfoGatheringBelief;
        }
    }    
    
    // TODO: Should be able to make this more efficient.
    public void updateAreaRelayed (TeammateAgent ag) {
        //long timer = System.currentTimeMillis();
        if (ag.robotNumber == robotNumber) //we are the same as ag, nothing to do here
            return;
        if (getID() == Constants.BASE_STATION_TEAMMATE_ID) //base station
            return;
        int new_counter = 0;
        boolean iAmCloserToBase = (ag.timeToBase() > timeToBase());
        /*if (iAmCloserToBase)
            System.out.println(toString() + " relaying for " + ag.name + " (" + timeToBase() + " vs " + ag.timeToBase() + ")");
        else
            System.out.println(ag.name + " relaying for " + toString() + " (" + ag.timeToBase() + " vs " + timeToBase() + ")");
        */
        if (iAmCloserToBase) {           
            for (Point point : ag.occGrid.getOwnedCells()) {
                if (occGrid.isGotRelayed(point.x, point.y)) {
                    occGrid.setGotUnrelayed(point.x, point.y);
                    new_counter++;
                }
            }
           if (new_counter > 0) {
               if (newInfo == 0) newInfo = 1;
               System.out.println(toString() + "setGotUnrelayed: " + new_counter);
           }
        } else {        
            if (newInfo > 0) {
                // Need to iterate over a copy of getOwnedCells list, as the list gets changed by setGotRelayed.
                new_counter = occGrid.setOwnedCellsRelayed();
                if (new_counter > 0)
                    System.out.println(toString() + "setGotRelayed: " + new_counter);
            }
        }
        //System.out.println(this.toString() + "updateAreaRelayed took " + (System.currentTimeMillis()-timer) + "ms.\n");
    }
    
    protected void updateGrid(double sensorData[]) {
        Point first, second;
        double dist, angle;
        int currX, currY;
        double currRayAngle;
        int currRayX, currRayY;
        
         //If there's no sensor data, done
        if(sensorData == null)
            return;
    
        int safeRange = (int)(sensRange * Constants.SAFE_RANGE / 100);
       
        // To create a polygon, add robot at start and end
        Polygon polygon = new Polygon();
        polygon.addPoint(x, y);

        
        //For every degree
        for(int i=0; i<=180; i+=1) {
            currRayAngle = heading - Math.PI/2 + Math.PI/180*i;
            
            if(sensorData[i] >= sensRange) {
                currRayX = x + (int)Math.round(sensRange*(Math.cos(currRayAngle)));
                currRayY = y + (int)Math.round(sensRange*(Math.sin(currRayAngle)));
            }
            else {
                currRayX = x + (int)Math.round(sensorData[i]*(Math.cos(currRayAngle)));
                currRayY = y + (int)Math.round(sensorData[i]*(Math.sin(currRayAngle)));
            }
            
            polygon.addPoint(currRayX, currRayY);
        }

        polygon.addPoint(x, y);



        // determine mins and maxes of polygon
        int xmin = polygon.getBounds().x;
        int ymin = polygon.getBounds().y;

        // Create temp grid 
        int[][] tempGrid = new int[polygon.getBounds().width+1][polygon.getBounds().height+1];
        for(int i=0; i<polygon.getBounds().width+1; i++)
            for(int j=0; j<polygon.getBounds().height+1; j++)
                tempGrid[i][j]=0;


        /*********************************************/
        //   I   Make outline of polygon

        // go through all points in scan, set lines between them to obstacle
        for(int i=0; i<polygon.npoints-1; i++) {
            first = new Point(polygon.xpoints[i], polygon.ypoints[i]);
            second = new Point(polygon.xpoints[i+1], polygon.ypoints[i+1]);
            dist = first.distance(second);
            angle = Math.atan2(second.y-first.y, second.x-first.x);
            for(int j=0; j<dist; j++) {
                currX = (int)(first.x + j*Math.cos(angle));
                currY = (int)(first.y + j*Math.sin(angle));
                tempGrid[currX-xmin][currY-ymin] = 1;
           }
        }


        /*********************************************/
        //   II  Fill free space from inside

        // first, create

        LinkedList<Point> toFill = new LinkedList<Point>();
        Point start = new Point((int)(x + 1*Math.cos(heading) - xmin), (int)(y + 1*Math.sin(heading) - ymin));

        // if start is obstacle (robot right in front of wall), don't bother updating anything
        if(occGrid.obstacleAt(start.x, start.y))
            return;

        Point curr;
        toFill.add(start);

        while(!toFill.isEmpty()) {
            curr = toFill.pop();
            tempGrid[curr.x][curr.y] = 2;

            // add neighbours
            if(curr.x-1 >= 0  &&  tempGrid[curr.x-1][curr.y]==0) {
                tempGrid[curr.x-1][curr.y] = -1;
                toFill.add(new Point(curr.x-1,curr.y));
            }
            if(curr.x+1 <= polygon.getBounds().width  &&  tempGrid[curr.x+1][curr.y]==0) {
                tempGrid[curr.x+1][curr.y] = -1;
                toFill.add(new Point(curr.x+1,curr.y));
            }
            if(curr.y-1 >= 0  &&  tempGrid[curr.x][curr.y-1]==0) {
                tempGrid[curr.x][curr.y-1] = -1;
                toFill.add(new Point(curr.x,curr.y-1));
            }
            if(curr.y+1 <= polygon.getBounds().height  &&  tempGrid[curr.x][curr.y+1]==0) {
                tempGrid[curr.x][curr.y+1] = -1;
                toFill.add(new Point(curr.x,curr.y+1));
            }
        }

        /*********************************************/
        //   III Fill in obstacles

        // go through all points in polygon (avoid first and last because it's the robot)
        for(int i=1; i<polygon.npoints-2; i++) {
            first = new Point(polygon.xpoints[i], polygon.ypoints[i]);
            second = new Point(polygon.xpoints[i+1], polygon.ypoints[i+1]);

		// if they are close enough, fill line between them
            if(first.distance(x,y) < (sensRange-2) && second.distance(x,y) < (sensRange-2) &&
               first.distance(second) < 5) {
                angle = Math.atan2(second.y-first.y, second.x-first.x);
                for(double j=0; j<first.distance(second); j++) {
                            currX = (int)(first.x + j*Math.cos(angle));
                            currY = (int)(first.y + j*Math.sin(angle));
                            tempGrid[currX-xmin][currY-ymin] = 4;
                        }
             }
        }


        /*********************************************/
        //   IV  Update real grid

        for(int i=0; i<polygon.getBounds().width; i++)
            for(int j=0; j<polygon.getBounds().height; j++) {
                if(tempGrid[i][j]==4) {
                    occGrid.setObstacleAt(i+xmin,j+ymin);
                    dirtyCells.add(new Point(i+xmin,j+ymin));
                }
                else if(tempGrid[i][j]==2) {
                    if((new Point(x,y)).distance(new Point(i+xmin,j+ymin)) < safeRange) { // &&
                        //angleDiff(Math.atan2((j+ymin)-y, (i+xmin)-x), heading) < 80)
                        occGrid.setSafeSpaceAt(i+xmin,j+ymin);
                        dirtyCells.add(new Point(i+xmin,j+ymin));
                    }
                    else {
                        occGrid.setFreeSpaceAt(i+xmin,j+ymin);
                        dirtyCells.add(new Point(i+xmin,j+ymin));

                    }
                }
            }        
    }

    protected int angleDiff(double theta1, double theta2){
        //System.out.println(theta1 + " " + theta2);
	    int angle1 = (int)(180/Math.PI*theta1 + 360)%360;
	    int angle2 = (int)(180/Math.PI*theta2 + 360)%360;
        int diff = Math.abs(angle1-angle2);
        return(Math.min(diff, 360-diff));
	}
    
    protected void updateFreeAndSafeSpace(Polygon newFreeSpace, Polygon newSafeSpace) {
        // May be possible to do some optimization here, I think a lot of cells are checked unnecessarily
        for(int i=newFreeSpace.getBounds().x; i<=newFreeSpace.getBounds().x+newFreeSpace.getBounds().width; i++)
            innerloop:
            for(int j=newFreeSpace.getBounds().y; j<=newFreeSpace.getBounds().y+newFreeSpace.getBounds().height; j++)
                if(occGrid.locationExists(i, j)) {
                    if(newFreeSpace.contains(i,j) && !occGrid.freeSpaceAt(i,j)) {
                       occGrid.setFreeSpaceAt(i, j);
                       dirtyCells.add(new Point(i,j));
                    }
                    if(newSafeSpace.contains(i,j)){
                        // double for loop to prevent empty-safe boundary (which
                        // would not qualify as a frontier)
                        for(int m=i-1; m<=i+1; m++)
                            for(int n=j-1; n<=j+1; n++)
                                if(occGrid.locationExists(m,n) && occGrid.emptyAt(m,n))
                                    continue innerloop;
                        if(!occGrid.safeSpaceAt(i,j)) {                             
                            occGrid.setSafeSpaceAt(i, j);
                            dirtyCells.add(new Point(i,j));
                        }
                    }
                }
    }
    
    protected void updateObstacles(Polygon newFreeSpace) {
        // Update obstacles -- all those bits for which radial polygon < sensRange
        // Ignore first point in radial polygon as this is the robot itself.
        Point first, second;
        double angle;
        int currX, currY;
        
        for(int i=1; i<newFreeSpace.npoints-1; i++) {
            first = new Point(newFreeSpace.xpoints[i], newFreeSpace.ypoints[i]);
            second = new Point(newFreeSpace.xpoints[i+1], newFreeSpace.ypoints[i+1]);

            // if two subsequent points are close and both hit an obstacle, assume there is a line between them.
            if(first.distance(x,y) < (sensRange-2) && second.distance(x,y) < (sensRange-2) &&
               first.distance(second) < 5) { // used to be 10
                angle = Math.atan2(second.y - first.y, second.x - first.x);                
                for(int j=0; j<first.distance(second); j++) {
                    currX = first.x + (int)(j * (Math.cos(angle)));
                    currY = first.y + (int)(j * (Math.sin(angle)));
                    double angleDepth = Math.atan2(currY - y, currX - x);
                    for (int k = 0; k < Constants.WALL_THICKNESS; k++)
                        occGrid.setObstacleAt(currX + (int)(k * Math.cos(angleDepth)), currY + (int)(k * Math.sin(angleDepth)));
                    dirtyCells.add(new Point(currX, currY));
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
        if(sensorData == null)
            return radialPolygon;
    
        //For every degree
        for(int i=startAngle; i<=finishAngle; i+=1) {
            currRayAngle = heading - Math.PI/2 + Math.PI/180*i;
            
            if(sensorData[i] >= maxRange) {
                currRayX = x + (int)Math.round(maxRange*(Math.cos(currRayAngle)));
                currRayY = y + (int)Math.round(maxRange*(Math.sin(currRayAngle)));
            }
            else {
                currRayX = x + (int)Math.round(sensorData[i]*(Math.cos(currRayAngle)));
                currRayY = y + (int)Math.round(sensorData[i]*(Math.sin(currRayAngle)));
            }
            
            radialPolygon.addPoint(currRayX, currRayY);
        }
       
        
        return radialPolygon;
    }

    // </editor-fold>     
    
// <editor-fold defaultstate="collapsed" desc="Communicate">

    public void receiveMessage(DataMessage msg) {
        TeammateAgent teammate = teammates.get(msg.ID);
        
        msg.receiveMessage(this, teammate);
        
        boolean isBaseStation = false;
        if ((teammate.getRobotNumber() == Constants.BASE_STATION_TEAMMATE_ID) || 
                (this.getRobotNumber() == Constants.BASE_STATION_TEAMMATE_ID))
            isBaseStation = true;
        
        //merge the occupancy grids, and add affected cells to dirty cell list to be repainted in the GUI
        dirtyCells.addAll(
                occGrid.mergeGrid(teammate.getOccupancyGrid(), isBaseStation));
        
        if ((simConfig != null) && (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.FrontierExploration)
                && (simConfig.getFrontierAlgorithm() == SimulatorConfig.frontiertype.UtilReturn))
            updateAreaRelayed(teammate);
        
        needUpdatingAreaKnown = true;
    }
    
    public boolean isCommunicating() {
        for(TeammateAgent teammate : teammates.values())
            if(teammate.isInRange())
                return true;
        return false;
    }
    

    public void updateAfterCommunication() {
        //long realtimeStart = System.currentTimeMillis();
        //System.out.print(this.toString() + "Updating post-communication ... ");
        
        for(TeammateAgent teammate : teammates.values())
            if(!teammate.isInRange())
                teammate.setTimeSinceLastComm(teammate.getTimeSinceLastComm()+1);
        
        //processRelayMarks();
        //System.out.println("Complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
// </editor-fold>     

}
