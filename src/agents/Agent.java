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

import config.Constants;
import config.RobotConfig;
import java.awt.Point;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

abstract public class Agent {

    int robotNumber;        // Robot's number.  CONSTANT.  Does not change throughout run.
    String name;            // Name
    int ID;
    /* Different from Robot Number, ID can change if robots swap roles,
                               and is used by robots to identify parent / child  */

    int x, y;               // Current position
    double heading;         // Current heading in radians
    int speed;

    int sensRange;          // Sensing range (radius)
    int commRange;          // Communication range (radius)
    int batteryPower;
    int energyCunsumption;
    public int ability;
    public ArrayList<ComStation> comStations;
    private int comStationLimit;

    double distanceToBase;

    RobotConfig.roletype role;

    public static enum AgentState {
        Initial, Explore, ReturnToBaseStation, WaitForParent, GiveParentInfo,
        GoToChild, WaitForChild, GetInfoFromChild, OutOfService, RELAY, INACTIVE, OCCUPIED, AKTIVE
    }

    public static enum ExplorationState {
        Initial, Explore, ReturnToBaseStation, WaitForParent, GiveParentInfo,
        GoToChild, WaitForChild, GetInfoFromChild, OutOfService
    }
    private AgentState state;
    private ExplorationState prevExploreState = ExplorationState.Initial;
    private ExplorationState exploreState = ExplorationState.Initial;

    int parent;             // Should keep ID and NOT RobotNumber of the parent
    int child;              // Should keep ID and NOT RobotNumber of the child

    private int stateTimer;  // keeps track of how much time spent in this state
    private int timeSinceGetChildInfo; //how long since we got child info

    // Keeps historical data about agent knowledge
    public Map<Integer, Double> knowledgeData = new HashMap();

    public Agent(RobotConfig robot) {
        robotNumber = robot.getRobotNumber();
        name = robot.getName();
        ID = robot.getRobotNumber();
        x = robot.getStartX();
        y = robot.getStartY();
        heading = robot.getStartHeading();
        sensRange = robot.getSensingRange();
        commRange = robot.getCommRange();
        batteryPower = robot.getBatteryLife();
        role = robot.getRole();
        state = AgentState.Initial;
        parent = robot.getParent();
        child = robot.getChild();
        speed = robot.getSpeed();
        ability = robot.getAbility();
        timeSinceGetChildInfo = 0;
        comStations = new ArrayList<>();
        this.comStationLimit = robot.getComStationLimit();
    }

    public int getAbility() {
        return ability;
    }

    public int getRobotNumber() {
        return this.robotNumber;
    }

    public void setRobotNumber(int newNo) {
        this.robotNumber = newNo;
    }

    public String getName() {
        return this.name;
    }

    public void setName(String n) {
        this.name = n;
    }

    public int getComStationLimit() {
        return comStationLimit;
    }

    public void setComStationLimit(int comStationLimit) {
        this.comStationLimit = comStationLimit;
    }

    public void setSpeed(int sp) {
        this.speed = sp;
    }

    public int getSpeed() {
        return this.speed;
    }

    public int getID() {
        return this.ID;
    }

    public void setID(int newID) {
        this.ID = newID;
    }

    public int getSenseRange() {
        return sensRange;
    }

    public int getCommRange() {
        return commRange;
    }

    public int getBatteryPower() {
        return this.batteryPower;
    }

    public void setBatteryPower(int power) {
        this.batteryPower = power;
    }

    public AgentState getState() {
        return state;
    }

    final public void setState(AgentState state) {
        this.state = state;
    }

    public ExplorationState getPrevExploreState() {
        return prevExploreState;
    }

    public ExplorationState getExploreState() {
        return exploreState;
    }

    public final void setExploreState(ExplorationState s) {
        if (this.exploreState != s) {
            prevExploreState = this.exploreState;
            this.setStateTimer(0);
            this.exploreState = s;
        }
    }

    public int getStateTimer() {
        return stateTimer;
    }

    public void setStateTimer(int t) {
        if (t != 0) {
            timeSinceGetChildInfo++;
        }
        stateTimer = t;
    }

    public int getX() {
        return this.x;
    }

    public void setX(int newx) {
        this.x = newx;
    }

    public int getY() {
        return this.y;
    }

    public void setY(int newy) {
        this.y = newy;
    }

    public Point getLocation() {
        return new Point(x, y);
    }

    /**
     * Gives the current heading of the agent.
     *
     * @return heading in radians
     */
    public double getHeading() {
        return this.heading;
    }

    /**
     * Sets the agents current heading. Will be corrected to a value between 0 and 2 Math.PI
     *
     * @param newHeading heading in radians
     */
    public void setHeading(double newHeading) {
        if (newHeading >= 2 * Math.PI) {
            setHeading(newHeading - 2 * Math.PI);
        }
        if (newHeading < 0) {
            setHeading(newHeading + 2 * Math.PI);
        }
        this.heading = newHeading;
    }

    public int getEnergyCunsumption() {
        return energyCunsumption;
    }

    public void setEnergyCunsumption(int energyCunsumption) {
        this.energyCunsumption = energyCunsumption;
    }

    public RobotConfig.roletype getExploreMode() {
        return this.role;
    }

    public void setRole(RobotConfig.roletype newR) {
        this.role = newR;
    }

    public void setRole(String newR) {
        this.role = RobotConfig.roletype.valueOf(newR);
    }

    public RobotConfig.roletype getRole() {
        return role;
    }

    public int getParent() {
        return parent;
    }

    public int getChild() {
        return child;
    }

    public int getTimeSinceGetChildInfo() {
        return timeSinceGetChildInfo;
    }

    public void setTimeSinceGetChildInfo(int val) {
        timeSinceGetChildInfo = val;
    }

    public void addComStation(ComStation com) {
        if (comStationLimit > comStations.size()) {
            this.comStations.add(com);
        }
    }

    public ComStation giveComStation() {
        if (this.comStations.size() > 0) {
            return this.comStations.remove(0);
        } else {
            return null;
        }
    }

    public boolean takeComStation(ComStation comstation) {
        if (comstation != null) {
            this.addComStation(comstation);
            return true;
        } else {
            return false;
        }
    }

    public ArrayList<ComStation> getComStations() {
        return comStations;
    }

// Utility Functions
    //Adds all points in list2 to list1 (no duplicates), returns merged list.
    public LinkedList<Point> mergeLists(LinkedList<Point> list1, LinkedList<Point> list2) {
        if (list2 == null) {
            return list1; // needs to be fixed, not sure why it is null
        }
        list2.stream().filter((p) -> (!list1.contains(p))).forEach((p) -> {
            list1.add(p);
        });
        return list1;
    }

    public double distanceTo(Agent otherAgent) {
        return (this.getLocation().distance(otherAgent.getLocation()));
    }

    public double distanceToBase() {
        return distanceToBase;
    }

    public double timeToBase() {
        if (speed == 0) {
            return Constants.MAX_TIME;
        }
        return (distanceToBase / speed);
    }

    public void setDistanceToBase(double dist) {
        distanceToBase = dist;
    }

    public double distanceTo(Point somePoint) {
        return (this.getLocation().distance(somePoint));
    }

    @Override
    public String toString() {
        return ("[" + this.name + "] [" + this.ID + "] ");
    }

    public RobotConfig extractConfig() {
        RobotConfig robot = new RobotConfig(
                this.robotNumber,
                this.name,
                this.x,
                this.y,
                this.heading,
                this.sensRange,
                this.commRange,
                this.batteryPower,
                this.role.toString(),
                this.parent,
                this.child,
                this.ability,
                this.comStationLimit,
                this.speed,
                this.energyCunsumption);
        return robot;
    }

    abstract public Point takeStep(int timeElapsed);

    abstract public void writeStep(Point nextLoc, double[] sensorData, boolean updateSensorData);

    abstract public void flush();
}
