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

import config.Constants;
import java.util.*;
import java.awt.*;

import config.RobotConfig;
import path.Path;

/**
 *
 * @author Julian de Hoog
 * 
 * This class maintains basic agent functionality / data required by any agent
 * 
 */


public class BasicAgent implements Agent {

// <editor-fold defaultstate="collapsed" desc="Class variables and Constructors">

    int robotNumber;        // Robot's number.  CONSTANT.  Does not change throughout run.
    String name;            // Name 
    int ID;                 /* Different from Robot Number, ID can change if robots swap roles,
                               and is used by robots to identify parent / child  */
    
    int x, y;               // Current position
    double heading;         // Current heading in radians
    double speed;
    
    int sensRange;          // Sensing range (radius)
    int commRange;          // Communication range (radius)
    int batteryPower;
    
    double distanceToBase;
    
    RobotConfig.roletype role;

    public static enum ExploreState {Initial, Explore, ReturnToParent, WaitForParent, GiveParentInfo, GoToChild, WaitForChild, GetInfoFromChild, OutOfService}
    private ExploreState state;

    int parent;             // Should keep ID and NOT RobotNumber of the parent
    int child;              // Should keep ID and NOT RobotNumber of the child
    
    private int stateTimer;  // keeps track of how much time spent in this state
    
    public Map<Integer, Double> knowledgeData = new HashMap(); // Keeps historical data about agent knowledge
        

    public BasicAgent(int no, String n, int id, int newX, int newY, double h, int sr, int cr, int bp, RobotConfig.roletype r, int p, int c, double sp) {
        robotNumber = no;
        name = n;
        ID = id;
        x = newX;
        y = newY;
        heading = h;
        sensRange = sr;
        commRange = cr;
        batteryPower = bp;
        role = r;
        state = ExploreState.Initial;
        parent = p;
        child = c;
        speed = sp;
    }
    
// </editor-fold>     

    
// <editor-fold defaultstate="collapsed" desc="Get and Set">
    
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
    
    public void setSpeed(double sp) {
        this.speed = sp;
    }
    
    public double getSpeed() {
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

    public ExploreState getState() {
        return state;
    }
    
    public void setState(ExploreState s) {
        if (this.state != s)
        {
            this.setStateTimer(0);
            this.state = s;
        }
    }
    
    public int getStateTimer() {
        return stateTimer;
    }
    
    public void setStateTimer(int t) {
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
    
    public double getHeading() {
        return this.heading;
    }
    
    public void setHeading(double newHeading) {
        this.heading = newHeading;
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

// </editor-fold>     

 
// <editor-fold defaultstate="collapsed" desc="Utility Functions">
    
    //Adds all points in list2 to list1 (no duplicates), returns merged list.
    public LinkedList<Point> mergeLists(LinkedList<Point> list1, LinkedList<Point> list2) {
        if (list2 == null) return list1; // needs to be fixed, not sure why it is null
        for(Point p : list2)
            if(!list1.contains(p))
                list1.add(p);
        
        return list1;
    }
    
    public double distanceTo(BasicAgent otherAgent) {
        return(this.getLocation().distance(otherAgent.getLocation()));
    }
    
    public double distanceToBase() {
        return distanceToBase;
    }
    
    public double timeToBase() {
        if (speed == 0)
            return Constants.MAX_TIME;
        return (distanceToBase / speed);
    }
    
    public void setDistanceToBase(double dist) {
        distanceToBase = dist;
    }

    public double distanceTo(Point somePoint) {
        return(this.getLocation().distance(somePoint));
    }
    
    @Override
    public String toString() {
        return ("[" + this.name + "] ");
    }

// </editor-fold> 
    
  

}
