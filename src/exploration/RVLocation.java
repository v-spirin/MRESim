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
package exploration;

import agents.BasicAgent;
import agents.RealAgent;
import communication.PropModel1;
import config.Constants;
import config.RobotConfig;
import environment.Environment;
import environment.OccupancyGrid;
import java.awt.Point;
import java.awt.Polygon;
import java.util.LinkedList;

/**
 * RVLocation describes a rendezvous location, and includes information about where each agent
 * should head to.
 * @author Victor
 */
public class RVLocation {
    private Point childLocation; // where the child of the two agents meeting up should go
    private Point parentLocation; //where the parent of the two agents meeting up should go
    private int timeMeeting; //when they agree to meet up
    private int minTimeMeeting; //time from which relay will be at the meeting point
    private int timeWait; //how long they agree to wait for partner at RV point
    
    public RVLocation(Point location) {
        childLocation = location;
        parentLocation = location;
        timeMeeting = Constants.MAX_TIME; //meeting time not agreed
        timeWait = Constants.MAX_TIME; //wait indefinitely
        minTimeMeeting = Constants.MAX_TIME;
    }
    
    public RVLocation copy() {
        RVLocation locCopy = new RVLocation(childLocation);
        locCopy.setChildLocation(new Point(childLocation));
        locCopy.setParentLocation(new Point(parentLocation));
        locCopy.setTimeMeeting(timeMeeting);
        locCopy.setTimeWait(timeWait);
        locCopy.setMinTimeMeeting(minTimeMeeting);
        return locCopy;
    }
    
    @Override
    public boolean equals(Object that) {
        if (this == that) return true;
        if ( !(that instanceof RVLocation)) return false;
        RVLocation other = (RVLocation)that;
        return
                this.getChildLocation().equals(other.getChildLocation()) &&
                this.getParentLocation().equals(other.getParentLocation());
                //this.getTimeMeeting() == other.getTimeMeeting() &&
                //this.getTimeWait() == other.getTimeWait() &&
                //this.getMinTimeMeeting() == other.getMinTimeMeeting();
        
    }
            
    
    // find the second RV point in a pair (one agent goes to the first point, other goes to the second)
    // The second point is found through wall, within comm range, that gives an advantage heading to the goal
    public static Point findSecondRVPoint(RealAgent agent, Point firstRV, Point goal, double minAcceptableRatio)
    {
        long realtimeStart = System.currentTimeMillis();
        LinkedList<Point> candidatePoints = new LinkedList<Point>();
        LinkedList<Point> directPoints = new LinkedList<Point>(); //connection is not through a wall
        
        OccupancyGrid occGrid = agent.getOccupancyGrid();
        
        int pointSkip = 1;
        
        /*Polygon commPoly = PropModel1.getRangeForRV(occGrid, 
                new BasicAgent(0, "", 0, firstRV.x, firstRV.y, 0, 0, 
                        Math.min(agent.getCommRange(), 
                                agent.getParentTeammate().getCommRange()), 0, 
                        RobotConfig.roletype.Relay, 0, 0, 0)
                );*/
        Polygon commPoly = PropModel1.getRangeForRV(occGrid, 
                new BasicAgent(0, "", 0, firstRV.x, firstRV.y, 0, 0, 200, 0, 
                        RobotConfig.roletype.Relay, 0, 0, 0)
                );
        
        int counter = 0;
        //for(Point p : ExplorationImage.polygonPoints(commPoly))
        for (int i = 0; i < commPoly.npoints; i++)
        {
            Point p = new Point(commPoly.xpoints[i], commPoly.ypoints[i]);
            if (occGrid.freeSpaceAt(p.x, p.y) /*&& !env.directLinePossible(firstRV.x, firstRV.y, p.x, p.y)*/)
            {
                if (counter % pointSkip == 0)
                {
                    if (!occGrid.directLinePossible(firstRV.x, firstRV.y, p.x, p.y))
                        candidatePoints.add(p);
                    else
                        directPoints.add(p);
                }
                counter++;
            }
        }
        //System.out.println("Added " + candidatePoints.size() + " candidate points, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        //realtimeStart = System.currentTimeMillis();
        // let's find which candidate point is closest to goal
        
        Point secondRV = firstRV;
        
        if (candidatePoints.size() > 0)
        {
            double minDistance = agent.calculatePath(firstRV, goal).getLength();
            
            for (Point p: candidatePoints)
            {
                double distance = agent.calculatePath(p, goal).getLength();
                if (distance < minDistance)
                {
                    minDistance = distance;
                    secondRV = p;
                }
            }
            
            //System.out.println("1, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
            //realtimeStart = System.currentTimeMillis();
            
            double minDistanceDirect = Integer.MAX_VALUE;
            /*for (Point p: directPoints)
            {
                double distance = agent.calculatePath(p, goal).getLength();
                if (distance < minDistanceDirect)
                {
                    minDistanceDirect = distance;
                }
            }*/
            minDistanceDirect = agent.calculatePath(firstRV, goal).getLength() - (agent.getCommRange() / Constants.DEFAULT_SPEED);
            
            //System.out.println("2, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
            //realtimeStart = System.currentTimeMillis();
            
            //communication through the wall gives no advantage
            if ((minDistanceDirect < 0) || ((minDistance / minDistanceDirect) > minAcceptableRatio)) 
                secondRV = firstRV;
            /*else
            {
                if ((minDistance / agent.calculatePath(firstRV, secondRV).getLength()) > minAcceptableRatio)
                    secondRV = firstRV;
            }*/
        }
        //System.out.println("Checked all candidate points, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        
        return secondRV;        
    }
    
    
    //<editor-fold defaultstate="collapsed" desc="Getters and setters">
    public void setChildLocation(Point childLocation) {
        this.childLocation = childLocation;
    }
    
    public void setParentLocation(Point parentLocation) {
        this.parentLocation = parentLocation;
    }
    
    public void setTimeMeeting(int timeMeeting) {
        this.timeMeeting = timeMeeting;
    }
    
    public void setMinTimeMeeting(int minTimeMeeting) {
        this.minTimeMeeting = minTimeMeeting;
    }
    
    public void setTimeWait(int timeWait) {
        this.timeWait = timeWait;
    }
    
    public Point getChildLocation() {
        return childLocation;
    }
    
    public Point getParentLocation() {
        return parentLocation;
    }
    
    public int getTimeMeeting() {
        return timeMeeting;
    }
    
    public int getMinTimeMeeting() {
        return minTimeMeeting;
    }
    
    public int getTimeWait() {
        return timeWait;
    }
    
    //</editor-fold>
}
