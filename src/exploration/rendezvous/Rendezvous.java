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

import agents.BasicAgent;
import agents.RealAgent;
import communication.PropModel1;
import config.Constants;
import config.RobotConfig;
import environment.OccupancyGrid;
import java.awt.Point;
import java.awt.Polygon;
import java.util.LinkedList;

/**
 * Rendezvous describes a rendezvous location, and includes information about where each agent
 * should head to.
 * @author Victor
 */
public class Rendezvous {
    private Point childLocation; // where the child of the two agents meeting up should go
    private Point parentLocation; //where the parent of the two agents meeting up should go
    private int timeMeeting; //when they agree to meet up
    //private int minTimeMeeting; //time from which relay will be at the meeting point
    private int timeWait; //how long they agree to wait for partner at RV point
    public Rendezvous parentsRVLocation; //this is the RV location for our parent to meet with its parent (we calculate it).
    
    public Rendezvous(Point location) {
        if (location == null) {
            System.err.println("!!! location is null? This should never happen!");
        }
        childLocation = location;
        parentLocation = location;
        timeMeeting = Constants.MAX_TIME; //meeting time not agreed
        timeWait = Constants.MAX_TIME; //wait indefinitely
        //minTimeMeeting = Constants.MAX_TIME;
    }
    
    public Rendezvous copy() {
        if (childLocation == null) {
            System.err.println("!!! childLocation is null? This should never happen!");
            return null;
        }
        Rendezvous locCopy = new Rendezvous(childLocation);
        locCopy.setChildLocation(new Point(childLocation));
        locCopy.setParentLocation(new Point(parentLocation));
        locCopy.setTimeMeeting(timeMeeting);
        locCopy.setTimeWait(timeWait);
        //locCopy.setMinTimeMeeting(minTimeMeeting);
        if (parentsRVLocation != null)
            locCopy.parentsRVLocation = parentsRVLocation.copy();
        return locCopy;
    }
    
    @Override
    public boolean equals(Object that) {
        if (this == that) return true;
        if ( !(that instanceof Rendezvous)) return false;
        Rendezvous other = (Rendezvous)that;
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
                        RobotConfig.roletype.Relay, 0, 0, 0, 2, 1)
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
            
            double minDistanceDirect;
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
    
    @Override
    public String toString() {
        return "parentLoc: (" + (int)parentLocation.getX() + "," + (int)parentLocation.getX() + "), childLoc: (" + (int)childLocation.getX() + "," + (int)childLocation.getX() + 
                "), timeMeeting: " + timeMeeting + ", timeWait: " + timeWait;
    }
    
    
    //<editor-fold defaultstate="collapsed" desc="Getters and setters">
    public void setChildLocation(Point childLocation) {
        this.childLocation = childLocation;
    }
    
    public void setParentLocation(Point parentLocation) {
        this.parentLocation = parentLocation;
    }
    
    public void setTimeMeeting(int timeMeeting) {
        //System.out.println("Setting meeting time to " + timeMeeting);
        this.timeMeeting = timeMeeting;
    }
    
    /*public void setMinTimeMeeting(int minTimeMeeting) {
        this.minTimeMeeting = minTimeMeeting;
    }
    
    public int getMinTimeMeeting() {
        return minTimeMeeting;
    }*/
    
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
    
    
    
    public int getTimeWait() {
        return timeWait;
    }
    
    //</editor-fold>
}
