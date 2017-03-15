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
package communication;

import exploration.NearRVPoint;
import java.awt.Point;

/**
 *
 * @author Victor
 * This class represents a communication link between two topological regions of the map.
 */
public class CommLink implements Comparable<CommLink>{
    private NearRVPoint localPoint = new NearRVPoint(0, 0);
    private NearRVPoint remotePoint = new NearRVPoint(0, 0);
    //private TopologicalNode localNode;
    //private TopologicalNode remoteNode;
    public double utility;
    public int numObstacles;
    
    public CommLink() {
        
    }
    
    public CommLink(Point localPoint, Point remotePoint) {
        this.localPoint = new NearRVPoint(localPoint.x, localPoint.y);
        this.remotePoint = new NearRVPoint(remotePoint.x, remotePoint.y);
        //this.localNode = localNode;
        //this.remoteNode = remoteNode;
    }
    
    public CommLink(NearRVPoint localPoint, NearRVPoint remotePoint) {
        this.localPoint = localPoint;
        this.remotePoint = remotePoint;
        //this.localNode = localNode;
        //this.remoteNode = remoteNode;
    }
    
    @Override
    public int compareTo(CommLink other) {
        if (other.utility > this.utility)
            return 1;
        else
            return -1;
    }
    
    public CommLink createReverseLink() {
        CommLink reverseLink = new CommLink(remotePoint, localPoint);
        return reverseLink;
    }
    
    public void setLocalPoint(Point localPoint) {
        this.localPoint = new NearRVPoint(localPoint.x, localPoint.y);
    }
    
    public NearRVPoint getLocalPoint() {
        return localPoint;
    }
    
    public void setRemotePoint(Point remotePoint) {
        this.remotePoint = new NearRVPoint(remotePoint.x, remotePoint.y);
    }
    
    public NearRVPoint getRemotePoint() {
        return remotePoint;
    }
    
    public double getDistanceLocalToFrontier() {
        return localPoint.distanceToFrontier;
    }
    
    public double getDistanceLocalToParent() {
        return localPoint.distanceToParent;
    }
    
    public double getDistanceRemoteToFrontier() {
        return remotePoint.distanceToFrontier;
    }
    
    public double getDistanceRemoteToParent() {
        return remotePoint.distanceToParent;
    }
    
    /*public void setLocalNode(TopologicalNode localNode) {
        this.localNode = localNode;
    }
    
    public TopologicalNode getLocalNode() {
        return localNode;
    }
    
    public void setRemoteNode(TopologicalNode remoteNode) {
        this.remoteNode = remoteNode;
    }
    
    public TopologicalNode getRemoteNode() {
        return remoteNode;
    }*/
}
