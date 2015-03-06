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
package environment;

import agents.RealAgent;
import config.Constants;
import java.util.*;
import java.awt.*;

/**
 *
 * @author julh
 */
public class Frontier implements Comparable<Frontier>  {

// <editor-fold defaultstate="collapsed" desc="Variables and Constructor">

    private Point centre;
    private double distanceToCentre;
    private Polygon areaPolygon;
    private LinkedList<Point> polygonOutline;
    private double area;
    
    
   // This constructor used by new calculatefrontier function
    public Frontier(int agentX, int agentY, LinkedList<Point> po) {
        this.polygonOutline = po;
        this.areaPolygon = createAreaPolygonFromList(po);
        this.area = calculateArea();
        this.centre = calculateCentre();
        this.distanceToCentre = centre.distance(agentX, agentY);
    }
    
    // This constructor used by copy() function
    public Frontier(LinkedList<Point> po, Polygon ap, double a, Point c, double d2c) {
        this.polygonOutline = po;
        this.areaPolygon = ap;
        this.area = a;
        this.centre = c;
        this.distanceToCentre = d2c;
    }
    

// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Key functions copy, compareto">

    public Frontier copy() {
        return new Frontier(polygonOutline, areaPolygon, area, centre, distanceToCentre);
    }

    public int compareTo(Frontier other) {
        if((other.area / other.getDistanceToCentre()) > (this.area / this.distanceToCentre))
            return 1;
        else
            return -1;
    }
    
    @Override
    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final Frontier other = (Frontier) obj;
        
        return this.centre.equals(other.centre) &&
                this.area == other.area;
    }

    @Override
    public int hashCode() {
        int hash = 3;
        hash = 41 * hash + (this.centre != null ? this.centre.hashCode() : 0);
        hash = 41 * hash + (int) (Double.doubleToLongBits(this.area) ^ (Double.doubleToLongBits(this.area) >>> 32));
        return hash;
    }
    
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Get and set">

    public Polygon getAreaPolygon() {
        return areaPolygon;
    }
    
    public Point getCentre() {
        return centre;
    }
    
    public double getDistanceToCentre() {
        return distanceToCentre;
    }
    
    public double getArea() {
        return area;
    }
    
    public LinkedList<Point> getPolygonOutline () {
        return polygonOutline;
    }

// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Utility functions">

    public Point getClosestPoint(Point ref, OccupancyGrid grid) {
        //return getCentre();
        
        double closestDist = 1000000;
        Point closestPoint = new Point(0,0);
        Point closePointNearWall = new Point(0,0);
        for(int i=0; i<polygonOutline.size(); i++) {
            if(polygonOutline.get(i).distance(ref) < closestDist)
            {
                if (!grid.obstacleWithinDistance(polygonOutline.get(i).x, polygonOutline.get(i).y, Constants.WALL_DISTANCE)) {
                    closestPoint = polygonOutline.get(i);
                    closestDist = polygonOutline.get(i).distance(ref);
                } else
                {
                    closePointNearWall = polygonOutline.get(i);
                }
            }
        }
        //return this.getCentre();
        
        //if can't find closest point as it's too close to wall, maybe we can return center
        if ((closestPoint.x == 0) && (closestPoint.y == 0))
        {
            if (!grid.obstacleWithinDistance(this.getCentre().x, this.getCentre().y, Constants.WALL_DISTANCE))
                return this.getCentre();
        }
        //    closestPoint = closePointNearWall; // at least it's something, the frontier is probably bad anyway...
        return closestPoint;
    }

    public Point getClosestPointInRange(RealAgent agent) {
        double closestDist = 1000000;
        Point closestPoint = new Point(0,0);
        for(int i=0; i<polygonOutline.size(); i++) {
            if(polygonOutline.get(i).distance(agent.getLocation()) < closestDist &&
               !agent.getOccupancyGrid().obstacleWithinDistance(polygonOutline.get(i).x, polygonOutline.get(i).y, Constants.WALL_DISTANCE) &&
               polygonOutline.get(i).distance(agent.getTeammate(1).getLocation()) < agent.getAllTeammates().size() * agent.getCommRange() - 5) {
                closestPoint = polygonOutline.get(i);
                closestDist = polygonOutline.get(i).distance(agent.getLocation());
            }
        }

        return closestPoint;
    }

    public boolean hasUnknownBoundary(OccupancyGrid grid) {
        for(Point p: polygonOutline)
            if(grid.frontierBorderCellAt(p.x, p.y))
                return true;
        return false;
    }

    private Polygon createAreaPolygonFromList(LinkedList<Point> list) {
        Polygon newP = new Polygon();
        for(Point p: list)
            newP.addPoint(p.x, p.y);
        
        return newP;
    }
    
    private double calculatePerimeterApprox() {
        double runningTotal = 0;
        int dx = 0;
        int dy = 0;
        
        for(int i=0; i<areaPolygon.npoints-1; i++)
        {
            //runningTotal += Math.pow(Math.pow(areaPolygon.xpoints[i] - areaPolygon.xpoints[i+1], 2) +
            //                 Math.pow(areaPolygon.ypoints[i] - areaPolygon.ypoints[i+1], 2), 1);
            dx = Math.abs(areaPolygon.xpoints[i] - areaPolygon.xpoints[i+1]);
            dy = Math.abs(areaPolygon.ypoints[i] - areaPolygon.ypoints[i+1]);
            runningTotal += Math.max(dx, dy);
        }
        //runningTotal += Math.pow(Math.pow(areaPolygon.xpoints[areaPolygon.npoints-1] - areaPolygon.xpoints[0], 2) +
        //                     Math.pow(areaPolygon.ypoints[areaPolygon.npoints-1] - areaPolygon.ypoints[0], 2), 1);
        dx = Math.abs(areaPolygon.xpoints[areaPolygon.npoints-1] - areaPolygon.xpoints[0]);
        dy = Math.abs(areaPolygon.ypoints[areaPolygon.npoints-1] - areaPolygon.ypoints[0]);
        runningTotal += Math.max(dx, dy);
        if(runningTotal < 0) runningTotal*=-1;
        
        return runningTotal;
    }
    
    // Simple polygon area calculation, see for example
    // http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/
    private double calculateArea() {
        double runningTotal = 0;
        
        for(int i=0; i<areaPolygon.npoints-1; i++)
            runningTotal += (areaPolygon.xpoints[i]*areaPolygon.ypoints[i+1] -
                             areaPolygon.xpoints[i+1]*areaPolygon.ypoints[i]);
        runningTotal += (areaPolygon.xpoints[areaPolygon.npoints-1]*areaPolygon.ypoints[0] -
                             areaPolygon.xpoints[0]*areaPolygon.ypoints[areaPolygon.npoints-1]);
        if(runningTotal < 0) runningTotal*=-1;
        
        runningTotal /= 2;
        
        
        return runningTotal + calculatePerimeterApprox();
    }
    
    private Point calculateCentre() {
        double cx, cy;
        Point temp;
        double runningTotal = 0;
        
        //determine centre X
        for(int i=0; i<areaPolygon.npoints-1; i++)
            runningTotal += ((areaPolygon.xpoints[i] + areaPolygon.xpoints[i+1]) *
                             (areaPolygon.xpoints[i]*areaPolygon.ypoints[i+1] -
                              areaPolygon.xpoints[i+1]*areaPolygon.ypoints[i]));
        runningTotal += ((areaPolygon.xpoints[areaPolygon.npoints-1] + areaPolygon.xpoints[0]) *
                         (areaPolygon.xpoints[areaPolygon.npoints-1]*areaPolygon.ypoints[0] -
                          areaPolygon.xpoints[0]*areaPolygon.ypoints[areaPolygon.npoints-1]));
 
        cx = runningTotal / (6*area);
        if(cx < 0) cx*=-1;
        
        runningTotal = 0;

        //determine centre Y
        for(int i=0; i<areaPolygon.npoints-1; i++)
            runningTotal += ((areaPolygon.ypoints[i] + areaPolygon.ypoints[i+1]) *
                             (areaPolygon.xpoints[i]*areaPolygon.ypoints[i+1] -
                              areaPolygon.xpoints[i+1]*areaPolygon.ypoints[i]));
        runningTotal += ((areaPolygon.ypoints[areaPolygon.npoints-1] + areaPolygon.ypoints[0]) *
                         (areaPolygon.xpoints[areaPolygon.npoints-1]*areaPolygon.ypoints[0] -
                          areaPolygon.xpoints[0]*areaPolygon.ypoints[areaPolygon.npoints-1]));
        
        cy = runningTotal / (6*area);
        if(cy < 0)cy*=-1;
        
        temp = new Point((int)cx, (int)cy);

        // If the centre is inside the frontier, perfect, we're done.
        if(areaPolygon.contains(temp))
            return temp;
        
        // However, if the centre is not inside the frontier (e.g. if it's crescent shaped)
        // then this is useless to us, since robot won't navigate into frontier.
        // TO DO (MAYBE):  find point somewhere in centre of mass.
        /*int area_crescent_limit = 20; //if the polygon is smaller than this limit, then we don't care that
                                      //the center is outside - we will likely get some measurements from the frontier
                                      //anyway, as the center is most likely close enough to the frontier
        if (area < area_crescent_limit)
            return temp;*/
        //otherwise, return a random polygon point
        Random rnd = new Random(Constants.RANDOM_SEED);
        int index = rnd.nextInt(areaPolygon.npoints);
        return new Point(areaPolygon.xpoints[index], areaPolygon.ypoints[index]);
    }
    
    @Override
    public String toString() {
        return new String("Frontier found with area " + area +
                          ", and centre (" + centre.x + "," + centre.y + ").");
    }
    
// </editor-fold>     

}
