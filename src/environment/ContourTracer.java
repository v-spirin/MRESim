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

import config.Constants;
import java.util.LinkedList;
import java.awt.Point;
/**
 *
 * @author julh
 */
public class ContourTracer {
    
    // Order is important!
    private static enum direction {NE, E, SE, S, SW, W, NW, N}

    private static Point dir2Point(Point pt, direction dir) {
        switch(dir) {
            case NE:    return new Point(pt.x+1,pt.y-1);
            case E:     return new Point(pt.x+1,pt.y);
            case SE:    return new Point(pt.x+1,pt.y+1);
            case S:     return new Point(pt.x,pt.y+1);
            case SW:    return new Point(pt.x-1,pt.y+1);
            case W:     return new Point(pt.x-1,pt.y);
            case NW:    return new Point(pt.x-1,pt.y-1);
            case N:     return new Point(pt.x,pt.y-1);
        }
        
        // this point should never be reached
        return null;
    }
    
    private static direction points2dir(Point from, Point to) {
        if(from.x - to.x == 1) {
            if(from.y - to.y == 1) return direction.SE;
            else if(from.y - to.y == 0) return direction.E;
            else if(from.y - to.y == -1) return direction.NE; }
        else if(from.x - to.x == 0) {
            if(from.y - to.y == 1) return direction.S;
            else if(from.y - to.y == -1) return direction.N; }
        else if(from.x - to.x == -1) {
            if(from.y - to.y == 1) return direction.SW;
            else if(from.y - to.y == 0) return direction.W;
            else if(from.y - to.y == -1) return direction.NW; }
        
        // this point should never be reached
        return null;
    }
    
    private static direction searchDir(Point from, Point to) {
        direction dir = points2dir(from,to);
        int searchIndex = (dir.ordinal() + 6) % 8;
        return direction.values()[searchIndex];
    }
    
    private static Point findNextPixelOnContour(OccupancyGrid occGrid, Point pt, direction dir) {
        direction currDir;
        Point currPoint;
        
        for(int i=0; i<8; i++) {
            currDir = direction.values()[(dir.ordinal() + i)%8];
            currPoint = dir2Point(pt, currDir);
            
            if(occGrid.locationExists(currPoint.x, currPoint.y) && 
               occGrid.frontierCellAt(currPoint.x, currPoint.y))
                  return currPoint;
        }
        
        // couldn't find any further points
        return null;
    }
    
    private static LinkedList <Point> traceContour(OccupancyGrid occGrid, int[][] label, int startX, int startY, direction startDir, int componentIndex) {
        Point currPixel, nextPixel;
        direction searchStart;
        LinkedList<Point> pts = new LinkedList<Point>();
        
        label[startX][startY] = componentIndex;
        Point firstPixel = new Point(startX, startY);
        pts.add(firstPixel);
        Point secondPixel = findNextPixelOnContour(occGrid, firstPixel, startDir);
        
        //if there is no further pixel, this is a one-pixel component and we're done
        if(secondPixel == null) return pts;
        
        //System.out.println("Firstpixel: " + firstPixel.x + " " + firstPixel.y);
        //System.out.println("Secondpixel: " + secondPixel.x + " " + secondPixel.y);
        
        searchStart = searchDir(secondPixel,firstPixel);
        //System.out.println("SS: " + searchStart.toString());

        currPixel = new Point(secondPixel.x, secondPixel.y);
        //System.out.println("CP: " + currPixel.x + " " + currPixel.y);
        
        nextPixel = findNextPixelOnContour(occGrid, currPixel, searchStart);
        //System.out.println("NP: " + nextPixel.x + " " + nextPixel.y);
        
        // In loop until all pixels on contour have been found
        while(!(currPixel.equals(firstPixel) && nextPixel.equals(secondPixel))) {
            pts.add(currPixel);
            label[currPixel.x][currPixel.y] = componentIndex;
            searchStart = searchDir(nextPixel, currPixel);
            //System.out.println("SS: " + searchStart.toString());
            currPixel = nextPixel;
            //System.out.println("CP: " + currPixel.x + " " + currPixel.y);
            nextPixel = findNextPixelOnContour(occGrid, currPixel, searchStart);
            //System.out.println("NP: " + nextPixel.x + " " + nextPixel.y);
        }
        
        return pts;
    }
    
    
    private static int[][] updateLabels(int[][] labels, LinkedList<Point> contour) {
        for(Point p: contour)
            labels[p.x][p.y] = 1;
        
        return labels;
    }
    
    public static LinkedList <LinkedList> findAllContours(OccupancyGrid occGrid) {
        LinkedList <LinkedList> contourList = new LinkedList<LinkedList>();
        LinkedList <Point> currContour;
        boolean hasFrontierCell;
        int maxGap, currGap;
        
        int[][] labels = new int[occGrid.width][occGrid.height];
        int componentIndex = 1;
        
        for(int i=0; i<labels.length; i++)
            for(int j=0; j<labels[0].length; j++)
                labels[i][j] = 0;
        
        // Assume that topline of occGrid is empty, i.e. no frontier cells.
        for(int j=0; j<occGrid.height; j++) 
            for(int i=0; i<occGrid.width; i++){
                if(occGrid.frontierCellAt(i, j) && 
                   (!occGrid.locationExists(i, j-1) || !occGrid.frontierCellAt(i,j-1)) && 
                   labels[i][j]==0) {
                    // We must have found external contour of new component
                    currContour = traceContour(occGrid, labels, i, j, direction.NE, componentIndex);
                    
                    // Check to make sure that current contour
                    //  (i)  borders on open space somewhere (ignore frontiers in e.g. corners of rooms)
                    //  (ii) has a gap somewhere big enough to plan a path into it
                    /*hasFrontierCell = false;
                    maxGap = 0;
                    currGap = 0;
                    for(Point p: currContour) {
                        if(occGrid.frontierBorderCellAt(p.x, p.y)) 
                            hasFrontierCell = true;
                        
                        if(occGrid.isInOpenSpace(p.x, p.y))
                            currGap++;
                        else if(currGap > maxGap) {
                            maxGap = currGap;
                            currGap = 0;
                        }
                    }
                    System.out.println("****** Max gap: " + maxGap);*/
                    for(Point p: currContour) {
                        if(occGrid.frontierBorderCellAt(p.x, p.y)) {
                    //if(hasFrontierCell) { // && maxGap >= Constants.STEP_SIZE) {
                        // this contour should be added
                        labels = updateLabels(labels, currContour);
                        contourList.add(currContour);
                        componentIndex++;
                        break;}
                    }
                }
                
            }
        
        return contourList;
    }
}
