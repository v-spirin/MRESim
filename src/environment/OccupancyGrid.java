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

import java.awt.*;
import java.awt.geom.Line2D;
import java.util.LinkedList;

/**
 *
 * @author julh
 */
public class OccupancyGrid {
    public enum OccGridBit { FreeSpace, SafeSpace, Obstacle, KnownAtBase, GotRelayed } // which bit does what
    
    private byte[][] grid;
    public int height;
    public int width;
    
    // occupancy grid information
    private int cellsMarkedAsRelayed;
    private int cellsMarkedAsKnownAtBase;
    private int cellsMarkedAsFree;
    
    public OccupancyGrid(int newWidth, int newHeight) {
        width = newWidth;
        height = newHeight;
        grid = new byte[width][height];
        for(int i=0; i<width; i++)
            for(int j=0; j<height; j++)
                grid[i][j] = 0;
        
        cellsMarkedAsRelayed = 0;
        cellsMarkedAsKnownAtBase = 0;
        cellsMarkedAsFree = 0;
    }
    
    public OccupancyGrid copy()
    {
        OccupancyGrid copyGrid = new OccupancyGrid(width, height);
        for(int i=0; i<width; i++)
            for(int j=0; j<height; j++)
                copyGrid.setByte(i, j, getByte(i, j));
        copyGrid.cellsMarkedAsKnownAtBase = cellsMarkedAsKnownAtBase;
        copyGrid.cellsMarkedAsRelayed = cellsMarkedAsRelayed;
        copyGrid.cellsMarkedAsFree = cellsMarkedAsFree;
        
        return copyGrid;
    }
    
    public boolean equals(Object obj) 
    {
        if (obj == null)
            return false;
        if (obj == this)
            return true;
        if (obj.getClass() != getClass())
            return false;
        
        return grid.equals(((OccupancyGrid)obj).grid);
    }
    
    //TODO: should be able to make this more efficient
    public LinkedList<Point> mergeGrid(OccupancyGrid partnerOccGrid, boolean withBaseStation) {
        LinkedList<Point> cellsUpdated = new LinkedList();
        for(int i=0; i<this.width; i++) {        
            for(int j=0; j<this.height; j++) {
                if(this.getByteNoRelay(i,j) != partnerOccGrid.getByteNoRelay(i,j)) {  
                    if (partnerOccGrid.freeSpaceAt(i, j))
                        this.setFreeSpaceAt(i, j);
                    if (partnerOccGrid.safeSpaceAt(i, j))
                        this.setSafeSpaceAt(i, j);
                    if (partnerOccGrid.obstacleAt(i, j))
                        this.setObstacleAt(i, j);
                    if (partnerOccGrid.isKnownAtBase(i, j))
                        this.setKnownAtBase(i, j);
                        
                    // if the information is completely new, get all of it, including relay status
                    // otherwise, we may be the relay! So get all new info, apart from relay status
                    if (this.getByte(i, j) == 0) {
                        if (partnerOccGrid.isGotRelayed(i, j))
                            this.setGotRelayed(i, j);
                        //this.setByte(i, j, (byte)(this.getByte(i,j) | partnerOccGrid.getByte(i,j)));
                    }
                    /*else {
                        this.setByte(i, j, (byte)(this.getByte(i,j) | partnerOccGrid.getByteNoRelay(i,j)));
                    }*/
                    if (withBaseStation)
                        this.setKnownAtBase(i, j);
                    cellsUpdated.add(new Point(i,j));
                }
            }
        }
        return cellsUpdated;
    }
    
    
    public boolean frontierCellAt(int xCoord, int yCoord) {
        return
                (
                freeSpaceAt(xCoord, yCoord) && 
               !safeSpaceAt(xCoord, yCoord) &&
               !obstacleAt(xCoord, yCoord)
                ) ||
                (
                freeSpaceAt(xCoord, yCoord) &&
                !obstacleAt(xCoord, yCoord) &&
                frontierBorderCellAt(xCoord, yCoord)
                )
                ;
    }

    public boolean frontierBorderCellAt(int xCoord, int yCoord) {
        //if(!frontierCellAt(xCoord, yCoord))
        //    return false;

        for(int i=xCoord-1; i<=xCoord+1; i++)
            for(int j=yCoord-1; j<=yCoord+1; j++)
                if(emptyAt(i,j))
                    return true;

        return false;
    }


    public boolean isInOpenSpace(int xCoord, int yCoord) {
        for(int i=xCoord-1; i<=xCoord+1; i++)
            for(int j=yCoord-1; j<=yCoord+1; j++)
                if(!freeSpaceAt(i,j))
                    return false;
        return true;
    }
    
    public boolean emptyAt(int xCoord, int yCoord) {
        if (xCoord < 0) return false;
        if (yCoord < 0) return false;
        if (xCoord > width) return false;
        if (yCoord > height) return false;
        return(!freeSpaceAt(xCoord, yCoord) &&
               !safeSpaceAt(xCoord, yCoord) &&
               !obstacleAt(xCoord, yCoord));
    }

    public boolean freeSpaceAt(int xCoord, int yCoord) {
        if(getBit(xCoord, yCoord, OccGridBit.FreeSpace.ordinal()) == 1)
            return true;
        else
            return false;
    }
    
        
    public boolean isKnownAtBase(int xCoord, int yCoord) {
        if(getBit(xCoord, yCoord, OccGridBit.KnownAtBase.ordinal()) == 1)
            return true;
        else
            return false;
    }
    
    public void setKnownAtBase(int xCoord, int yCoord) {
        if (!isKnownAtBase(xCoord, yCoord))
            cellsMarkedAsKnownAtBase++;
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.KnownAtBase, 1);
    }
    
    public boolean isGotRelayed(int xCoord, int yCoord) {
        if(getBit(xCoord, yCoord, OccGridBit.GotRelayed.ordinal()) == 1)
            return true;
        else
            return false;
    }
     
    // Marks this cell as being relayed to base by another robot
    // Used in UtilExploration.
    public void setGotRelayed(int xCoord, int yCoord) {
        if (!isGotRelayed(xCoord, yCoord))
            cellsMarkedAsRelayed++;
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.GotRelayed, 1);
        
    }
    
    // Marks this cell as NOT being relayed to base by another robot
    // Used in UtilExploration.
    public void setGotUnrelayed(int xCoord, int yCoord) {
        if (isGotRelayed(xCoord, yCoord))
            cellsMarkedAsRelayed--;
        assert (cellsMarkedAsRelayed >= 0);
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.GotRelayed, 0);
    }
    
    public int getNumRelayedCells()
    {   
        return cellsMarkedAsRelayed;
    }
    
    public int getNumCellsKnownAtBase()
    {
        return cellsMarkedAsKnownAtBase;
    }
    
    public int getNumFreeCells()
    {
        return cellsMarkedAsFree;
    }

    public void setFreeSpaceAt(int xCoord, int yCoord) {
        if (!freeSpaceAt(xCoord, yCoord))
            cellsMarkedAsFree++;
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.FreeSpace, 1);
    }
    
    public void setNoObstacleAt(int xCoord, int yCoord) {
        try{
            setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.Obstacle, 0);
        }
        catch(ArrayIndexOutOfBoundsException  e) {
            System.out.println(this.toString() + "Error: ArrayIndexOutOfBoundsException.  Did not set as no obstacle.");
        }
    }

    public boolean safeSpaceAt(int xCoord, int yCoord) {
        if(getBit(xCoord, yCoord, OccGridBit.SafeSpace.ordinal()) == 1)
            return true;
        else
            return false;
    }

    public void setSafeSpaceAt(int xCoord, int yCoord) {
        if (!freeSpaceAt(xCoord, yCoord))
            cellsMarkedAsFree++;
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.FreeSpace, 1);
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.SafeSpace, 1);
    }

    public boolean obstacleAt(int xCoord, int yCoord) {
        if(getBit(xCoord, yCoord, OccGridBit.Obstacle.ordinal()) == 1)
            return true;
        else
            return false;
    }

    public void setObstacleAt(int xCoord, int yCoord) {
        try{
            setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.Obstacle, 1);
        }
        catch(ArrayIndexOutOfBoundsException  e) {
            System.out.println(this.toString() + "Error: ArrayIndexOutOfBoundsException.  Did not set as obstacle.");
        }
    }

    public byte getByte(int x, int y) {
        return grid[x][y];
    }
    
    public byte getByteNoRelay(int x, int y) {
        return (byte) (grid[x][y] & ~(1 << OccGridBit.GotRelayed.ordinal()));
    }
    
    public int getBit(int xCoord, int yCoord, int bit) {
        try {
        if((grid[xCoord][yCoord] & (byte)(Math.pow(2,bit))) > 0)
            return 1;
        else
            return 0;
        }
        catch(ArrayIndexOutOfBoundsException e) {
            System.out.println("ERROR: Array index out of bounds at x=" + xCoord + ", y=" + yCoord + ".");
            return 0;
        }
    }
    
    public String toString(int xCoord, int yCoord) {
        String bitString = new String();
        for(int i=0; i<8; i++) bitString.concat(Integer.toString(getBit(xCoord, yCoord, i)));
        return (this.toString() + "Byte at " + xCoord + ", " + yCoord + " has value " + bitString);
    }
    
    @Override
    public String toString() {
        return ("[OccupancyGrid] ");
    }
    
    public boolean locationExists(int x, int y) {
        return(x < width  && x >= 0 &&  y < height && y >= 0);
    }
    
    // Checks if there is a line from source to dest that doesn't go through obstacles.
    // Note:  by current implementation lines are possible through unknown space.  To change this
    // only a slight tweak in second part of if statement needed, i.e. change to:
    // && !freeSpaceAt(i,j)
    public boolean directLinePossible(int sourceX, int sourceY, int destX, int destY) {
        for(int i=Math.min(sourceX, destX)+1; i<=Math.max(sourceX, destX)-1; i++)
            for(int j=Math.min(sourceY, destY)+1; j<=Math.max(sourceY, destY)-1; j++)
                if((distPointToLine(sourceX,sourceY,destX,destY,i,j) <= 0.5) && obstacleAt(i,j))
                    return false;
        
        return true;
    }
    
    public boolean isOnLine(Point endPoint1, Point endPoint2, Point checkPoint)
    {
        return (Line2D.ptSegDist(endPoint1.x, endPoint1.y, endPoint2.x, endPoint2.y, checkPoint.x, checkPoint.y) < 1.0);
    }

    // This function finds the shortest distance from P3 to the line between P1 and P2
    public double distPointToLine(int x1, int y1, int x2, int y2, int x3, int y3) {
        if((x3 == x1 && y3 == y1)  || (x3 == x2 && y3 == y2)) return 0;
        
        double dist = Math.sqrt(Math.pow(y2 - y1,2) + Math.pow(x2 - x1, 2));
        double slope = ((x3 - x1) * (x2 - x1) + (y3 - y1) * (y2 - y1)) / Math.pow(dist,2);
        
        double xIntersection = x1 + slope * (x2 - x1);
        double yIntersection = y1 + slope * (y2 - y1);
        
        double shortestDist = Math.sqrt(Math.pow(x3 - xIntersection, 2) + Math.pow(y3 - yIntersection, 2));
        
        return shortestDist;
    }
    
    // Returns distance to nearest wall, up to a maximum distance
    public boolean obstacleWithinDistance(int x, int y, int minDistance) {
        for(int i=x-minDistance; i<=x+minDistance; i++)
            for(int j=y-minDistance; j<=y+minDistance; j++)
                if(locationExists(i,j) &&
                   new Point(x,y).distance(i,j) <= minDistance &&
                   obstacleAt(i,j))
                    return true;
        return false;
    }
    
    public int getEmptySpacesWithinSquare(int x, int y, int minDistance) {
        int count = 0;
        for(int i=x-minDistance; i<=x+minDistance; i++)
            for(int j=y-minDistance; j<=y+minDistance; j++)
                if(locationExists(i,j) &&
                   /*new Point(x,y).distance(i,j) <= minDistance &&*/
                   emptyAt(i,j) && ((i!=x) || (j!=y)))
                    count++;
        return count;
    }
    
    public int getSafeSpacesWithinSquare(int x, int y, int minDistance) {
        int count = 0;
        for(int i=x-minDistance; i<=x+minDistance; i++)
            for(int j=y-minDistance; j<=y+minDistance; j++)
                if(locationExists(i,j) &&
                   /*new Point(x,y).distance(i,j) < minDistance &&*/
                   safeSpaceAt(i,j) && ((i!=x) || (j!=y)))
                    count++;
        return count;
    }
    
    public int numObstaclesOnLine(int x1, int y1, int x2, int y2) {
        int counter = 0;
        double angle = Math.atan2(y2 - y1, x2 - x1);
        int distance = (int)(Math.sqrt(Math.pow(y2-y1, 2) + Math.pow(x2-x1, 2)));
        int currX, currY;
        
        for(int i=0; i<=distance; i++) {
            currX = x1 + (int)(Math.cos(angle) * i);
            currY = y1 + (int)(Math.sin(angle) * i);
            
            if(this.obstacleAt(currX, currY)){
                counter++;
                i++;  // To prevent the same pixel counting as an obstacle twice.
            }
        }
        
        return counter;
    }
    
    public int numPossibleObstaclesOnLine(int x1, int y1, int x2, int y2) {
        int counter = 0;
        double angle = Math.atan2(y2 - y1, x2 - x1);
        int distance = (int)(Math.sqrt(Math.pow(y2-y1, 2) + Math.pow(x2-x1, 2)));
        int currX, currY;
        
        for(int i=0; i<=distance; i++) {
            currX = x1 + (int)(Math.cos(angle) * i);
            currY = y1 + (int)(Math.sin(angle) * i);
            
            if(!this.freeSpaceAt(currX, currY)){
                counter++;
                i++;  // To prevent the same pixel counting as an obstacle twice.
            }
        }
        
        return counter;
    }
    
    private void setBit(int xCoord, int yCoord, OccGridBit bit, int value) {
        setBit(xCoord, yCoord, bit.ordinal(), value);
    }
    
    private void setBit(int xCoord, int yCoord, int bit, int value) {
        int bitValue = grid[xCoord][yCoord] & (byte)(Math.pow(2,bit));
        if(bitValue == 0)
            if(value == 0) return;
            else grid[xCoord][yCoord] += (byte)(Math.pow(2,bit));
        else
            if(value == 1) return;
            else grid[xCoord][yCoord] -= (byte)(Math.pow(2,bit));
    }
    
    private void setByte(int x, int y, byte value) {
        grid[x][y] = value;
    }
    
    //<editor-fold defaultstate="collapsed" desc="DELETE">
    /*public boolean testTrueAt(int xCoord, int yCoord) {
        if(getBit(xCoord, yCoord, OccGridBit.Test.ordinal()) == 1)
            return true;
        else
            return false;
    }

    public void setTestTrueAt(int xCoord, int yCoord) {
        setBit(xCoord, yCoord, OccupancyGrid.OccGridBit.Test, 1);
    }
    
    // Sets test bit in all occupancy grids to 0 -- purely for testing purposes.
    public void initializeTestBits() {
        for(int i=0; i<grid.length; i++)
            for(int j=0; j<grid[0].length; j++)
                setBit(i,j,OccGridBit.Test.ordinal(),0);
    }
    */
    //</editor-fold>
}

