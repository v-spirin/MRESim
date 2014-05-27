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
package communication;

import agents.BasicAgent;
import environment.Environment;
import agents.RealAgent;
import environment.OccupancyGrid;
import java.awt.*;

public class PropModel1 {
    
    // Values easy to change depending on model, see e.g.
    // Bahl & Padmanabhan:  RADAR: An In-Building RF-based User Location and Tracking System
    private static final double REF_SIGNAL = -92; //-82;
    private static final int PATHLOSS_FACTOR = 1;
    //private static final double REF_DISTANCE = 1;
    private static final double WALL_ATTENUATION = 5;
    private static final int MAX_WALLS = 4;
    private static final double CUTOFF = -92;
    
    public static Polygon getRange(Environment env, BasicAgent agent) {
        double INCREMENT = Math.PI / 64;
        Polygon range = new Polygon();
        double angle;
        int oldX, oldY, currX, currY;
        
        for(int i=0; i<=2*Math.PI/INCREMENT; i++) {
            angle = agent.getHeading() + INCREMENT * i;
            oldX = agent.getX();
            oldY = agent.getY();
            for(int j=2; ; j+=2) {
                currX = agent.getX() + (int)(Math.cos(angle) * j);
                currY = agent.getY() + (int)(Math.sin(angle) * j);
                if(!env.locationExists(currX, currY) ||
                   signalStrength(agent.getCommRange(), env, agent.getLocation(), new Point(currX, currY)) < CUTOFF) {
                    range.addPoint(oldX, oldY);
                    break;
                }
                oldX = currX;
                oldY = currY;
            }
        }
        
        return range;
    }
    
    public static Polygon getRangeForRV(OccupancyGrid occGrid, BasicAgent agent) {
        double INCREMENT = Math.PI / 8;
        Polygon range = new Polygon();
        double angle;
        int ssuperOldX, ssuperOldY, superOldX, superOldY, oldX, oldY, currX, currY;
        
        for(int i=0; i<=2*Math.PI/INCREMENT; i++) {
            angle = agent.getHeading() + INCREMENT * i;
            oldX = agent.getX();
            oldY = agent.getY();
            superOldX = oldX;
            superOldY = oldY;
            ssuperOldX = oldX;
            ssuperOldY = oldY;
            for(int j=2; ; j+=2) {
                currX = agent.getX() + (int)(Math.cos(angle) * j);
                currY = agent.getY() + (int)(Math.sin(angle) * j);
                if(!occGrid.locationExists(currX, currY) ||
                   signalStrength((int)(agent.getCommRange()*0.5), occGrid, agent.getLocation(), new Point(currX, currY)) < (CUTOFF)) {                                    
                    range.addPoint(ssuperOldX, ssuperOldY);                    
                    break;
                    
                }
                if (!occGrid.obstacleWithinDistance(currX, currY, 3))
                {
                    if (occGrid.freeSpaceAt(currY, currY))
                    {
                        ssuperOldX = superOldX;
                        ssuperOldY = superOldY;
                        superOldX = oldX;
                        superOldY = oldY;
                        oldX = currX;
                        oldY = currY;
                    }
                }
            }
        }
        
        return range;
    }
    
    public static Polygon getRangeForRV(Environment env, BasicAgent agent) {
        double INCREMENT = Math.PI / (32 / 5);
        Polygon range = new Polygon();
        double angle;
        int oldX, oldY, currX, currY;
        
        for(int i=0; i<=2*Math.PI/INCREMENT; i++) {
            angle = agent.getHeading() + INCREMENT * i;
            oldX = agent.getX();
            oldY = agent.getY();
            for(int j=2; ; j+=2) {
                currX = agent.getX() + (int)(Math.cos(angle) * j);
                currY = agent.getY() + (int)(Math.sin(angle) * j);
                if(!env.locationExists(currX, currY) ||
                   signalStrength(agent.getCommRange(), env, agent.getLocation(), new Point(currX, currY)) < (CUTOFF + 5)) {                    
                    range.addPoint(oldX, oldY);                    
                    break;
                    
                }
                if (!env.obstacleWithinDistance(currX, currY, 3))
                {
                    oldX = currX;
                    oldY = currY;
                }
            }
        }
        
        return range;
    }
    
    //<editor-fold defaultstate="collapsed" desc="Signal Strength">
    private static double signalStrength(int agentRange, Environment env, Point p1, Point p2) {
        int numWalls = Math.min(MAX_WALLS, env.numObstaclesOnLine(p1.x, p1.y, p2.x, p2.y));
        double distance = p1.distance(p2);
        
        return (REF_SIGNAL - 10 * PATHLOSS_FACTOR * Math.log(distance / /*REF_DISTANCE*/agentRange) - numWalls * WALL_ATTENUATION);
    }
    
    private static double signalStrength(int agentRange, OccupancyGrid occGrid, Point p1, Point p2) {
        int numWalls = Math.min(MAX_WALLS, occGrid.numPossibleObstaclesOnLine(p1.x, p1.y, p2.x, p2.y));
        double distance = p1.distance(p2);
        
        return (REF_SIGNAL - 10 * PATHLOSS_FACTOR * Math.log(distance / /*REF_DISTANCE*/agentRange) - numWalls * WALL_ATTENUATION);
    }
    //</editor-fold>
    
    public static int[][] detectCommunication(Environment env, RealAgent[] agent) {
        int commTable[][] = new int[agent.length][agent.length];
        
        for(int i=0; i<agent.length; i++)
            for(int j=0; j<agent.length; j++)
                commTable[i][j] = 0;
        
        for(int i=0; i<agent.length-1; i++)
            for(int j=i+1; j<agent.length; j++) {
                if(signalStrength(agent[i].getCommRange(), env, agent[i].getLocation(), agent[j].getLocation()) > CUTOFF) {
                    commTable[i][j] = 1;
                    commTable[j][i] = 1;
                }
            }
        
        return commTable;
    }    
    
    public static double getRangeEstimate(RealAgent agent, Point p) {
        double INCREMENT = Math.PI / 64;
        Polygon range = new Polygon();
        double angle;
        int oldX, oldY, currX, currY;
        
        for(int i=0; i<=2*Math.PI/INCREMENT; i++) {
            angle = 0 + INCREMENT * i;
            oldX = p.x;
            oldY = p.y;
            for(int j=2; ; j+=2) {
                currX = p.x + (int)(Math.cos(angle) * j);
                currY = p.y + (int)(Math.sin(angle) * j);
                if(!agent.getOccupancyGrid().locationExists(currX, currY) ||
                   signalStrength(agent.getCommRange(), agent.getOccupancyGrid(), p, new Point(currX, currY)) < CUTOFF) {
                    range.addPoint(oldX, oldY);
                    break;
                }
                oldX = currX;
                oldY = currY;
            }
        }

        // Calculate area of range

        double sum = 0.0;
        for (int i = 0; i < range.npoints-1; i++) {
            sum = sum + (range.xpoints[i] * range.ypoints[i+1]) - (range.ypoints[i] * range.xpoints[i+1]);
        }
        sum = sum + (range.xpoints[range.npoints-1] * range.ypoints[0]) - (range.ypoints[range.npoints-1] * range.xpoints[0]);
        return Math.abs(0.5 * sum);
    }        
}
