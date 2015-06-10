/*
 *     Copyright 2010, 2015 Julian de Hoog (julian@dehoog.ca), Victor Spirin (victor.spirin@cs.ox.ac.uk)
 *
 *     This file is part of MRESim 2.2, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our papers:
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

import agents.BasicAgent;
import environment.Environment;
import agents.RealAgent;
import environment.OccupancyGrid;
import java.awt.*;

public class PropModel1 {
    
    // Values easy to change depending on model, see e.g.
    // Bahl & Padmanabhan:  RADAR: An In-Building RF-based User Location and Tracking System
    private static final double REF_SIGNAL = -93; //-41
    private static final double PATHLOSS_FACTOR = 2.5; //2.5
    //private static final double REF_DISTANCE = 150; // 1m = -41dbm, 100
    private static final double WALL_ATTENUATION_SIM = 0.2; //3.1 this is per wall pixel on map, so should generally be a low value.
                                                      //We currently only have one attenuation factor for all obstacle types
                                                      //In real life, this could vary depending on wall material.
                                                      //This should be a different value for the simulator and for robots
                                                      //As robots cannot sense how thick a wall is.
    private static final double WALL_ATTENUATION_AGENT = 0.6; //this should probably be higher for agent than for simulator, as agents
                                                            //don't know how thick the walls are generally.
    private static final int MAX_WALLS = 125; //maximum combined wall "thickness" after which attenuation stops making a difference
    private static final double CUTOFF = -93;
    
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
    
    public static double getMaxRange(double REF_DISTANCE) {
        if (REF_SIGNAL <= CUTOFF) return REF_DISTANCE;
        else return (Math.pow(10, (REF_SIGNAL - CUTOFF) / (10 * PATHLOSS_FACTOR)) * REF_DISTANCE);
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
    
    public static boolean isConnected(Environment env, RealAgent ag1, RealAgent ag2) {
        return (signalStrength(Math.min(ag1.getCommRange(), ag2.getCommRange()), env, ag1.getLocation(), 
                ag2.getLocation()) >= CUTOFF);
    }
    
    public static boolean isConnected(OccupancyGrid grid, double agentRange, Point p1, Point p2) {
        return (signalStrength(agentRange, grid, p1, p2) >= CUTOFF);
    }
    
    //<editor-fold defaultstate="collapsed" desc="Signal Strength">
    //For use by simulation
    private static double signalStrength(double agentRange, Environment env, Point p1, Point p2) {
        int numWalls = Math.min(MAX_WALLS, env.numObstaclesOnLine(p1.x, p1.y, p2.x, p2.y));
        double distance = p1.distance(p2);
        
        return (REF_SIGNAL - 10 * PATHLOSS_FACTOR * Math.log10(distance / /*REF_DISTANCE*/agentRange) - numWalls * WALL_ATTENUATION_SIM);
    }
    
    //For use by individual robots
    public static double signalStrength(double agentRange, OccupancyGrid occGrid, Point p1, Point p2) {
        int numWalls = Math.min(MAX_WALLS, occGrid.numPossibleObstaclesOnLine(p1.x, p1.y, p2.x, p2.y));
        double distance = p1.distance(p2);
        
        return (REF_SIGNAL - 10 * PATHLOSS_FACTOR * Math.log10(distance / /*REF_DISTANCE*/agentRange) - numWalls * WALL_ATTENUATION_AGENT);
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
