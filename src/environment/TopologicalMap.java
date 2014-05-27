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

import agents.BasicAgent;
import agents.RealAgent;
import communication.PropModel1;
import config.Constants;
import config.RobotConfig;
import exploration.RVLocation;
import java.awt.Point;
import java.awt.Polygon;
import java.util.HashMap;
import java.util.LinkedList;
import path.Path;
import path.TopologicalNode;

/**
 *
 * @author Victor
 */
public class TopologicalMap {
    private OccupancyGrid occGrid;
    private int skeletonGrid[][];
    private LinkedList<Point> skeletonPoints;
    private LinkedList<Point> keyPoints;
    private LinkedList<Point> borderPoints;
    private int areaGrid[][];
    private HashMap<Integer, TopologicalNode> topologicalNodes;
    private Path path;
    
    private int skeletonGridBorder[][];
    private LinkedList<Point> skeletonPointsBorder;
    private LinkedList<Point> keyPointsBorder;
    private LinkedList<Point> secondKeyPointsBorder;
    
    public TopologicalMap(OccupancyGrid occGrid)
    {
        setGrid(occGrid);
        path = new Path();
    }
    
    public void setGrid(OccupancyGrid occGrid)
    {
        this.occGrid = occGrid;
    }
    
    public OccupancyGrid getGrid()
    {
        return this.occGrid;
    }
    
    public void generateSkeleton()
    {
        skeletonGrid = Skeleton.skeletonize(Skeleton.findSkeleton(occGrid));
        skeletonPoints = Skeleton.gridToList(skeletonGrid);
    }
    
    public void findKeyPoints()
    {
        keyPoints = Skeleton.findKeyPoints(skeletonGrid, occGrid);
    }
    
    public LinkedList<Point> getSkeletonPoints() {
        return skeletonPoints;
    }
    
    public LinkedList<Point> getSkeletonPointsBorder() {
        return skeletonPointsBorder;
    }
    
    public LinkedList<Point> getKeyPoints() {
        return keyPoints;
    }
    
    public HashMap<Integer, TopologicalNode> getTopologicalNodes()
    {
        return topologicalNodes;
    }
    
    public void generateBorderPoints() {
        borderPoints = Skeleton.findKeyAreaBorders(areaGrid);
    }
    
    public LinkedList<Point> getBorderPoints() {
        if (borderPoints == null)
            generateBorderPoints();
        return borderPoints;
    }
    
    public int[][] getSkeletonGrid() {
        return skeletonGrid;
    }
    
    public void generateKeyAreas()
    {
        areaGrid = Skeleton.fillKeyAreas(occGrid, keyPoints);
        topologicalNodes = new HashMap<Integer, TopologicalNode>();
        
        int index = 0;
        for (Point p: keyPoints)
        {
            index++;
            topologicalNodes.put(index, new TopologicalNode(index, p));
        }
        topologicalNodes.put(Constants.UNEXPLORED_NODE_ID, new TopologicalNode(Constants.UNEXPLORED_NODE_ID, new Point(-1, -1)));
        
        //find node neighbours
        long realtimeStart = System.currentTimeMillis();
        //System.out.println("Generating node neighbour relationships...");
        generateBorderPoints();
        for (Point p: getBorderPoints())
        {
            if (areaGrid[p.x][p.y] > 0)
            {
                int curCell = areaGrid[p.x][p.y];                
                TopologicalNode node = topologicalNodes.get(curCell);
                for (int i = -1; i < 2; i++)
                {
                    for (int j = -1; j < 2; j++)
                    {
                        if ((areaGrid[p.x+i][p.y+j] != curCell) && (areaGrid[p.x+i][p.y+j] > 0))
                        {
                            TopologicalNode neighbourNode = topologicalNodes.get(areaGrid[p.x+i][p.y+j]);
                            if (!node.getListOfNeighbours().contains(neighbourNode))
                            {
                                if ((curCell != Constants.UNEXPLORED_NODE_ID)
                                        && (areaGrid[p.x+i][p.y+j] != Constants.UNEXPLORED_NODE_ID))
                                {
                                    Path pathToNode = new Path();
                                    pathToNode.setStartPoint(node.getPosition());
                                    pathToNode.setGoalPoint(neighbourNode.getPosition());
                                    pathToNode.getAStarPath(occGrid, node.getPosition(), neighbourNode.getPosition(), false);
                                    //pathToNode.getJumpPath(occGrid, node.getPosition(), neighbourNode.getPosition(), false);
                                    node.addNeighbour(neighbourNode, pathToNode);                                
                                    neighbourNode.addNeighbour(node, pathToNode.generateReversePath());
                                } else
                                {
                                    node.addNeighbour(neighbourNode, null);
                                    neighbourNode.addNeighbour(node, null);
                                }
                            }
                        }  
                    }
                }
            }
        }
        //System.out.println("Done, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        
    }
    
    public int[][] getAreaGrid()
    {
        return areaGrid;
    }
    
        public void setPathStart(Point start)
    {
        path.setStartPoint(start);
    }
    
    public void setPathGoal(Point goal)
    {
        path.setGoalPoint(goal);
    }
    
    public Point getPathStart()
    {
        return path.getStartPoint();
    }
    
    public Point getPathGoal()
    {
        return path.getGoalPoint();
    }
    
    public void getAStarPath()
    {
        path = new Path(occGrid, path.getStartPoint(), path.getGoalPoint(), false, false);
    }
    
    public void getJumpPath()
    {
        path = new Path(occGrid, path.getStartPoint(), path.getGoalPoint(), false, true);
    }
    
    public void getTopologicalPath()
    {
        path = new Path(occGrid, this, path.getStartPoint(), path.getGoalPoint(), false);
    }
    
    public Path getPath() {
        return path;
    }
    
    public LinkedList<Point> getKeyPointsBorder()
    {
        return keyPointsBorder;
    }
    
    public LinkedList<Point> getSecondKeyPointsBorder()
    {
        return secondKeyPointsBorder;
    }
    
    
    //<editor-fold defaultstate="collapsed" desc="RV through walls stuff">
    public void generateSkeletonNearBorders()
    {
        skeletonGridBorder = Skeleton.skeletonize(Skeleton.findSkeletonNearBorders(occGrid));
        skeletonPointsBorder = Skeleton.gridToList(skeletonGridBorder);
    }
    
    public void findKeyPointsBorder()
    {
        keyPointsBorder = Skeleton.findBorderRVPoints(skeletonGridBorder, occGrid);
    }
    
    public void findSecondKeyPointsBorder(Point goal, RealAgent agent)
    {
        secondKeyPointsBorder = Skeleton.findSecondBorderRVPoints(keyPointsBorder, agent, goal);
    }
    
    public RVLocation findNearestBorderKeyPoint(Point otherPoint, RealAgent agent) {
        RVLocation result = new RVLocation(otherPoint);
        
        double minDistance = 10000;
        int minElement = -1;
        
        for (int i = 1; i < keyPointsBorder.size(); i++) {
            if (!keyPointsBorder.get(i).equals(secondKeyPointsBorder.get(i)))
            {
                double newDistance = agent.calculatePath(keyPointsBorder.get(i), otherPoint).getLength();
                if ((minElement == -1) || (newDistance < minDistance)) {
                    minElement = i;
                    minDistance = newDistance;
                }
            }
        }
        
        if (minElement != -1)
        {
            System.out.println(agent.toString() + " evaluating " + keyPointsBorder.get(minElement) + " - " + secondKeyPointsBorder.get(minElement));
            result.setChildLocation(keyPointsBorder.get(minElement));
            result.setParentLocation(secondKeyPointsBorder.get(minElement));
            
            //strict rule - too strict?
            /*
            Path Base2OldRV = agent.calculatePath(agent.getTeammate(Constants.BASE_STATION_ID).getLocation(), 
                    agent.getParentRendezvous().getParentLocation());
            
            Path Base2NewRV = agent.calculatePath(agent.getTeammate(Constants.BASE_STATION_ID).getLocation(), 
                    result.getParentLocation());
            
            if (!Base2OldRV.found || !Base2NewRV.found || (Base2OldRV.getLength() < Base2NewRV.getLength()))
            {
                result.setChildLocation(agent.getParentRendezvous().getChildLocation());
                result.setParentLocation(agent.getParentRendezvous().getParentLocation());
                return result;
            }
            
            Point frontierCentre = null;
            if(agent.getLastFrontier() != null)
                frontierCentre = agent.getLastFrontier().getClosestPoint(agent.getLocation(), agent.getOccupancyGrid());
            else
                frontierCentre = agent.getLocation();
            if (frontierCentre != null)
            {                
                Path Front2OldRV = agent.calculatePath(frontierCentre, agent.getParentRendezvous().getChildLocation());
                Path Front2NewRV = agent.calculatePath(frontierCentre, result.getChildLocation());
                
                if (!Front2NewRV.found || !Front2NewRV.found || (Front2OldRV.getLength() < Front2NewRV.getLength()))
                {
                    result.setChildLocation(agent.getParentRendezvous().getChildLocation());
                    result.setParentLocation(agent.getParentRendezvous().getParentLocation());
                    return result;
                }                
            } else
            {
                result.setChildLocation(agent.getParentRendezvous().getChildLocation());
                result.setParentLocation(agent.getParentRendezvous().getParentLocation());
                return result;
            }
            
            return result;*/
            
            double relayPathLengthOld = 0;
            double relayPathLengthNew = 0;
            double explorerPathLengthOld = 0;
            double explorerPathLengthNew = 0;
            
            //<editor-fold defaultstate="collapsed" desc="let's find the point, where RV will actually communicate with base">
            Point baseLoc = agent.getTeammate(Constants.BASE_STATION_ID).getLocation();
            Point relayLoc = agent.getParentTeammate().getLocation();
            Point baseComm = baseLoc;
            
            Polygon commPoly = PropModel1.getRangeForRV(occGrid,
                    new BasicAgent(0, "", 0, baseLoc.x, baseLoc.y, 0, 0, 400, 0,
                            RobotConfig.roletype.Relay, 0, 0, 0)
            );
            
            LinkedList<Point> candidatePoints = new LinkedList<Point>();
            //for(Point p : ExplorationImage.polygonPoints(commPoly))
            for (int i = 0; i < commPoly.npoints; i++)
            {
                Point p = new Point(commPoly.xpoints[i], commPoly.ypoints[i]);
                if (occGrid.freeSpaceAt(p.x, p.y) /*&& !env.directLinePossible(firstRV.x, firstRV.y, p.x, p.y)*/)
                {                    
                    if (occGrid.directLinePossible(baseLoc.x, baseLoc.y, p.x, p.y))
                        candidatePoints.add(p);
                }
            }
            
            double minBaseRelayDistance = agent.calculatePath(baseLoc, relayLoc).getLength();
            
            for (Point p: candidatePoints)
            {
                double distance = agent.calculatePath(p, relayLoc).getLength();
                if (distance < minBaseRelayDistance)
                {
                    minBaseRelayDistance = distance;
                    baseComm = p;
                }
            }
            //</editor-fold>
            
            //<editor-fold defaultstate="collapsed" desc="let's find the point, where RV will actually communicate with explorer">
            Point oldRV = agent.getParentRendezvous().getParentLocation();
            Point oldComm = oldRV;
            
            /*commPoly = PropModel1.getRangeForRV(occGrid,
                    new BasicAgent(0, "", 0, oldRV.x, oldRV.y, 0, 0, 400, 0,
                            RobotConfig.roletype.Relay, 0, 0, 0)
            );
            
            LinkedList<Point> candidatePoints2 = new LinkedList<Point>();
            //for(Point p : ExplorationImage.polygonPoints(commPoly))
            for (int i = 0; i < commPoly.npoints; i++)
            {
                Point p = new Point(commPoly.xpoints[i], commPoly.ypoints[i]);
                if (occGrid.freeSpaceAt(p.x, p.y))
                {                    
                    if (occGrid.directLinePossible(oldRV.x, oldRV.y, p.x, p.y))
                        candidatePoints2.add(p);
                }
            }
            
            double minRVRelayDistance = agent.calculatePath(baseLoc, oldComm).getLength();
            
            for (Point p: candidatePoints2)
            {
                double distance = agent.calculatePath(p, baseLoc).getLength();
                if (distance < minRVRelayDistance)
                {
                    minRVRelayDistance = distance;
                    oldComm = p;
                }
            }*/
            //</editor-fold>
            
            Path Relay2Base = agent.calculatePath(agent.getParentTeammate().getLocation(), baseComm);

            Path Base2OldRV = agent.calculatePath(baseComm, oldComm);
            System.out.println(agent.toString() + " baseComm: " + baseComm + ", oldComm: " + oldComm + 
                    ", baseLoc: " + baseLoc + ", baseRV: " + oldRV);
            
            if (!Base2OldRV.found)
            {
                System.out.println(agent.toString() + " !!!! Base2OldRv not found, baseComm: " + baseComm + ", oldComm: " + oldComm);
                Base2OldRV = agent.calculatePath(baseLoc, oldRV);
            }
            
            Path Base2NewRV = agent.calculatePath(baseComm, 
                    result.getParentLocation());
            
            if (!Relay2Base.found || !Base2OldRV.found || !Base2NewRV.found)
            {
                result.setChildLocation(agent.getParentRendezvous().getChildLocation());
                result.setParentLocation(agent.getParentRendezvous().getParentLocation());
                System.out.println(agent.toString() + " using conventional RV point !!!!! Relay1Base.found is " + Relay2Base.found + " Base2OldRV.found is " + Base2OldRV.found + " Base2NewRV.found is " + Base2NewRV.found);
                System.out.println(agent.toString() + " baseComm: " + baseComm + ", oldComm: " + oldComm);
                return result;
            }
            
            relayPathLengthOld = Relay2Base.getLength() + Base2OldRV.getLength();
            relayPathLengthNew = Relay2Base.getLength() + Base2NewRV.getLength();
            
            //<editor-fold defaultstate="collapsed" desc="Check time for explorer to reach frontier, to make sure he has time to explore before returning">
            Point frontierCentre = null;
            if(agent.getLastFrontier() != null)
                frontierCentre = agent.getLastFrontier().getClosestPoint(agent.getLocation(), agent.getOccupancyGrid());
            else
            {
                System.out.println(agent + " setting frontierCentre to agentLoc");
                frontierCentre = agent.getLocation();
            }
            if (frontierCentre != null)
            {                
                Path Explorer2Frontier = agent.calculatePath(agent.getLocation(), frontierCentre);
                Path Front2OldRV = agent.calculatePath(frontierCentre, agent.getParentRendezvous().getChildLocation());
                Path Front2NewRV = agent.calculatePath(frontierCentre, result.getChildLocation());
                
                if (!Explorer2Frontier.found || !Front2NewRV.found || !Front2NewRV.found)
                {
                    System.out.println(agent.toString() + " using conventional RV point !!!!! Explorer2Frontier.found is " + Explorer2Frontier.found + " Front2NewRV.found is " + Front2NewRV.found + " Front2NewRV.found is " + Front2NewRV.found);
                    result.setChildLocation(agent.getParentRendezvous().getChildLocation());
                    result.setParentLocation(agent.getParentRendezvous().getParentLocation());
                    return result;
                }
                
                explorerPathLengthOld = Explorer2Frontier.getLength() + Front2OldRV.getLength() 
                        + Constants.FRONTIER_MIN_EXPLORE_TIME * Constants.DEFAULT_SPEED;
                explorerPathLengthNew = Explorer2Frontier.getLength() + Front2NewRV.getLength()
                        + Constants.FRONTIER_MIN_EXPLORE_TIME * Constants.DEFAULT_SPEED;
                System.out.println(agent.toString() + " frontierCentre: " + frontierCentre + ", agentLocation: " + agent.getLocation() + ", Exp2Front: " + Explorer2Frontier.getLength() + ", Front2OldRV: " + Front2OldRV.getLength() + ", Front2NewRV: " + Front2NewRV.getLength() + ", explorerPathLengthOld: " + explorerPathLengthOld + ", explorerPathLengthNew: " + explorerPathLengthNew);
            } else
            {
                result.setChildLocation(agent.getParentRendezvous().getChildLocation());
                result.setParentLocation(agent.getParentRendezvous().getParentLocation());
                System.out.println(agent.toString() + " using conventional RV point !!!!! frontierCenter is null");
                return result;
                //explorerPathLengthOld = agent.calculatePath(agent.getLocation(), agent.getParentRendezvous().getChildLocation()).getLength();
                //explorerPathLengthNew = agent.calculatePath(agent.getLocation(), result.getChildLocation()).getLength();
            }
            //</editor-fold>
            
            double overallTimeOld = Math.max(explorerPathLengthOld, relayPathLengthOld);
            double overallTimeNew = Math.max(explorerPathLengthNew, relayPathLengthNew);

            if (overallTimeNew > overallTimeOld * Constants.MIN_RV_THROUGH_WALL_ACCEPT_RATIO)
            {
                System.out.println(agent.toString() + " using conventional RV point");
                System.out.println(agent.toString() + " overallTimeNew: " + overallTimeNew + ", overallTimeOld: " + overallTimeOld + ", relay2Base: " + Relay2Base.getLength() + ", Base2OldRV: " + Base2OldRV.getLength() + ", Base2NewRV: " + Base2NewRV.getLength() + ", explorerPathLengthOld: " + explorerPathLengthOld + ", explorerPathLengthNew: " + explorerPathLengthNew);
                result.setChildLocation(agent.getParentRendezvous().getChildLocation());
                result.setParentLocation(agent.getParentRendezvous().getParentLocation());
            } else
            {
                System.out.println(agent.toString() + " is meeting the relay through the wall!!");
                System.out.println(agent.toString() + " overallTimeNew: " + overallTimeNew + ", overallTimeOld: " + overallTimeOld + ", relay2Base: " + Relay2Base.getLength() + ", Base2OldRV: " + Base2OldRV.getLength() + ", Base2NewRV: " + Base2NewRV.getLength() + ", explorerPathLengthOld: " + explorerPathLengthOld + ", explorerPathLengthNew: " + explorerPathLengthNew);
            }
        } else {
            System.out.println(agent.toString() + " !!!!! no rvpairs found?!");
        }
        
        return result;
    }
            
    
    //</editor-fold>
}
