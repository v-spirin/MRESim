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

package environment;

import agents.RealAgent;
import config.SimConstants;
import java.awt.Point;
import java.awt.Rectangle;
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

    private int skeletonGridBorder[][];
    private LinkedList<Point> skeletonPointsBorder;
    private LinkedList<Point> keyPointsBorder;
    private LinkedList<Point> secondKeyPointsBorder;

    //cached paths between nodes; first param is two points, start and finish
    private static HashMap<Rectangle, Path> pathCache = new HashMap<Rectangle, Path>();
    private LinkedList<Point> junctionPoints;

    public TopologicalMap(OccupancyGrid occGrid) {
        setGrid(occGrid);
        update(true);
    }

    public final void update(boolean force) {
        if (!force && occGrid.getMapCellsChanged() < SimConstants.MAP_CHANGED_THRESHOLD) {
            return;
        }
        skeletonGrid = null;
        skeletonPoints = null;
        keyPoints = null;
        borderPoints = null;
        areaGrid = null;
        topologicalNodes = null;

        skeletonGridBorder = null;
        skeletonPointsBorder = null;
        keyPointsBorder = null;
        secondKeyPointsBorder = null;
        junctionPoints = null;

    }

    public final void setGrid(OccupancyGrid occGrid) {
        this.occGrid = occGrid;
    }

    public OccupancyGrid getGrid() {
        return this.occGrid;
    }

    private void generateSkeleton() {
        skeletonGrid = occGrid.getSkeleton();
        skeletonPoints = Skeleton.gridToList(skeletonGrid);
    }

    /**
     * Calculates the keyPoints of this map. KeyPoints are basically nodes of the skeleton. Call
     * generateSkeleton first!
     */
    private void findKeyPoints() {
        keyPoints = Skeleton.findKeyPoints(getSkeletonGrid(), occGrid, false, 40);
    }

    /**
     * Gives a list of Points at the junctions of the skeleton
     *
     * @return
     */
    public LinkedList<Point> getJunctionPoints() {
        if (junctionPoints == null) {
            findJunctionPoints();
        }
        return junctionPoints;
    }

    private void findJunctionPoints() {
        junctionPoints = Skeleton.findJunctionPoints(getSkeletonGrid(), occGrid, false);
    }

    public LinkedList<Point> getSkeletonPoints() {
        if (skeletonPoints == null) {
            generateSkeleton();
        }
        return skeletonPoints;
    }

    public LinkedList<Point> getSkeletonPointsBorder() {
        if (skeletonPointsBorder == null) {
            generateSkeletonNearBorders();
        }
        return skeletonPointsBorder;
    }

    public LinkedList<Point> getKeyPoints() {
        if (keyPoints == null) {
            findKeyPoints();
        }
        return keyPoints;
    }

    public HashMap<Integer, TopologicalNode> getTopologicalNodes(boolean update) {
        if (topologicalNodes == null || update) {
            generateKeyAreas();
        }
        return topologicalNodes;
    }

    private void generateBorderPoints() {
        borderPoints = Skeleton.findKeyAreaBorders(getAreaGrid());
    }

    public LinkedList<Point> getBorderPoints() {
        if (borderPoints == null) {
            generateBorderPoints();
        }
        return borderPoints;
    }

    public int[][] getSkeletonGrid() {
        if (skeletonGrid == null) {
            generateSkeleton();
        }
        return skeletonGrid;
    }

    private void generateKeyAreas() {
        // declare topological nodes (we will define relations between them later)
        // each node has one keypoint which is rougly in the center of the node region
        // this keypoint is used to pre-calculate occupancy grid paths between nodes.
        topologicalNodes = new HashMap<Integer, TopologicalNode>();

        int index = 0;
        for (Point p : getKeyPoints()) {
            index++;
            if (index == SimConstants.UNEXPLORED_NODE_ID) {
                index++;
            }
            topologicalNodes.put(index, new TopologicalNode(index, p));
        }
        topologicalNodes.put(SimConstants.UNEXPLORED_NODE_ID, new TopologicalNode(SimConstants.UNEXPLORED_NODE_ID, new Point(-1, -1)));

        // calculate the areas for each node
        areaGrid = Skeleton.fillKeyAreas(occGrid, getKeyPoints(), topologicalNodes);
        //find node neighbours
        generateBorderPoints();

        for (Point p : borderPoints) {
            if (areaGrid[p.x][p.y] > 0) {
                int curCell = areaGrid[p.x][p.y];
                TopologicalNode node = topologicalNodes.get(curCell);
                for (int i = -1; i < 2; i++) {
                    for (int j = -1; j < 2; j++) {
                        if ((areaGrid[p.x + i][p.y + j] != curCell) && (areaGrid[p.x + i][p.y + j] > 0)) {
                            TopologicalNode neighbourNode = topologicalNodes.get(areaGrid[p.x + i][p.y + j]);
                            if (!node.getListOfNeighbours().contains(neighbourNode)) {
                                if ((curCell != SimConstants.UNEXPLORED_NODE_ID)
                                        && (areaGrid[p.x + i][p.y + j] != SimConstants.UNEXPLORED_NODE_ID)) {
                                    Path pathToNode;
                                    //check path cache
                                    Rectangle pathCoords = new Rectangle(node.getPosition().x, node.getPosition().y,
                                            neighbourNode.getPosition().x, neighbourNode.getPosition().y);
                                    if (pathCache.containsKey(pathCoords)) {
                                        pathToNode = pathCache.get(pathCoords);
                                    } else {
                                        pathToNode = new Path(occGrid, (Point) node.getPosition(), (Point) neighbourNode.getPosition(), false, true);
                                        if (!pathToNode.getStartPoint().equals(node.getPosition())
                                                || !pathToNode.getGoalPoint().equals(neighbourNode.getPosition())) {
                                            System.err.println("CATASTROPHIC ERROR!! Path from (" + node.getPosition().x + "," + node.getPosition().y + ") to (" + neighbourNode.getPosition().x + "," + neighbourNode.getPosition().y + "). Path start = (" + pathToNode.getStartPoint().x + "," + pathToNode.getStartPoint().y + "), path goal = (" + pathToNode.getGoalPoint().x + "," + pathToNode.getGoalPoint().y + ")");
                                        }
                                        pathCache.put(pathCoords, pathToNode);
                                        Path reversePath = pathToNode.getReversePath();
                                        Rectangle reversePathCoords = new Rectangle(neighbourNode.getPosition().x, neighbourNode.getPosition().y,
                                                node.getPosition().x, node.getPosition().y);
                                        pathCache.put(reversePathCoords, reversePath);
                                    }
                                    node.addNeighbour(neighbourNode, pathToNode);
                                    neighbourNode.addNeighbour(node, pathToNode.getReversePath());
                                } else {

                                    node.addNeighbour(neighbourNode, null);
                                    neighbourNode.addNeighbour(node, null);
                                    //Should not be neccessary!
                                    /*if (areaGrid[p.x + i][p.y + j] == SimConstants.UNEXPLORED_NODE_ID) {
                                        node.getCellList().stream().forEach((nodeCell) -> {
                                            occGrid.unsetFinalTopologicalMapCell(nodeCell.x, nodeCell.y);
                                        });
                                    } else {
                                        neighbourNode.getCellList().stream().forEach((nodeCell) -> {
                                            occGrid.unsetFinalTopologicalMapCell(nodeCell.x, nodeCell.y);
                                        });
                                    }*/
                                }
                            }
                        }
                    }
                }
            }
        }

    }

    public int[][] getAreaGrid() {
        if (areaGrid == null) {
            generateKeyAreas();
        }
        return areaGrid;
    }

    public LinkedList<Point> getKeyPointsBorder() {
        if (keyPointsBorder == null) {
            findKeyPointsBorder();
        }
        return keyPointsBorder;
    }

    public LinkedList<Point> getSecondKeyPointsBorder(Point goal, RealAgent agent) {
        if (secondKeyPointsBorder == null) {
            findSecondKeyPointsBorder(goal, agent);
        }
        return secondKeyPointsBorder;
    }

    //RV through walls stuff
    private void generateSkeletonNearBorders() {
        skeletonGridBorder = Skeleton.findSkeletonNearBorders(occGrid);
        skeletonPointsBorder = Skeleton.gridToList(skeletonGridBorder);
    }

    private void findKeyPointsBorder() {
        keyPointsBorder = Skeleton.findBorderRVPoints(skeletonGridBorder, occGrid);
    }

    private void findSecondKeyPointsBorder(Point goal, RealAgent agent) {
        secondKeyPointsBorder = Skeleton.findSecondBorderRVPoints(keyPointsBorder, agent, goal);
    }

    public int getTopologicalArea(Point p) {
        return areaGrid[p.x][p.y];
    }
}
