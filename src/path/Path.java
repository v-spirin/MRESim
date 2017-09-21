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

package path;

import config.SimConstants;
import environment.Environment;
import environment.OccupancyGrid;
import environment.TopologicalMap;
import gui.ShowSettings.ShowSettingsAgent;
import java.awt.Point;
import java.io.File;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import simulator.ExplorationImage;

/**
 *
 * @author julh
 */
public class Path {

    public boolean found;
    private Point startPoint;
    private Point goalPoint;
    private OccupancyGrid grid;
    private boolean limit;

    List<Point> reversePathPoints;
    List<Point> pathPoints;
    LinkedList<TopologicalNode> pathNodesReverse;
    LinkedList<TopologicalNode> pathNodes;
    LinkedList<Point> allPathPixels;
    LinkedList<Path> pathSections;
    double length;
    private TopologicalMap tMap;
    private int currentPoint = 0;
    private boolean jump;
    private boolean valid = true;
    private boolean exact;

    private Path() {

    }

    public Path(OccupancyGrid agentGrid, Point startpoint, Point endpoint, boolean limit, boolean jump, boolean exact) {
        this.startPoint = startpoint;
        this.goalPoint = endpoint;
        this.limit = limit;
        this.grid = agentGrid;
        this.jump = jump = false;
        this.tMap = null;
        this.pathPoints = new LinkedList<Point>();
        this.reversePathPoints = null;
        this.length = 0;
        this.found = false;
        this.allPathPixels = null;
        this.exact = exact;
        if (!jump) {
            this.found = calculateAStarPath(exact);
        } else {
            this.found = calculateJumpPath();
            if (!this.found) {
                this.found = calculateAStarPath(exact);
            }
        }
    }

    public Path(OccupancyGrid agentGrid, TopologicalMap tMap,
            Point startpoint, Point endpoint, boolean limit, boolean jump, boolean exact) throws IllegalStateException {
        this.startPoint = startpoint;
        this.goalPoint = endpoint;
        this.tMap = tMap;
        this.grid = agentGrid;
        this.pathPoints = new LinkedList<Point>();
        this.pathNodes = new LinkedList<TopologicalNode>();
        this.pathNodesReverse = new LinkedList<TopologicalNode>();
        this.pathSections = new LinkedList<>();

        this.limit = limit;
        this.jump = jump = false;
        this.found = false;
        this.reversePathPoints = null;
        this.allPathPixels = null;
        this.length = 0;
        this.exact = exact;

        calcuateTopoPath();
    }

    private boolean calcuateTopoPath() throws IllegalStateException {
        //Find which area the agent starts from/want to go to
        int[][] areaGrid = tMap.getAreaGrid();
        HashMap<Integer, TopologicalNode> topologicalNodes = tMap.getTopologicalNodes(false);
        TopologicalNode startNode = topologicalNodes.get(areaGrid[startPoint.x][startPoint.y]);
        TopologicalNode goalNode = topologicalNodes.get(areaGrid[goalPoint.x][goalPoint.y]);
        if ((startNode == null) || (goalNode == null)) {
            topologicalNodes = tMap.getTopologicalNodes(true);
            areaGrid = tMap.getAreaGrid();
            startNode = topologicalNodes.get(areaGrid[startPoint.x][startPoint.y]);
            goalNode = topologicalNodes.get(areaGrid[goalPoint.x][goalPoint.y]);
            if ((startNode == null) || (goalNode == null)) {
                //System.err.println(this + "Error: startNode: " + startNode + " , goalNode: " + goalNode);
                throw new IllegalStateException(this + "Error: startNode: " + startNode + " , goalNode: " + goalNode);
            }
        }
        if (startNode.getID() == SimConstants.UNEXPLORED_NODE_ID || goalNode.getID() == SimConstants.UNEXPLORED_NODE_ID) {
            tMap.update(true);
            topologicalNodes = tMap.getTopologicalNodes(true);
            areaGrid = tMap.getAreaGrid();
            startNode = topologicalNodes.get(areaGrid[startPoint.x][startPoint.y]);
            goalNode = topologicalNodes.get(areaGrid[goalPoint.x][goalPoint.y]);
            if (startNode == null || goalNode == null) {
                return true;
            }
            if (startNode.getID() == SimConstants.UNEXPLORED_NODE_ID || goalNode.getID() == SimConstants.UNEXPLORED_NODE_ID) {
                //System.err.println(this + "Error after TopoNodesUpdate: startNode: " + startNode + " , goalNode: " + goalNode);
                throw new IllegalStateException(this + "Error after TopoNodesUpdate: startNode: " + startNode + " , goalNode: " + goalNode);
            }
        }
        if (!startNode.equals(goalNode) && (startNode.getListOfNeighbours().isEmpty())) {
            //System.err.println(this + "Error: startNode: " + startNode + " has no neighbors, but is not the only node");
            throw new IllegalStateException(this + "Error: startNode: " + startNode + " has no neighbors, but is not the only node");
        }
        if (!startNode.equals(goalNode) && (goalNode.getListOfNeighbours().isEmpty())) {
            //System.err.println(this + "Error: goalNode: " + goalNode + " has no neighbors, but is not the only node");
            throw new IllegalStateException(this + "Error: goalNode: " + goalNode + " has no neighbors, but is not the only node");
        }
        if (startNode.equals(goalNode)) {
            if (startPoint.equals(goalPoint)) {
                this.pathPoints.add(goalPoint);
                this.length = 1;
            }
            //Path inside an area, normal planning
            if (!jump) {
                this.found = calculateAStarPath(exact);
            } else {
                this.found = calculateJumpPath();
                if (!this.found) {
                    this.found = calculateAStarPath(exact);
                }
            }
            return true;
        }
        // This is the standard-case:
        boolean foundNodePath = calculateAStarNodePath(startNode, goalNode);
        if (!foundNodePath || pathNodes.size() < 3) {
            //Again normal planning as there was no path found using nodes
            if (!jump) {
                this.found = calculateAStarPath(exact);
            } else {
                this.found = calculateJumpPath();
                if (!this.found) {
                    this.found = calculateAStarPath(exact);
                }
            }
        } else {
            //First find path from startPoint to startNode

            //plan path to first neighbor, should reduce starting in the wrong direction
            Path startPath = new Path(grid, startPoint, pathNodes.get(1).getPosition(), limit, jump, exact);
            this.pathPoints.addAll(startPath.getPoints());
            pathSections.add(startPath);
            //Second add path from StartNode to goalNode, this is just adding precomputed pathes
            for (int i = 1; i < pathNodes.size() - 2; i++) {
                Path tempPath = pathNodes.get(i).getPathToNeighbour(pathNodes.get(i + 1));
                if (!tempPath.isValid()) {
                    tempPath.repairPath();
                }
                this.pathPoints.addAll(tempPath.getPoints());
                pathSections.add(tempPath);
            }
            //Third find path from goalNode-1 to goalPoint (otherwise the agend might go to the goal-node-center and backwards to the goalPoint
            Path goalPath = new Path(grid, pathNodes.get(pathNodes.size() - 2).getPosition(), goalPoint, limit, jump, exact);
            this.pathPoints.addAll(goalPath.getPoints());
            pathSections.add(goalPath);
            if (startPath.found && goalPath.found) {
                this.found = true;
            }
        }
        return false;
    }

    final public boolean calculateAStarNodePath(TopologicalNode startNode, TopologicalNode goalNode) {
        //implementing http://en.wikipedia.org/wiki/A*#Pseudocode
        List<TopologicalNode> closedSet = new LinkedList<TopologicalNode>();
        List<TopologicalNode> openSet = new LinkedList<TopologicalNode>();

        openSet.add(startNode);

        HashMap<TopologicalNode, TopologicalNode> came_from = new HashMap<TopologicalNode, TopologicalNode>();
        HashMap<TopologicalNode, Double> g_score = new HashMap<TopologicalNode, Double>();
        HashMap<TopologicalNode, Double> f_score = new HashMap<TopologicalNode, Double>();
        TopologicalNode current;
        double tentative_g_score;

        g_score.put(startNode, 0.0);
        f_score.put(startNode, g_score.get(startNode) + heuristicCostEstimate(startNode, goalNode));

        while (!openSet.isEmpty()) {
            int current_index = getLowestScoreInList(openSet, f_score);
            current = openSet.get(current_index);
            if (current.equals(goalNode)) {
                reconstructNodePath(came_from, goalNode);
                return true;
            }

            openSet.remove(current_index);
            closedSet.add(current);

            for (TopologicalNode neighbour : current.getListOfNeighbours()) {
                if (neighbour.getID() == SimConstants.UNEXPLORED_NODE_ID || current.getPathToNeighbour(neighbour) == null || closedSet.contains(neighbour)) {
                    continue;
                }
                tentative_g_score = g_score.get(current) + current.getPathToNeighbour(neighbour).getLength();

                if (!openSet.contains(neighbour) || !g_score.containsKey(neighbour) || (tentative_g_score < g_score.get(neighbour))) {
                    if (!openSet.contains(neighbour)) {
                        openSet.add(neighbour);
                    }
                    if (!current.equals(neighbour)) {
                        came_from.put(neighbour, current);
                    } else {
                        System.err.println("How can this be??");
                    }
                    g_score.put(neighbour, tentative_g_score);
                    f_score.put(neighbour, g_score.get(neighbour) + heuristicCostEstimate(neighbour, goalNode));
                }
            }

        }
        return false;
    }

    private void reconstructNodePath(HashMap<TopologicalNode, TopologicalNode> came_from, TopologicalNode current_node) {
        while (came_from.containsKey(current_node) && (came_from.get(current_node) != current_node)) {
            pathNodesReverse.add(current_node);
            current_node = came_from.get(current_node);
        }
        pathNodesReverse.add(current_node);
        for (int i = pathNodesReverse.size() - 1; i >= 0; i--) {
            pathNodes.add(pathNodesReverse.get(i));
        }
    }

    final public boolean calculateAStarPath(boolean exact) {
        int stepSize = SimConstants.STEP_SIZE;
        if (exact) {
            stepSize = 1;
        }
        pathPoints = new LinkedList<Point>();
        reversePathPoints = new LinkedList<Point>();

        long realtimeStart = System.currentTimeMillis();
        if ((goalPoint.x == 0) && (goalPoint.y == 0)) //something went really wrong and it takes forever to calculate/fail
        {
            return false;
        }

        //implementing http://en.wikipedia.org/wiki/A*#Pseudocode
        List<Point> closedSet = new LinkedList<Point>();
        List<Point> openSet = new LinkedList<Point>();

        openSet.add(startPoint);

        HashMap<Point, Point> came_from = new HashMap<Point, Point>();
        HashMap<Point, Double> g_score = new HashMap<Point, Double>();
        HashMap<Point, Double> f_score = new HashMap<Point, Double>();
        Point current;
        double tentative_g_score;

        g_score.put(startPoint, 0.0);
        f_score.put(startPoint, g_score.get(startPoint) + heuristicCostEstimate(startPoint, goalPoint));

        boolean limit_hit = false;

        while (!openSet.isEmpty()) {
            long time_elapsed = System.currentTimeMillis() - realtimeStart;
            int maxTime = SimConstants.MAX_PATH_SEARCH_TIME;
            if (exact) {
                maxTime = 5 * SimConstants.MAX_PATH_SEARCH_TIME;
            }
            if ((time_elapsed > maxTime) /*&& (limit)*/) {
                System.err.println("Took too long (A*), startpoint is " + startPoint.toString()
                        + ", endpoint is " + goalPoint.toString() + "time elapsed: " + time_elapsed + "ms.");

                outputPathError();
                limit_hit = true;
                break;
            }
            int current_index = getLowestScoreInList(openSet, f_score);
            current = openSet.get(current_index);
            if (current.distance(goalPoint) <= SimConstants.STEP_SIZE * 2) {
                came_from.put(goalPoint, current);
                reconstructPath(came_from, goalPoint);
                break;
            }

            openSet.remove(current_index);
            closedSet.add(current);

            for (Point neighbour : neighbours(current, stepSize)) {
                if (closedSet.contains(neighbour)) {
                    continue;
                }
                tentative_g_score = g_score.get(current) + current.distance(neighbour);

                if (!openSet.contains(neighbour) || !g_score.containsKey(neighbour) || (tentative_g_score < g_score.get(neighbour))) {
                    if (!openSet.contains(neighbour)) {
                        openSet.add(neighbour);
                    }
                    came_from.put(neighbour, current);
                    g_score.put(neighbour, tentative_g_score);
                    f_score.put(neighbour, g_score.get(neighbour) + heuristicCostEstimate(neighbour, goalPoint));
                }
            }
        }
        recalcLength();

        return !limit_hit && testPath(true);
    }

    private void reconstructPath(HashMap<Point, Point> came_from, Point current_node) {
        while (came_from.containsKey(current_node) && (came_from.get(current_node) != current_node)) {
            pathPoints.add(current_node);
            current_node = came_from.get(current_node);
        }
        pathPoints.add(current_node);
        for (int i = pathPoints.size() - 1; i >= 0; i--) {
            reversePathPoints.add(pathPoints.get(i));
        }
        reverse();
        found = true;
        recalcLength();
        testPath(true);
    }

    final public boolean calculateJumpPath() {
        pathPoints = new LinkedList<Point>();
        reversePathPoints = new LinkedList<Point>();

        /*if (startPoint == goalPoint)
        {
            pathPoints.add(goalPoint);
            found = true;
            length = 0;
            return;
        }*/
        long realtimeStart = System.currentTimeMillis();
        if ((goalPoint.x == 0) && (goalPoint.y == 0)) //something went really wrong and it takes forever to calculate/fail
        {
            System.err.print("Goal is (0, 0), something went wrong, aborting path planning...");
            return false;
        }

        //implementing http://en.wikipedia.org/wiki/A*#Pseudocode
        List<Point> closedSet = new LinkedList<Point>();
        List<Point> openSet = new LinkedList<Point>();

        openSet.add(startPoint);

        HashMap<Point, Point> came_from = new HashMap<Point, Point>();
        HashMap<Point, Double> g_score = new HashMap<Point, Double>();
        HashMap<Point, Double> f_score = new HashMap<Point, Double>();
        Point current;
        double tentative_g_score;

        g_score.put(startPoint, 0.0);
        f_score.put(startPoint, g_score.get(startPoint) + heuristicCostEstimate(startPoint, goalPoint));

        boolean limit_hit = false;

        while (!openSet.isEmpty()) {
            long time_elapsed = System.currentTimeMillis() - realtimeStart;
            if ((time_elapsed > SimConstants.MAX_PATH_SEARCH_TIME) && (limit)) {
                System.err.println("Took too long, time elapsed: " + time_elapsed + "ms.");
                limit_hit = true;
                break;
            }
            int current_index = getLowestScoreInList(openSet, f_score);
            current = openSet.get(current_index);
            if (current.distance(goalPoint) <= 2 * SimConstants.STEP_SIZE) {
                came_from.put(goalPoint, current);
                reconstructJumpPath(came_from, goalPoint);
                break;
            }

            openSet.remove(current_index);
            closedSet.add(current);

            for (Point neighbour : jump_neighbours(current, came_from.get(current))) {
                Point jumpPoint = jump(neighbour, current);

                if (jumpPoint != null) {
                    if (closedSet.contains(jumpPoint)) {
                        continue;
                    }

                    tentative_g_score = g_score.get(current) + current.distance(jumpPoint);

                    if (!openSet.contains(jumpPoint) || !g_score.containsKey(jumpPoint) || (tentative_g_score < g_score.get(jumpPoint))) {
                        if (!openSet.contains(jumpPoint)) {
                            openSet.add(jumpPoint);
                        }
                        came_from.put(jumpPoint, current);
                        g_score.put(jumpPoint, tentative_g_score);
                        f_score.put(jumpPoint, g_score.get(jumpPoint) + heuristicCostEstimate(jumpPoint, goalPoint));
                    }
                }
            }
        }
        recalcLength();
        if (this.length == 0) {
            return false;
        }

        return !limit_hit;
    }

    private void reconstructJumpPath(HashMap<Point, Point> came_from, Point current_node) {
        while (came_from.containsKey(current_node) && (came_from.get(current_node) != current_node)) {
            if (!pathPoints.isEmpty()) {
                LinkedList<Point> pts = pointsAlongSegment(pathPoints.get(pathPoints.size() - 1), current_node);

                if (pts.get(0).distance(current_node) < pts.get(pts.size() - 1).distance(current_node)) //points are in reverse
                {
                    for (int i = pts.size() - 2; i >= 0; i--) {
                        //if ((pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) > 0)
                        //        && (pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) <= 2))
                        pathPoints.add(pts.get(i));
                    }
                } else {
                    for (int i = 1; i < pts.size(); i++) {
                        //if ((pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) > 0)
                        //        && (pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) <= 2))
                        pathPoints.add(pts.get(i));
                    }
                }
            } else {
                pathPoints.add(current_node);
            }
            current_node = came_from.get(current_node);
        }
        if (!pathPoints.isEmpty() && pathPoints.size() != 1) {
            LinkedList<Point> pts = pointsAlongSegment(pathPoints.get(pathPoints.size() - 1), current_node);

            if (pts.get(0).distance(current_node) < pts.get(pts.size() - 1).distance(current_node)) //points are in reverse
            {
                for (int i = pts.size() - 2; i >= 0; i--) {
                    //if ((pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) > 0)
                    //        && (pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) <= 2))
                    pathPoints.add(pts.get(i));
                }
            } else {
                for (int i = 1; i < pts.size(); i++) {
                    //if ((pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) > 0)
                    //        && (pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) <= 2))
                    pathPoints.add(pts.get(i));
                }
            }
        } else {
            pathPoints.add(current_node);
        }
        for (int i = pathPoints.size() - 1; i >= 0; i--) {
            reversePathPoints.add(pathPoints.get(i));
        }
        reverse();
        recalcLength();
    }

    /**
     * Search recursively in the direction (parent -> child), stopping only when a jump point is
     * found.
     *
     * @return {Array.<[number, number]>} The x, y coordinate of the jump point found, or null if
     * not found
     */
    private Point jump(Point neighbour, Point current) {
        int x = neighbour.x;
        int y = neighbour.y;
        int px = current.x;
        int py = current.y;
        int dx = x - px;
        int dy = y - py;
        Point jx, jy;

        if (!(grid.locationExists(x, y) && grid.freeSpaceAt(x, y))) {
            return null;
        } else if (neighbour.distance(goalPoint) <= 1) {
            return new Point(x, y);
        }

        // check for forced neighbors
        // along the diagonal
        if (dx != 0 && dy != 0) {
            /*if (((grid.locationExists(x - dx, y + dy) && !grid.obstacleAt(x - dx, y + dy)) &&
                    !(grid.locationExists(x - dx, y) && !grid.obstacleAt(x - dx, y))) ||
                ((grid.locationExists(x + dx, y - dy) && !grid.obstacleAt(x + dx, y - dy)) &&
                    !(grid.locationExists(x, y - dy) && !grid.obstacleAt(x, y - dy)))) {
                return new Point(x, y);
            }*/ //This should never happen anyway as we are not cutting corners
            //Instead, check if the move is allowed, i.e. we are not cutting corners
            if (!(grid.locationExists(x - dx, y) && grid.freeSpaceAt(x - dx, y))
                    || !(grid.locationExists(x, y - dy) && grid.freeSpaceAt(x, y - dy))) {
                return null;
            }
        } // horizontally/vertically
        else if (dx != 0) { // moving along x
            /*if(((grid.locationExists(x + dx, y + 1) && !grid.obstacleAt(x + dx, y + 1)) &&
                        !(grid.locationExists(x, y + 1) && !grid.obstacleAt(x, y + 1))) ||
                ((grid.locationExists(x + dx, y - 1) && !grid.obstacleAt(x + dx, y - 1)) &&
                        !(grid.locationExists(x, y - 1) && !grid.obstacleAt(x, y - 1)))) {
                    return new Point(x, y);
                }*/
            if ((((grid.locationExists(x + dx, y + 1) && grid.freeSpaceAt(x + dx, y + 1))
                    || (grid.locationExists(x, y + 1) && grid.freeSpaceAt(x, y + 1)))
                    && !(grid.locationExists(x - dx, y + 1) && grid.freeSpaceAt(x - dx, y + 1)))
                    || (((grid.locationExists(x + dx, y - 1) && grid.freeSpaceAt(x + dx, y - 1))
                    || (grid.locationExists(x, y - 1) && grid.freeSpaceAt(x, y - 1)))
                    && !(grid.locationExists(x - dx, y - 1) && grid.freeSpaceAt(x - dx, y - 1)))) {
                return new Point(x, y);
            }
        } else if ((((grid.locationExists(x + 1, y + dy) && grid.freeSpaceAt(x + 1, y + dy))
                || (grid.locationExists(x + 1, y) && grid.freeSpaceAt(x + 1, y)))
                && !(grid.locationExists(x + 1, y - dy) && grid.freeSpaceAt(x + 1, y - dy)))
                || (((grid.locationExists(x - 1, y + dy) && grid.freeSpaceAt(x - 1, y + dy))
                || (grid.locationExists(x - 1, y) && grid.freeSpaceAt(x - 1, y)))
                && !(grid.locationExists(x - 1, y - dy) && grid.freeSpaceAt(x - 1, y - dy)))) {
            return new Point(x, y);
        }

        // when moving diagonally, must check for vertical/horizontal jump points
        if (dx != 0 && dy != 0) {
            jx = jump(new Point(x + dx, y), new Point(x, y));
            jy = jump(new Point(x, y + dy), new Point(x, y));
            if ((jx != null) || (jy != null)) {
                return new Point(x, y);
            }
        }

        // moving diagonally, must make sure both of the vertical/horizontal
        // neighbors is open to allow the path
        if ((grid.locationExists(x + dx, y) && grid.freeSpaceAt(x + dx, y))
                && (grid.locationExists(x, y + dy) && grid.freeSpaceAt(x, y + dy))) {
            return jump(new Point(x + dx, y + dy), new Point(x, y));
        } else {
            return null;
        }
    }

    private LinkedList<Point> jump_neighbours(Point pt, Point parent) {
        if (parent == null) {
            return neighbours(pt, 1);
        }

        LinkedList<Point> validNeighbours = new LinkedList<Point>();

        int nodeX = pt.x;
        int nodeY = pt.y;
        int parentX = parent.x;
        int parentY = parent.y;

        int dx = (nodeX - parentX) / Math.max(Math.abs(nodeX - parentX), 1);
        int dy = (nodeY - parentY) / Math.max(Math.abs(nodeY - parentY), 1);

        // search diagonally
        if (dx != 0 && dy != 0) {
            if (grid.locationExists(nodeX, nodeY + dy) && grid.freeSpaceAt(nodeX, nodeY + dy)) {
                validNeighbours.add(new Point(nodeX, nodeY + dy));
            }
            if (grid.locationExists(nodeX + dx, nodeY) && grid.freeSpaceAt(nodeX + dx, nodeY)) {
                validNeighbours.add(new Point(nodeX + dx, nodeY));
            }
            if ((grid.locationExists(nodeX, nodeY + dy) && grid.freeSpaceAt(nodeX, nodeY + dy))
                    && (grid.locationExists(nodeX + dx, nodeY) && grid.freeSpaceAt(nodeX + dx, nodeY))) {
                validNeighbours.add(new Point(nodeX + dx, nodeY + dy));
            }
            /*if (!(grid.locationExists(nodeX - dx, nodeY) && !grid.obstacleAt(nodeX - dx, nodeY)) &&
                    (grid.locationExists(nodeX, nodeY + dy) && !grid.obstacleAt(nodeX, nodeY + dy))) {
                validNeighbours.add(new Point(nodeX - dx, nodeY + dy));
            }
            if (!(grid.locationExists(nodeX, nodeY - dy) && !grid.obstacleAt(nodeX, nodeY - dy)) &&
                   (grid.locationExists(nodeX + dx, nodeY) && !grid.obstacleAt(nodeX + dx, nodeY))) {
                validNeighbours.add(new Point(nodeX + dx, nodeY - dy));
            }*/
        } // search horizontally/vertically
        else if (dx == 0) {
            if (grid.locationExists(nodeX, nodeY + dy) && grid.freeSpaceAt(nodeX, nodeY + dy)) {
                //if (grid.locationExists(nodeX, nodeY + dy) && !grid.obstacleAt(nodeX, nodeY + dy)) {
                validNeighbours.add(new Point(nodeX, nodeY + dy));
            }
            if (!(grid.locationExists(nodeX + 1, nodeY - dy) && grid.freeSpaceAt(nodeX + 1, nodeY - dy))/* &&
                        (grid.locationExists(nodeX + 1, nodeY) && !grid.obstacleAt(nodeX + 1, nodeY))*/) {
                validNeighbours.add(new Point(nodeX + 1, nodeY + dy));
                validNeighbours.add(new Point(nodeX + 1, nodeY));
            }
            if (!(grid.locationExists(nodeX - 1, nodeY - dy) && grid.freeSpaceAt(nodeX - 1, nodeY - dy))/* &&
                        (grid.locationExists(nodeX - 1, nodeY) && !grid.obstacleAt(nodeX - 1, nodeY))*/) {
                validNeighbours.add(new Point(nodeX - 1, nodeY + dy));
                validNeighbours.add(new Point(nodeX - 1, nodeY));
            }

        } else {
            if (grid.locationExists(nodeX + dx, nodeY) && grid.freeSpaceAt(nodeX + dx, nodeY)) {
                //if (grid.locationExists(nodeX + dx, nodeY) && !grid.obstacleAt(nodeX + dx, nodeY)) {
                validNeighbours.add(new Point(nodeX + dx, nodeY));
            }
            if (!(grid.locationExists(nodeX - dx, nodeY + 1) && grid.freeSpaceAt(nodeX - dx, nodeY + 1))/* &&
                        (grid.locationExists(nodeX, nodeY + 1) && !grid.obstacleAt(nodeX, nodeY + 1))*/) {
                validNeighbours.add(new Point(nodeX + dx, nodeY + 1));
                validNeighbours.add(new Point(nodeX, nodeY + 1));
            }
            if (!(grid.locationExists(nodeX - dx, nodeY - 1) && grid.freeSpaceAt(nodeX - dx, nodeY - 1))/* &&
                        (grid.locationExists(nodeX, nodeY - 1) && !grid.obstacleAt(nodeX, nodeY - 1))*/) {
                validNeighbours.add(new Point(nodeX + dx, nodeY - 1));
                validNeighbours.add(new Point(nodeX, nodeY - 1));
            }
        }

        return validNeighbours;
    }

    public Point getStartPoint() {
        return startPoint;
    }

    public Point getGoalPoint() {
        return goalPoint;
    }

    private double heuristicCostEstimate(Point start, Point goal) {
        return start.distance(goal);
    }

    private double heuristicCostEstimate(TopologicalNode startNode, TopologicalNode goalNode) {
        return startNode.getPosition().distance(goalNode.getPosition());
    }

    private int getLowestScoreInList(List<?> set, HashMap<?, Double> score) {
        int bestElement = 0;
        double bestValue = score.get(set.get(0));
        double newValue;
        for (int i = 1; i < set.size(); i++) {
            newValue = score.get(set.get(i));
            if (newValue < bestValue) {
                bestElement = i;
                bestValue = newValue;
            }
        }
        return bestElement;
    }

    public List<Point> getPoints() {
        return pathPoints;
    }

    public List<Point> getReversePoints() {
        return reversePathPoints;
    }

    public double getLength() {
        if (length == 0) {
            recalcLength();
        }
        return length;
    }

    public LinkedList<Point> getAllPathPixels() {
        if (allPathPixels == null) {
            allPathPixels = new LinkedList<>();
            Iterator<Point> i = pathPoints.iterator();
            Point curr, prev = startPoint;
            while (i.hasNext()) {
                curr = i.next();
                allPathPixels = mergeLists(allPathPixels, pointsAlongSegment(prev, curr));
                prev = curr;
            }
        }
        return allPathPixels;
    }

    public LinkedList<Point> getTakenPathPixels() {
        int c = 0;
        LinkedList<Point> pathPixels = new LinkedList<>();
        Iterator<Point> i = pathPoints.iterator();
        Point curr, prev = startPoint;
        while (i.hasNext() && c <= this.currentPoint) {
            curr = i.next();
            pathPixels = mergeLists(pathPixels, pointsAlongSegment(prev, curr));
            prev = curr;
            c++;
        }

        return pathPixels;
    }

    public LinkedList<Point> getCommingPathPixels() {
        int c = 0;
        LinkedList<Point> pathPixels = new LinkedList<>();
        Iterator<Point> i = pathPoints.iterator();
        Point curr, prev = startPoint;
        while (i.hasNext()) {
            curr = i.next();
            if (c > this.currentPoint) {
                pathPixels = mergeLists(pathPixels, pointsAlongSegment(prev, curr));
            }
            prev = curr;
            c++;
        }

        return pathPixels;
    }

    public void reverse() {
        List<Point> t = pathPoints;
        pathPoints = reversePathPoints;
        reversePathPoints = t;
    }

    public Path getReversePath() {
        //Path p = new Path(grid, goalPoint, startPoint, limit, jump, false);
        Path p = new Path();
        p.pathPoints = reversePathPoints;
        p.reversePathPoints = pathPoints;
        p.found = found;
        p.length = length;
        p.valid = valid;
        p.goalPoint = startPoint;
        p.startPoint = goalPoint;
        p.tMap = tMap;
        p.currentPoint = 0;
        p.exact = exact;
        p.grid = grid;
        if (pathSections != null) {
            p.pathSections = new LinkedList<>();
            for (int i = pathSections.size() - 1; i >= 0; i--) {
                p.pathSections.add(pathSections.get(i));
            }
        }

        return p;
    }

    private void recalcLength() {
        if (pathPoints != null && !pathPoints.isEmpty()) {
            Iterator<Point> i = pathPoints.iterator();
            Point curr, last = pathPoints.get(0);
            length = 0;
            while (i.hasNext()) {
                curr = i.next();
                length += last.distance(curr);
                //allPathPixels = mergeLists(allPathPixels, pointsAlongSegment(prev.x, prev.y, curr.x, curr.y));
                last = curr;
            }
        }
    }

    private LinkedList<Point> neighbours(Point pt, int stepSize) {
        LinkedList<Point> validNeighbours = new LinkedList<Point>();
        int neighbourX, neighbourY;

        for (neighbourX = pt.x - stepSize; neighbourX <= pt.x + stepSize; neighbourX += stepSize) {
            for (neighbourY = pt.y - stepSize; neighbourY <= pt.y + stepSize; neighbourY += stepSize) {

                // Check 0: don't add same node
                if (neighbourX == pt.x && neighbourY == pt.y) {
                    continue;
                }

                // Check 1: does location exist
                if (!grid.locationExists(neighbourX, neighbourY)) {
                    continue;
                }

                // Check 2: is it free space (or at least not an obstacle, choose which line to comment)
                if (!grid.freeSpaceAt(neighbourX, neighbourY)) {
                    //if (grid.obstacleAt(neighbourX, neighbourY)) {
                    continue;
                }

                // Check 3: is location reachable
                if (!grid.directLinePossible(pt, new Point(neighbourX, neighbourY), true, false)) {
                    continue;
                }

                //No cutting corners - this check only works if STEP_SIZE == 1
                if (stepSize == 1) {
                    int dx = neighbourX - pt.x;
                    int dy = neighbourY - pt.y;
                    boolean diagonal = (dx != 0) && (dy != 0);
                    //  --only add diagonal cells if there is space on both sides. Otherwise path has to go 'manhattan' way
                    if (diagonal && !(grid.freeSpaceAt(pt.x + dx, pt.y) && grid.freeSpaceAt(pt.x, pt.y + dy))) {
                        continue;
                    }
                }

                // Check 4: is it not too close to wall (unless it's a goalPoint)
                if (exact && grid.obstacleWithinDistance(neighbourX, neighbourY, 2)
                        && !(goalPoint.distance(neighbourX, neighbourY) <= 2)
                        && !(startPoint.distance(neighbourX, neighbourY) <= 2)) {
                    continue;
                }
                // Check 5: avoid running into teammates
                /*teammateCollision = false;
                for(TeammateAgent t: agent.getAllTeammates().values())
                    if(t.isInDirectRange() &&
                       t.distanceTo(new Point(neighbourX, neighbourY)) < 2*SimConstants.WALL_DISTANCE &&
                       !(goalPoint.distance(neighbourX, neighbourY) <= SimConstants.WALL_DISTANCE )) {
                        teammateCollision = true;
                        break;
                    }
                if(teammateCollision)
                    continue;*/
                // If we get here, all checks passed, add neighbour
                validNeighbours.add(new Point(neighbourX, neighbourY));
            }
        }

        /*if(grid.locationExists(neighbourX, neighbourY) &&
                   grid.directLinePossible(pt.x, pt.y, neighbourX, neighbourY) &&
                   (!grid.obstacleWithinDistance(neighbourX, neighbourY, SimConstants.WALL_DISTANCE) ||
                    goalPoint.distance(neighbourX, neighbourY) <= SimConstants.WALL_DISTANCE ) &&
                    grid.freeSpaceAt(neighbourX, neighbourY))
                        validNeighbours.add(new Point(neighbourX, neighbourY));*/
        return validNeighbours;
    }

    /**
     * Adds all points in list2 to list1 (no duplicates), returns merged list.
     */
    private LinkedList<Point> mergeLists(LinkedList<Point> list1, LinkedList<Point> list2) {
        list2.stream().filter((p) -> (!list1.contains(p))).forEach((p) -> {
            list1.add(p);
        });

        return list1;
    }

    /**
     * .
     * This can and should be improved, no need to check entire box, function also defined elsewhere
     */
    private LinkedList<Point> pointsAlongSegment(Point a, Point b) {
        LinkedList<Point> pts = new LinkedList<Point>();

        for (int i = Math.min(a.x, b.x); i <= Math.max(a.x, b.x); i++) {
            for (int j = Math.min(a.y, b.y); j <= Math.max(a.y, b.y); j++) {
                if (grid.distPointToLine(a, b, new Point(i, j)) < 0.5) {
                    pts.add(new Point(i, j));
                }
            }
        }

        return pts;
    }

    @Override
    public String toString() {
        if (this.startPoint != null && this.goalPoint != null) {
            return ("[Path Planner] (" + this.startPoint.x + "," + this.startPoint.y + ") -> (" + this.goalPoint.x + "," + this.goalPoint.y + ")");
        } else {
            return ("[Path Planner] null-Path");
        }
    }

    private void outputPathError() {
        if (SimConstants.OUTPUT_PATH_ERROR) {
            try {
                ExplorationImage img = new ExplorationImage(new Environment(grid.height, grid.width));
                ShowSettingsAgent agentSettings = new ShowSettingsAgent();
                agentSettings.showFreeSpace = true;
                if (tMap == null) {
                    agentSettings.baseStation = false;
                    img.fullUpdatePath(grid, startPoint, goalPoint, agentSettings);
                } else {
                    agentSettings.showTopologicalMap = true;
                    img.fullUpdatePath(grid, tMap, startPoint, goalPoint, agentSettings);
                }
                new File(SimConstants.DEFAULT_PATH_LOG_DIRECTORY).mkdirs();
                img.saveScreenshot(SimConstants.DEFAULT_PATH_LOG_DIRECTORY, "");
                System.out.println("Outputting path debug screens to: " + SimConstants.DEFAULT_PATH_LOG_DIRECTORY);

            } catch (Exception e) {
                System.err.println("Couldn't save path error screenshot, reason: " + e.getMessage());
            }
        }
    }

    /**
     * New method to use paths: create path and walk through it by itetator nextPoint.
     *
     * @return next Point to go to
     */
    public Point nextPoint() {
        if (pathPoints.isEmpty()) {
            if (!jump) {
                calculateAStarPath(false);
            } else {
                calculateJumpPath();
                if (pathPoints.isEmpty()) {
                    calculateAStarPath(false);
                }
            }
        }
        //currentPoint starts with 0 (startPoint) so we want to increment first!
        // pathPoints.size() - 1 is the goalPoint
        if (currentPoint >= pathPoints.size() - 1) {
            currentPoint = pathPoints.size() - 1;
        } else {
            currentPoint++;
        }
        if (currentPoint < 0) {
            throw new RuntimeException("FATAL ERROR: Path is size 0");
        }
        return pathPoints.get(currentPoint);
    }

    public boolean isFinished() {
        return currentPoint >= pathPoints.size() - 1;
    }

    public void resetStep() {
        currentPoint--;
    }

    public void reset() {
        this.found = false;
        this.currentPoint = 0;
        this.length = 0;
        this.pathNodes.clear();
        this.pathNodesReverse.clear();
        this.pathPoints.clear();
        this.reversePathPoints.clear();
        this.allPathPixels.clear();
    }

    public void start() {
        this.currentPoint = 0;
    }

    public boolean isValid() {
        return this.found && this.valid && !this.pathPoints.isEmpty() && !this.isFinished();
    }

    public void setInvalid() {
        if (this.pathSections != null) {
            pathSections.stream().forEach(p -> {
                p.setInvalid();
            });
        }
        this.found = false;
        this.valid = false;
    }

    public boolean testPath(boolean complete) {
        if (complete && (pathPoints == null || pathPoints.isEmpty() || !pathPoints.get(0).equals(this.startPoint) || !pathPoints.get(pathPoints.size() - 1).equals(this.goalPoint))) {
            this.found = false;
            this.setInvalid();
            return false;
        }
        for (int i = 1; i < pathPoints.size(); i++) {
            if (pathPoints.get(i).distance(pathPoints.get(i - 1)) > 30) {
                return false;
            }
        }
        return true;
    }

    public boolean repairPath() {
        this.pathPoints = new LinkedList<Point>();
        this.reversePathPoints = null;
        this.length = 0;
        this.allPathPixels = null;
        this.exact = true;
        this.valid = true;
        if (this.tMap == null) {
            this.found = calculateAStarPath(exact);
        } else {
            calcuateTopoPath();
        }
        return this.found;
    }

    public boolean getExact() {
        return this.exact;
    }
}
