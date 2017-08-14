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

import config.Constants;
import environment.Environment;
import environment.OccupancyGrid;
import environment.TopologicalMap;
import gui.ShowSettings.ShowSettingsAgent;
import java.awt.Point;
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
    private Point start;
    private Point goal;
    private OccupancyGrid grid;
    private boolean jump_allowed;
    private boolean limit;

    List<Point> reversePathPoints;
    List<Point> pathPoints;
    LinkedList<TopologicalNode> pathNodesReverse;
    LinkedList<TopologicalNode> pathNodes;
    LinkedList<Point> allPathPixels;
    double length;
    private TopologicalMap tMap;
    private int currentPoint = 0;
    private boolean jump;

    public Path(OccupancyGrid agentGrid, Point startpoint, Point endpoint, boolean limit, boolean jump) {
        this.start = startpoint;
        this.goal = endpoint;
        this.limit = limit;
        this.grid = agentGrid;
        this.jump = jump;
        if (!jump) {
            calculateAStarPath();
        } else {
            calculateJumpPath();
        }
    }

    public Path(OccupancyGrid agentGrid, TopologicalMap tMap,
            Point startpoint, Point endpoint, boolean limit) {
        //System.out.println("PLANNING PATH FROM " + startpoint + " TO " + endpoint);
        int[][] areaGrid = tMap.getAreaGrid();
        HashMap<Integer, TopologicalNode> topologicalNodes = tMap.getTopologicalNodes();

        this.start = startpoint;
        this.goal = endpoint;
        this.tMap = tMap;
        this.grid = agentGrid;
        this.pathPoints = new LinkedList<Point>();
        TopologicalNode startNode = topologicalNodes.get(areaGrid[startpoint.x][startpoint.y]);
        TopologicalNode goalNode = topologicalNodes.get(areaGrid[endpoint.x][endpoint.y]);

        if ((startNode == null) || (goalNode == null)) //Something wrong with building topological map
        {
            if (startNode == null) {
                if (agentGrid.obstacleAt(startpoint.x, startpoint.y)) {
                    // there cannot be an obstacle here, as we are planning a path from this point!
                    agentGrid.setNoObstacleAt(startpoint.x, startpoint.y);
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println("There was an obstacle at startpoint! Aborting.");
                    }
                    return;
                }
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println("startNode is null: " + startpoint + ", aborting");
                }
            }
            if (goalNode == null) {
                if (agentGrid.obstacleAt(endpoint.x, endpoint.y)) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println("There was an obstacle at goalpoint! Aborting.");
                    }
                    return;
                }
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println("goalNode is null: " + endpoint + ", aborting");
                }
            }
            return;
        }

        // not sure why this can happen, but consider those nodes unexplored as workaround
        if (!startNode.equals(goalNode) && (startNode.getListOfNeighbours().isEmpty())) {
            for (int[] areaGrid1 : areaGrid) {
                for (int j = 0; j < areaGrid[0].length; j++) {
                    if (areaGrid1[j] == startNode.getID()) {
                        areaGrid1[j] = Constants.UNEXPLORED_NODE_ID;
                    }
                }
            }
            startNode.setID(Constants.UNEXPLORED_NODE_ID);
        }

        if (!startNode.equals(goalNode) && (goalNode.getListOfNeighbours().isEmpty())) {
            for (int[] areaGrid1 : areaGrid) {
                for (int j = 0; j < areaGrid[0].length; j++) {
                    if (areaGrid1[j] == goalNode.getID()) {
                        areaGrid1[j] = Constants.UNEXPLORED_NODE_ID;
                    }
                }
            }
            goalNode.setID(Constants.UNEXPLORED_NODE_ID);
        }

        if (startNode.equals(goalNode)
                || (startNode.getListOfNeighbours().contains(goalNode) && (goalNode.getID() != Constants.UNEXPLORED_NODE_ID))
                || ((startNode.getID() == Constants.UNEXPLORED_NODE_ID) && (goalNode.getID() == Constants.UNEXPLORED_NODE_ID))) {
            boolean pathFound = false; //getJumpPath(agentGrid, startpoint, endpoint, limit);
            if (!pathFound) {
                pathFound = calculateAStarPath();
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println("Jump path did not work either... trying A* path");
                }
                if (!pathFound) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println("Cannot use topological map, startpoint is " + startpoint.toString()
                                + ", endpoint is " + endpoint.toString() + ", trying A*...");
                    }
                    outputPathError();
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println("A* and JumpPath did not work either... :(");
                    }
                    outputPathError();
                }
            }
            return;
        }

        Path p0 = null;
        Path p3 = null;

        if (startNode.getID() == Constants.UNEXPLORED_NODE_ID) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Start point " + startpoint + " in unexplored space!");
            }
            findNearestExploredNode(areaGrid, topologicalNodes);
            List<Point> pathToExploredPoints = pathPoints;
            if (pathPoints.isEmpty()) {
                return;
            }
            Point lastPoint = pathPoints.get(pathPoints.size() - 1);
            //System.out.println("New start point is " + lastPoint);
            startNode = topologicalNodes.get(areaGrid[lastPoint.x][lastPoint.y]);
            p0 = new Path(agentGrid, startpoint, lastPoint, false, false);
            if (!p0.found) {
                found = false;
                outputPathError();
                return;
            }
            startpoint = lastPoint;
        }
        if (goalNode.getID() == Constants.UNEXPLORED_NODE_ID) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Goal point " + endpoint + " in unexplored space!");
            }
            findNearestExploredNode(areaGrid, topologicalNodes);
            List<Point> pathToExploredPoints = pathPoints;
            if (pathPoints.isEmpty()) {
                return;
            }
            Point lastPoint = pathPoints.get(pathPoints.size() - 1);
            //System.out.println("New goal point is " + lastPoint);
            goalNode = topologicalNodes.get(areaGrid[lastPoint.x][lastPoint.y]);
            p3 = new Path(agentGrid, lastPoint, endpoint, false, false);
            if (!p3.found) {
                found = false;
                p3.outputPathError();
                return;
            }
            endpoint = lastPoint;
        }

        if ((startNode == null) || (goalNode == null) || startNode.equals(goalNode) || startNode.getListOfNeighbours().contains(goalNode)
                || ((startNode.getID() == Constants.UNEXPLORED_NODE_ID) && (goalNode.getID() == Constants.UNEXPLORED_NODE_ID))) {
            Path p1 = new Path(agentGrid, startpoint, endpoint, false, true);
            if (!p1.found) {
                found = false;
                outputPathError();
                return;
            }
            pathPoints = new LinkedList<Point>();
            if (p0 != null) {
                for (Point p : p0.getPoints()) {
                    pathPoints.add(p);
                }
            }
            p1.getPoints().stream().forEach((p) -> {
                pathPoints.add(p);
            });
            if (p3 != null) {
                for (Point p : p3.getPoints()) {
                    pathPoints.add(p);
                }
            }
            return;
        }

        calculateAStarPath(startNode, goalNode);
        if (pathNodes.isEmpty()) //No path found
        {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Could not find topological path, startpoint is " + startpoint.toString()
                        + ", endpoint is " + endpoint.toString() + ", trying A*...");
            }
            outputPathError();
            boolean pathFound = calculateAStarPath();
            if (!pathFound) {
                pathFound = calculateJumpPath();
                if (!pathFound) {
                    if (Constants.DEBUG_OUTPUT) {
                        System.out.println("A* and JumpPath did not work either... :(");
                    }
                    outputPathError();
                }
            } else if (Constants.DEBUG_OUTPUT) {
                System.out.println("A* worked! Successfully recovered full path");
            }
            return;
        }
        Path p1 = new Path(agentGrid, startpoint, (Point) pathNodes.get(1).getPosition().clone(), false, true);
        Path p2 = new Path(agentGrid, (Point) pathNodes.get(pathNodes.size() - 2).getPosition().clone(), endpoint, false, true);

        if (!p1.found) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Could not find p1! From " + startpoint + " to " + (Point) pathNodes.get(1).getPosition());
            }
            p1.outputPathError();
            found = false;
            return;
        }

        if (!p2.found) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Could not find p2! From " + (Point) pathNodes.get(pathNodes.size() - 2).getPosition()
                        + " to " + endpoint);
            }
            p2.outputPathError();
            found = false;
            return;
        }

        pathPoints = new LinkedList<Point>();
        int index = 0;
        if (p0 != null) {
            for (Point p : p0.getPoints()) {
                pathPoints.add(p);
            }
        }
        for (TopologicalNode n : pathNodes) {
            //node only
            //pathPoints.add(n.getPosition());

            if (n.equals(startNode)) {
                p1.getPoints().stream().forEach((p) -> {
                    pathPoints.add(p);
                });
            }

            if (!n.equals(startNode) && !n.equals(goalNode) && !pathNodes.get(index + 1).equals(goalNode)) {
                for (Point p : n.getPathToNeighbour(pathNodes.get(index + 1)).getPoints()) {
                    pathPoints.add((Point) p.clone());
                }
            }

            if (n.equals(goalNode)) {
                p2.getPoints().stream().forEach((p) -> {
                    pathPoints.add(p);
                });
            }

            index++;
        }
        if (p3 != null) {
            for (Point p : p3.getPoints()) {
                pathPoints.add(p);
            }
        }
        recalcLength();

        //System.out.println("Path length is " + this.getLength());
    }

    public void calculateAStarPath(TopologicalNode startNode, TopologicalNode goalNode) {
        found = false;
        pathNodesReverse = new LinkedList<TopologicalNode>();
        pathNodes = new LinkedList<TopologicalNode>();

        long realtimeStart = System.currentTimeMillis();
        //System.out.print(Constants.INDENT + "Planning path from " + startNode.getID() + " to " + goalNode.getID() + ". ");

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
            long time_elapsed = System.currentTimeMillis() - realtimeStart;

            int current_index = getLowestScoreInList(openSet, f_score);
            current = openSet.get(current_index);
            if (current.equals(goalNode)) {
                //came_from.put(goalNode, current);
                //System.out.println("Found topological node path, reconstructing...");
                reconstructPath(came_from, goalNode);
                break;
            }

            openSet.remove(current_index);
            closedSet.add(current);

            for (TopologicalNode neighbour : current.getListOfNeighbours()) {
                if (neighbour.getID() != Constants.UNEXPLORED_NODE_ID) {
                    if (closedSet.contains(neighbour)) {
                        continue;
                    }
                    tentative_g_score = g_score.get(current) + current.getPathToNeighbour(neighbour).getLength();

                    if (!openSet.contains(neighbour) || !g_score.containsKey(neighbour) || (tentative_g_score < g_score.get(neighbour))) {
                        if (!openSet.contains(neighbour)) {
                            openSet.add(neighbour);
                        }
                        if (!current.equals(neighbour)) {
                            came_from.put(neighbour, current);
                        }
                        g_score.put(neighbour, tentative_g_score);
                        f_score.put(neighbour, g_score.get(neighbour) + heuristicCostEstimate(neighbour, goalNode));
                    }
                }
            }
        }

        //System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

    public boolean calculateAStarPath() {
        found = false;
        pathPoints = new LinkedList<Point>();
        reversePathPoints = new LinkedList<Point>();

        /*if (start == goal)
        {
            pathPoints.add(goal);
            found = true;
            length = 0;
            return;
        }*/
        long realtimeStart = System.currentTimeMillis();
        //System.out.print(Constants.INDENT + "Planning path from " + start.x + "," + start.y + " to " + goal.x + "," + goal.y + ". ");
        if ((goal.x == 0) && (goal.y == 0)) //something went really wrong and it takes forever to calculate/fail
        {
            //System.out.print("Goal is (0, 0), something went wrong, aborting path planning...");
            return false;
        }

        //implementing http://en.wikipedia.org/wiki/A*#Pseudocode
        List<Point> closedSet = new LinkedList<Point>();
        List<Point> openSet = new LinkedList<Point>();

        openSet.add(start);

        HashMap<Point, Point> came_from = new HashMap<Point, Point>();
        HashMap<Point, Double> g_score = new HashMap<Point, Double>();
        HashMap<Point, Double> f_score = new HashMap<Point, Double>();
        Point current;
        double tentative_g_score;

        g_score.put(start, 0.0);
        f_score.put(start, g_score.get(start) + heuristicCostEstimate(start, goal));

        boolean limit_hit = false;

        while (!openSet.isEmpty()) {
            long time_elapsed = System.currentTimeMillis() - realtimeStart;
            if ((time_elapsed > Constants.MAX_PATH_SEARCH_TIME) /*&& (limit)*/) {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println("Took too long (A*), startpoint is " + start.toString()
                            + ", endpoint is " + goal.toString() + "time elapsed: " + time_elapsed + "ms.");
                }
                outputPathError();
                limit_hit = true;
                break;
            }
            int current_index = getLowestScoreInList(openSet, f_score);
            current = openSet.get(current_index);
            if (current.distance(goal) < Constants.STEP_SIZE * 2) {
                came_from.put(goal, current);
                reconstructPath(came_from, goal);
                break;
            }

            openSet.remove(current_index);
            closedSet.add(current);

            for (Point neighbour : neighbours(current)) {
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
                    f_score.put(neighbour, g_score.get(neighbour) + heuristicCostEstimate(neighbour, goal));
                }
            }
        }
        recalcLength();

        return !limit_hit;
        //System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

    public boolean calculateJumpPath() {
        found = false;
        pathPoints = new LinkedList<Point>();
        reversePathPoints = new LinkedList<Point>();

        /*if (start == goal)
        {
            pathPoints.add(goal);
            found = true;
            length = 0;
            return;
        }*/
        long realtimeStart = System.currentTimeMillis();
        //System.out.print(Constants.INDENT + "Planning jump path from " + start.x + "," + start.y + " to " + goal.x + "," + goal.y + ". ");
        if ((goal.x == 0) && (goal.y == 0)) //something went really wrong and it takes forever to calculate/fail
        {
            System.out.print("Goal is (0, 0), something went wrong, aborting path planning...");
            return false;
        }

        //implementing http://en.wikipedia.org/wiki/A*#Pseudocode
        List<Point> closedSet = new LinkedList<Point>();
        List<Point> openSet = new LinkedList<Point>();

        openSet.add(start);

        HashMap<Point, Point> came_from = new HashMap<Point, Point>();
        HashMap<Point, Double> g_score = new HashMap<Point, Double>();
        HashMap<Point, Double> f_score = new HashMap<Point, Double>();
        Point current;
        double tentative_g_score;

        g_score.put(start, 0.0);
        f_score.put(start, g_score.get(start) + heuristicCostEstimate(start, goal));

        boolean limit_hit = false;

        while (!openSet.isEmpty()) {
            long time_elapsed = System.currentTimeMillis() - realtimeStart;
            if ((time_elapsed > Constants.MAX_PATH_SEARCH_TIME) && (limit)) {
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println("Took too long, time elapsed: " + time_elapsed + "ms.");
                }
                limit_hit = true;
                break;
            }
            int current_index = getLowestScoreInList(openSet, f_score);
            current = openSet.get(current_index);
            if (current.distance(goal) <= 2 * Constants.STEP_SIZE) {
                came_from.put(goal, current);
                reconstructJumpPath(came_from, goal);
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
                        f_score.put(jumpPoint, g_score.get(jumpPoint) + heuristicCostEstimate(jumpPoint, goal));
                    }
                }
            }
        }
        recalcLength();

        //System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        return !limit_hit;
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
        } else if (neighbour.distance(goal) <= 1) {
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

    private void findNearestExploredNode(int[][] areaGrid, HashMap<Integer, TopologicalNode> topologicalNodes) {
        found = false;
        pathPoints = new LinkedList<Point>();
        reversePathPoints = new LinkedList<Point>();

        /*if (start == goal)
        {
            pathPoints.add(goal);
            found = true;
            length = 0;
            return;
        }*/
        long realtimeStart = System.currentTimeMillis();
        //System.out.print(Constants.INDENT + "Planning path from " + start.x + "," + start.y + " to " + goal.x + "," + goal.y + ". ");
        if ((goal.x == 0) && (goal.y == 0)) //something went really wrong and it takes forever to calculate/fail
        {
            //System.out.print("Goal is (0, 0), something went wrong, aborting path planning...");
            return;
        }

        //implementing http://en.wikipedia.org/wiki/A*#Pseudocode
        List<Point> closedSet = new LinkedList<Point>();
        List<Point> openSet = new LinkedList<Point>();

        openSet.add(start);

        HashMap<Point, Point> came_from = new HashMap<Point, Point>();
        HashMap<Point, Double> g_score = new HashMap<Point, Double>();
        HashMap<Point, Double> f_score = new HashMap<Point, Double>();
        Point current;
        double tentative_g_score;

        g_score.put(start, 0.0);
        f_score.put(start, g_score.get(start) + heuristicCostEstimate(start, goal));

        while (!openSet.isEmpty()) {
            long time_elapsed = System.currentTimeMillis() - realtimeStart;
            if (time_elapsed > Constants.MAX_PATH_SEARCH_TIME) {
                //System.out.println("Took too long, time elapsed: " + time_elapsed + "ms.");
                break;
            }
            int current_index = getLowestScoreInList(openSet, f_score);
            current = openSet.get(current_index);
            if ((areaGrid[current.x][current.y] != Constants.UNEXPLORED_NODE_ID) && (topologicalNodes.get(areaGrid[current.x][current.y]) != null)
                    && (!topologicalNodes.get(areaGrid[current.x][current.y]).getListOfNeighbours().isEmpty())) {
                //came_from.put(goal, current);
                reconstructPath(came_from, current);
                break;
            }

            openSet.remove(current_index);
            closedSet.add(current);

            for (Point neighbour : neighbours(current)) {
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
                    f_score.put(neighbour, g_score.get(neighbour) + heuristicCostEstimate(neighbour, goal));
                }
            }
        }

        //System.out.println("Took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

    public Point getStartPoint() {
        return start;
    }

    public Point getGoalPoint() {
        return goal;
    }

    private double heuristicCostEstimate(Point start, Point goal) {
        return start.distance(goal);
    }

    private double heuristicCostEstimate(TopologicalNode startNode, TopologicalNode goalNode) {
        return startNode.getPosition().distance(goalNode.getPosition());
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
    }

    private void reconstructJumpPath(HashMap<Point, Point> came_from, Point current_node) {
        while (came_from.containsKey(current_node) && (came_from.get(current_node) != current_node)) {
            if (!pathPoints.isEmpty()) {
                LinkedList<Point> pts = pointsAlongSegment(pathPoints.get(pathPoints.size() - 1).x, pathPoints.get(pathPoints.size() - 1).y,
                        current_node.x, current_node.y);

                if (pts.get(0).distance(current_node) < pts.get(pts.size() - 1).distance(current_node)) //points are in reverse
                {
                    for (int i = pts.size() - 2; i >= 0; i--) {
                        //if ((pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) > 0)
                        //        && (pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) <= 2))
                        pathPoints.add(pts.get(i));
                        //else
                        //    System.out.println("Point discarded.");
                    }
                } else {
                    for (int i = 1; i < pts.size(); i++) {
                        //if ((pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) > 0)
                        //        && (pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) <= 2))
                        pathPoints.add(pts.get(i));
                        //else
                        //    System.out.println("Point discarded.");
                    }
                }
            } else {
                pathPoints.add(current_node);
            }
            current_node = came_from.get(current_node);
        }
        if (!pathPoints.isEmpty()) {
            LinkedList<Point> pts = pointsAlongSegment(pathPoints.get(pathPoints.size() - 1).x, pathPoints.get(pathPoints.size() - 1).y,
                    current_node.x, current_node.y);

            if (pts.get(0).distance(current_node) < pts.get(pts.size() - 1).distance(current_node)) //points are in reverse
            {
                for (int i = pts.size() - 2; i >= 0; i--) {
                    //if ((pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) > 0)
                    //        && (pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) <= 2))
                    pathPoints.add(pts.get(i));
                    //else
                    //    System.out.println("Point discarded.");
                }
            } else {
                for (int i = 1; i < pts.size(); i++) {
                    //if ((pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) > 0)
                    //        && (pathPoints.get(pathPoints.size() - 1).distance(pts.get(i)) <= 2))
                    pathPoints.add(pts.get(i));
                    //else
                    //    System.out.println("Point discarded.");
                }
            }
        } else {
            pathPoints.add(current_node);
        }
        for (int i = pathPoints.size() - 1; i >= 0; i--) {
            reversePathPoints.add(pathPoints.get(i));
        }
        reverse();
        found = true;
        recalcLength();
    }

    private void reconstructPath(HashMap<TopologicalNode, TopologicalNode> came_from, TopologicalNode current_node) {
        while (came_from.containsKey(current_node) && (came_from.get(current_node) != current_node)) {
            pathNodesReverse.add(current_node);
            current_node = came_from.get(current_node);
        }
        pathNodesReverse.add(current_node);
        for (int i = pathNodesReverse.size() - 1; i >= 0; i--) {
            pathNodes.add(pathNodesReverse.get(i));
        }
        found = true;
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
        allPathPixels = new LinkedList<>();
        Iterator<Point> i = pathPoints.iterator();
        Point curr, prev = start;
        while (i.hasNext()) {
            curr = i.next();
            allPathPixels = mergeLists(allPathPixels, pointsAlongSegment(prev.x, prev.y, curr.x, curr.y));
            prev = curr;
        }
        return allPathPixels;
    }

    public void reverse() {
        List<Point> t = pathPoints;
        pathPoints = reversePathPoints;
        reversePathPoints = t;
    }

    public Path getReversePath() {
        Path p = new Path(grid, goal, start, limit, jump_allowed);
        p.pathPoints = reversePathPoints;
        p.reversePathPoints = pathPoints;
        p.found = found;
        p.length = length;
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

    private LinkedList<Point> neighbours(Point pt) {
        return neighbours(pt, Constants.STEP_SIZE);
    }

    private LinkedList<Point> neighbours(Point pt, int stepSize) {
        LinkedList<Point> validNeighbours = new LinkedList<Point>();
        int neighbourX, neighbourY;
        boolean teammateCollision;

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
                //if(!grid.freeSpaceAt(neighbourX, neighbourY))
                if (grid.obstacleAt(neighbourX, neighbourY)) {
                    continue;
                }

                // Check 3: is location reachable
                if (!grid.directLinePossible(pt.x, pt.y, neighbourX, neighbourY, true, false)) {
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

                // Check 4: is it not too close to wall (unless it's a goal)
                /*if(grid.obstacleWithinDistance(neighbourX, neighbourY, Constants.WALL_DISTANCE) &&
                   !(goal.distance(neighbourX, neighbourY) <= Constants.WALL_DISTANCE ) &&
                   !(start.distance(neighbourX, neighbourY) <= Constants.WALL_DISTANCE))
                    continue;*/
                // Check 5: avoid running into teammates
                /*teammateCollision = false;
                for(TeammateAgent t: agent.getAllTeammates().values())
                    if(t.isInDirectRange() &&
                       t.distanceTo(new Point(neighbourX, neighbourY)) < 2*Constants.WALL_DISTANCE &&
                       !(goal.distance(neighbourX, neighbourY) <= Constants.WALL_DISTANCE )) {
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
                   (!grid.obstacleWithinDistance(neighbourX, neighbourY, Constants.WALL_DISTANCE) ||
                    goal.distance(neighbourX, neighbourY) <= Constants.WALL_DISTANCE ) &&
                    grid.freeSpaceAt(neighbourX, neighbourY))
                        validNeighbours.add(new Point(neighbourX, neighbourY));*/
        //System.out.println("Found " + validNeighbours.size() + "neighbours");
        return validNeighbours;
    }

    //Adds all points in list2 to list1 (no duplicates), returns merged list.
    private LinkedList<Point> mergeLists(LinkedList<Point> list1, LinkedList<Point> list2) {
        list2.stream().filter((p) -> (!list1.contains(p))).forEach((p) -> {
            list1.add(p);
        });

        return list1;
    }

    // This can and should be improved, no need to check entire box, function also defined elsewhere
    private LinkedList<Point> pointsAlongSegment(int x1, int y1, int x2, int y2) {
        LinkedList<Point> pts = new LinkedList<Point>();

        for (int i = Math.min(x1, x2); i <= Math.max(x1, x2); i++) {
            for (int j = Math.min(y1, y2); j <= Math.max(y1, y2); j++) {
                if (grid.distPointToLine(x1, y1, x2, y2, i, j) < 0.5) {
                    pts.add(new Point(i, j));
                }
            }
        }

        return pts;
    }

    @Override
    public String toString() {
        if (this.start != null && this.goal != null) {
            return ("[Path Planner] (" + this.start.x + "," + this.start.y + ") -> (" + this.goal.x + "," + this.goal.y + ")");
        } else {
            return ("[Path Planner] null-Path");
        }
    }

    private void outputPathError() {
        if (Constants.OUTPUT_PATH_ERROR) {
            try {
                ExplorationImage img = new ExplorationImage(new Environment(grid.height, grid.width));
                ShowSettingsAgent agentSettings = new ShowSettingsAgent();
                agentSettings.showFreeSpace = true;
                if (tMap == null) {
                    agentSettings.baseStation = false;
                    img.fullUpdatePath(grid, start, goal, agentSettings);
                } else {
                    agentSettings.showTopologicalMap = true;
                    img.fullUpdatePath(grid, tMap, start, goal, agentSettings);
                }
                img.saveScreenshot(Constants.DEFAULT_PATH_LOG_DIRECTORY);
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println("Outputting path debug screens to: " + Constants.DEFAULT_PATH_LOG_DIRECTORY);
                }
            } catch (Exception e) {
                System.err.println("Couldn't save path error screenshot, reason: " + e.getMessage());
            }
        }
    }

    /**
     * New method to use paths: create path and walk throug it by itetator nextPoint.
     *
     * @return next Point to go to
     */
    public Point nextPoint() {
        if (pathPoints.isEmpty()) {
            if (!jump) {
                calculateAStarPath();
            } else {
                calculateJumpPath();
            }
        }
        //currentPOint starts with 0 (startPoint) so we want to increment first!
        // pathPoints.size() - 1 is the goal
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
}
