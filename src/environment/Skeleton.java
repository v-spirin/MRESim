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
import communication.PropModel1;
import config.SimConstants;
import config.SimulatorConfig;
import java.awt.Color;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.LinkedList;
import javax.imageio.ImageIO;
import path.TopologicalNode;

/**
 * Skeleton of a monochrome map. SOME CODE IN THIS FILE TAKEN FROM Sudhanshu Kumar:
 * http://www.sourcecodesworld.com/source/show.asp?ScriptID=692
 *
 * @author julh
 */
public class Skeleton {

    /**
     * Counts the neighbors of a pixel which are not 0
     *
     * @param grid of monochrome pixels (int[][])
     * @param x Point.x
     * @param y Point.y
     * @return 0-8 nonzeros
     */
    public static int numNonzeroNeighbors(int[][] grid, int x, int y) {
        int num = 0;
        if (grid[x][y - 1] != 0) {
            num++;
        }
        if (grid[x + 1][y - 1] != 0) {
            num++;
        }
        if (grid[x + 1][y] != 0) {
            num++;
        }
        if (grid[x + 1][y + 1] != 0) {
            num++;
        }
        if (grid[x][y + 1] != 0) {
            num++;
        }
        if (grid[x - 1][y + 1] != 0) {
            num++;
        }
        if (grid[x - 1][y] != 0) {
            num++;
        }
        if (grid[x - 1][y - 1] != 0) {
            num++;
        }

        return num;
    }

    /**
     * Counts the changes of pixel-values of the neighbors from 0 to 1 (black to white)
     *
     * @param grid of monochrome pixels (int[][])
     * @param x Point.x
     * @param y Point.y
     * @return 0-8 changes
     */
    public static int neighborTraversal(int[][] grid, int x, int y) {
        int numChanges = 0;
        if (grid[x][y - 1] == 0 && grid[x + 1][y - 1] != 0) {
            numChanges++;
        }
        if (grid[x + 1][y - 1] == 0 && grid[x + 1][y] != 0) {
            numChanges++;
        }
        if (grid[x + 1][y] == 0 && grid[x + 1][y + 1] != 0) {
            numChanges++;
        }
        if (grid[x + 1][y + 1] == 0 && grid[x][y + 1] != 0) {
            numChanges++;
        }
        if (grid[x][y + 1] == 0 && grid[x - 1][y + 1] != 0) {
            numChanges++;
        }
        if (grid[x - 1][y + 1] == 0 && grid[x - 1][y] != 0) {
            numChanges++;
        }
        if (grid[x - 1][y] == 0 && grid[x - 1][y - 1] != 0) {
            numChanges++;
        }
        if (grid[x - 1][y - 1] == 0 && grid[x][y - 1] != 0) {
            numChanges++;
        }

        return numChanges;
    }

    /**
     * Finds the free space as skeleton
     *
     * @param grid
     * @return
     */
    private static int[][] findCSpace(int[][] grid) {
        return skeletonize(grid, 0);
    }

    /**
     * this method finds the points that are within a margin of the obstacles
     */
    private static int[][] skeletonizeNearBorders(int[][] grid) {
        int[][] cspace = findCSpace(grid);

        int width = cspace.length;
        int height = cspace[0].length;
        int skeleton[][] = new int[width][height];

        //lets find border points
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                //if we are in cspace and are bordering at least one non-cspace cell
                if ((cspace[i][j] == 1) && (numNonzeroNeighbors(cspace, i, j) < 8)) {
                    skeleton[i][j] = 1;
                } else {
                    skeleton[i][j] = 0;
                }
            }
        }

        return skeleton;
    }

    /**
     * SKeletonizes the given int[][] max times
     *
     * @param grid the given grid
     * @param max the 'factor' 0 gives the free space of the given grid, Integer.Max_Value should
     * give a 1px-thick skeleton, 70 will leave out some space in big areas
     * @return the skeletonized grid
     */
    private static int[][] skeletonize(int[][] grid, int max) {
        int width = grid.length;
        int height = grid[0].length;

        int u1[][] = new int[width][height];
        int u2[][] = new int[width][height];

        for (int i = 0; i < width; i++) {
            System.arraycopy(grid[i], 0, u1[i], 0, height);
            System.arraycopy(grid[i], 0, u2[i], 0, height);
        }

        boolean found = true;
        int counter = 0;

        while (found) {
            found = false;
            for (int i = 2; i < width - 2; i++) {
                for (int j = 2; j < height - 2; j++) {
                    if (u1[i][j] == 1) {
                        if ((numNonzeroNeighbors(u1, i, j) >= 2) && (numNonzeroNeighbors(u1, i, j) <= 6)) {
                            if (neighborTraversal(u1, i, j) == 1) {
                                if (u1[i][j - 1] * u1[i - 1][j] * u1[i + 1][j] == 0 || neighborTraversal(u1, i, j - 1) != 1) {
                                    if (u1[i][j - 1] * u1[i - 1][j] * u1[i][j + 1] == 0 || neighborTraversal(u1, i - 1, j) != 1) {
                                        u2[i][j] = 0;
                                        found = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (!found || counter == max) {
                return u2;
            }
            for (int i = 2; i < width - 2; i++) {
                System.arraycopy(u2[i], 2, u1[i], 2, height - 2 - 2);
            }

            counter++;
        }
        return null;
    }

    /**
     * gives a list of Points !0 of the given grid (in a skeleton this will be a list of
     * skeletonpoints)
     *
     * @param grid
     * @return
     */
    public static LinkedList<Point> gridToList(int[][] grid) {
        LinkedList<Point> sk = new LinkedList<Point>();

        for (int j = 0; j < grid[0].length; j++) {
            for (int i = 0; i < grid.length; i++) {
                if (grid[i][j] == 1) {
                    sk.add(new Point(i, j));
                }
            }
        }
        return sk;
    }

    /**
     * Debugging method
     *
     * @param mat
     */
    private static void printMatrix(int[][] mat) {
        for (int j = 0; j < mat[0].length; j++) {
            for (int[] mat1 : mat) {
                System.out.print(mat1[j] + " ");
            }
            System.out.println();
        }
        System.out.println();
        System.out.println();
    }

    /**
     * CHecks if the box 2*SimConstants.WIDE_OPEN_SPACE_VALUE (currently 2*50) around the given Point
     *
     * @param grid
     * @param x Point.x
     * @param y Point.y
     * @return true if the point is in open space
     */
    private static boolean inWideOpenSpace(int[][] grid, int x, int y) {
        for (int i = Math.max(0, x - SimConstants.WIDE_OPEN_SPACE_VALUE); i <= Math.min(grid.length - 1, x + SimConstants.WIDE_OPEN_SPACE_VALUE); i++) {
            for (int j = Math.max(0, y - SimConstants.WIDE_OPEN_SPACE_VALUE); j <= Math.min(grid[0].length - 1, y + SimConstants.WIDE_OPEN_SPACE_VALUE); j++) {
                if (grid[i][j] == 0) {
                    return false;
                }
            }
        }

        return true;
    }

    /**
     * Find Skeleton, wide open spaces will be an obstacle (currently 100x100)
     *
     * @param grid
     * @return
     */
    public static int[][] findSkeleton(OccupancyGrid grid) {
        return findSkeleton(grid, true, false);
    }

    /**
     * Find Skeleton, wide open spaces will be an obstacle (currently 100x100)
     *
     * @param grid
     * @return
     */
    public static int[][] findSkeletonNearBorders(OccupancyGrid grid) {
        return findSkeleton(grid, false, true);
    }

    /**
     * Generates the skeleton of the given grid
     *
     * @param grid
     * @param treatWideOpenSpaceAsObstacle
     * @param skeletonNearBorders
     * @return
     */
    private static int[][] findSkeleton(OccupancyGrid grid, boolean treatWideOpenSpaceAsObstacle,
            boolean skeletonNearBorders) {
        int[][] freeSpaceGrid = new int[grid.width][grid.height];

        for (int i = 0; i < grid.width; i++) {
            for (int j = 0; j < grid.height; j++) {
                if (grid.freeSpaceAt(i, j)) {//&& (!skeletonNearBorders || (!obstacleWithinDistance(grid, i, j, 5)))) {
                    freeSpaceGrid[i][j] = 1;
                } else {
                    freeSpaceGrid[i][j] = 0;
                }
            }
        }

        if (treatWideOpenSpaceAsObstacle) {
            for (int i = 0; i < grid.width; i++) {
                for (int j = 0; j < grid.height; j++) {
                    if (freeSpaceGrid[i][j] == 1 && inWideOpenSpace(freeSpaceGrid, i, j)) {
                        //Only one pixel is not threated as an obstacle
                        freeSpaceGrid[i][j] = 0;
                        freeSpaceGrid[i - 1][j] = 0;
                        freeSpaceGrid[i + 1][j] = 0;
                        freeSpaceGrid[i][j + 1] = 0;
                        freeSpaceGrid[i][j - 1] = 0;
                    }
                }
            }
        }
        //skeleton = distanceTransform(skeleton);
        int[][] skeleton;
        if (!skeletonNearBorders) {
            skeleton = skeletonize(freeSpaceGrid, Integer.MAX_VALUE);
        } else {
            skeleton = skeletonizeNearBorders(freeSpaceGrid);
        }
        return skeleton;
    }

    /**
     *
     * @param status given grid as Environment
     * @param treatWideOpenSpaceAsObstacle
     * @param skeletonNearBorders
     * @param max
     * @return int[][]
     */
    public static int[][] findSkeleton(Environment status, boolean treatWideOpenSpaceAsObstacle,
            boolean skeletonNearBorders, int max) {
        Environment.Status[][] statusG = status.getFullStatus();
        int[][] freeSpaceGrid = new int[statusG.length][statusG[0].length];

        for (int i = 0; i < statusG.length; i++) {
            for (int j = 0; j < statusG[0].length; j++) {
                if (statusG[i][j] == Environment.Status.unexplored) {//&& (!skeletonNearBorders || (!status.obstacleWithinDistance(j, j, 5)))) {
                    freeSpaceGrid[i][j] = 1;
                } else {
                    freeSpaceGrid[i][j] = 0;
                }
            }
        }

        if (treatWideOpenSpaceAsObstacle) {
            for (int i = 0; i < statusG.length; i++) {
                for (int j = 0; j < statusG[0].length; j++) {
                    if (freeSpaceGrid[i][j] == 1 && inWideOpenSpace(freeSpaceGrid, i, j)) {
                        //Only one pixel is not threated as an obstacle
                        freeSpaceGrid[i][j] = 0;
                        freeSpaceGrid[i - 1][j] = 0;
                        freeSpaceGrid[i + 1][j] = 0;
                        freeSpaceGrid[i][j + 1] = 0;
                        freeSpaceGrid[i][j - 1] = 0;
                    }
                }
            }
        }

        //skeleton = distanceTransform(skeleton);
        int[][] skeleton;
        if (!skeletonNearBorders) {
            skeleton = skeletonize(freeSpaceGrid, max);
        } else {
            skeleton = skeletonizeNearBorders(freeSpaceGrid);
        }
        return skeleton;
    }

    /**
     * Gives a list of Points at the junctions of the skeleton
     *
     * @param skeleton
     * @param grid
     * @param endPoints use ends as junctions too
     * @return
     */
    public static LinkedList<Point> findJunctionPoints(int[][] skeleton, IntGrid grid, boolean endPoints) {
        LinkedList<Point> junctions = new LinkedList<Point>();

        // Pass 1:  find key points (junctions)
        for (int i = 2; i < skeleton.length - 2; i++) {
            for (int j = 2; j < skeleton[0].length - 2; j++) {
                if (skeleton[i][j] != 0 && numNonzeroNeighbors(skeleton, i, j) >= 3 && neighborTraversal(skeleton, i, j) >= 3) {
                    junctions.add(new Point(i, j));
                }
                if (endPoints && skeleton[i][j] != 0 && numNonzeroNeighbors(skeleton, i, j) == 1) {
                    junctions.add(new Point(i, j));
                }
            }
        }

        // Pass 3:  prune points too close to another rv point or too close to an obstacle
        Point p;
        for (int i = junctions.size() - 1; i >= 0; i--) {
            p = junctions.get(i);
            if (grid.obstacleWithinDistance(p.x, p.y, SimConstants.WALL_DISTANCE)) {
                junctions.remove(i);
                continue;
            }
            for (int j = junctions.size() - 1; j >= 0; j--) {
                if (i != j && p.distance(junctions.get(j)) < SimConstants.KEY_POINT_DISTANCE) {
                    junctions.remove(i);
                    break;
                }
            }
        }

        return junctions;
    }

    /**
     * Gives a list of Points with all JunctionPoints with points in between these junctions with
     * distances around 'distance' on the skeleton
     *
     * @param skeleton
     * @param grid
     * @param endpoints use ends of skeleton as keypoints too?
     * @param distance distance between keypoints
     * @return
     */
    public static LinkedList<Point> findKeyPoints(int[][] skeleton, IntGrid grid, boolean endpoints, int distance) {
        LinkedList<Point> rvPts = new LinkedList<Point>();

        // Pass 1:  find key points (junctions)
        rvPts.addAll(findJunctionPoints(skeleton, grid, endpoints));

        // Pass 2:  fill in gaps
        LinkedList<Point> pts = gridToList(skeleton);
        boolean addToRVlist;
        for (Point p : pts) {
            // First check if it's an endpoint
            if (numNonzeroNeighbors(skeleton, p.x, p.y) < 2) {
                rvPts.add(p);
                continue;
            }

            // Second check if it's far away from all other rv points
            addToRVlist = true;
            for (Point q : rvPts) {
                if (p.distance(q) < distance) {
                    addToRVlist = false;
                    break;
                }
            }
            if (addToRVlist) {
                rvPts.add(p);
            }
        }

        Point p;

        // Pass 3:  prune points too close to another rv point or too close to an obstacle
        for (int i = rvPts.size() - 1; i >= 0; i--) {
            p = rvPts.get(i);
            if (grid.obstacleWithinDistance(p.x, p.y, SimConstants.WALL_DISTANCE)) {
                rvPts.remove(i);
                continue;
            }
            for (int j = rvPts.size() - 1; j >= 0; j--) {
                if (p.distance(rvPts.get(j)) < 20 && i != j) {
                    rvPts.remove(i);
                    break;
                }
            }
        }

        return rvPts;
    }

    /**
     * Fill a grid with the id of the next topological node (on key-points) by expanding around the
     * nodes simultaniously
     *
     * @param occGrid
     * @param keyPoints
     * @param nodes
     * @return
     */
    public static int[][] fillKeyAreas(OccupancyGrid occGrid, LinkedList<Point> keyPoints, HashMap<Integer, TopologicalNode> nodes) {
        int[][] areaGrid = new int[occGrid.width][occGrid.height];
        LinkedList<Point> pointsOfInterest = new LinkedList<Point>();

        //Initialize areaGrid - set unexplored areas to UNEXPLORED_NODE_ID, obstacles to -1, free space to 0.
        for (int i = 0; i < areaGrid.length; i++) {
            for (int j = 0; j < areaGrid[0].length; j++) {
                if ((occGrid.obstacleAt(i, j)) || (i == 0) || (j == 0) || (i == (areaGrid.length - 1)) || (j == (areaGrid[0].length - 1))) {
                    areaGrid[i][j] = -1;
                } else if ((!occGrid.obstacleAt(i, j)) && (!occGrid.freeSpaceAt(i, j))) {
                    areaGrid[i][j] = SimConstants.UNEXPLORED_NODE_ID; //unexplored space
                } else {
                    areaGrid[i][j] = 0;
                }
            }
        }

        // initialize the keypoint of each Node to the node's ID; add the immediate neighbour cells to "points of interest" list
        for (TopologicalNode t : nodes.values()) {
            if (t.getID() == SimConstants.UNEXPLORED_NODE_ID) {
                continue;
            }
            Point p = t.getPosition();
            if ((areaGrid[p.x][p.y] >= 0) && (areaGrid[p.x][p.y] < SimConstants.UNEXPLORED_NODE_ID)) {
                areaGrid[p.x][p.y] = t.getID();
                if (areaGrid[p.x - 1][p.y] == 0) {
                    pointsOfInterest.add(new Point(p.x - 1, p.y));
                }
                if (areaGrid[p.x + 1][p.y] == 0) {
                    pointsOfInterest.add(new Point(p.x + 1, p.y));
                }
                if (areaGrid[p.x][p.y - 1] == 0) {
                    pointsOfInterest.add(new Point(p.x, p.y - 1));
                }
                if (areaGrid[p.x][p.y + 1] == 0) {
                    pointsOfInterest.add(new Point(p.x, p.y + 1));
                }
                if (areaGrid[p.x - 1][p.y - 1] == 0) {
                    pointsOfInterest.add(new Point(p.x - 1, p.y - 1));
                }
                if (areaGrid[p.x - 1][p.y + 1] == 0) {
                    pointsOfInterest.add(new Point(p.x - 1, p.y + 1));
                }
                if (areaGrid[p.x + 1][p.y + 1] == 0) {
                    pointsOfInterest.add(new Point(p.x + 1, p.y + 1));
                }
                if (areaGrid[p.x + 1][p.y - 1] == 0) {
                    pointsOfInterest.add(new Point(p.x + 1, p.y - 1));
                }
            }
        }
        int maxQueueSize;

        maxQueueSize = 0;
        while (!pointsOfInterest.isEmpty()) {
            if (pointsOfInterest.size() > maxQueueSize) {
                maxQueueSize = pointsOfInterest.size();
            }
            Point p = pointsOfInterest.iterator().next();
            pointsOfInterest.remove(p);
            if (areaGrid[p.x][p.y] == 0) //cell unassigned to any node and freeSpace
            {
                // calculate what area most surrounding cells belong to
                int nw = 0;
                int n = 0;
                int ne = 0;
                int w = 0;
                int e = 0;
                int sw = 0;
                int s = 0;
                int se = 0;

                if ((areaGrid[p.x - 1][p.y - 1] > 0) && (areaGrid[p.x - 1][p.y - 1] < SimConstants.UNEXPLORED_NODE_ID)) {
                    //nw has a node
                    //If any other neighbor of p is the same node, nw++
                    if (areaGrid[p.x][p.y - 1] == areaGrid[p.x - 1][p.y - 1]) {
                        nw++;
                    }
                    if (areaGrid[p.x + 1][p.y - 1] == areaGrid[p.x - 1][p.y - 1]) {
                        nw++;
                    }
                    if (areaGrid[p.x - 1][p.y] == areaGrid[p.x - 1][p.y - 1]) {
                        nw++;
                    }
                    if (areaGrid[p.x + 1][p.y] == areaGrid[p.x - 1][p.y - 1]) {
                        nw++;
                    }
                    if (areaGrid[p.x - 1][p.y + 1] == areaGrid[p.x - 1][p.y - 1]) {
                        nw++;
                    }
                    if (areaGrid[p.x][p.y + 1] == areaGrid[p.x - 1][p.y - 1]) {
                        nw++;
                    }
                    if (areaGrid[p.x + 1][p.y + 1] == areaGrid[p.x - 1][p.y - 1]) {
                        nw++;
                    }
                } else {
                    // if p is obstacle, unexploredNode nw--
                    nw--;
                }
                if ((areaGrid[p.x][p.y - 1] > 0) && (areaGrid[p.x][p.y - 1] < SimConstants.UNEXPLORED_NODE_ID)) {
                    if (areaGrid[p.x - 1][p.y - 1] == areaGrid[p.x][p.y - 1]) {
                        n++;
                    }
                    if (areaGrid[p.x + 1][p.y - 1] == areaGrid[p.x][p.y - 1]) {
                        n++;
                    }
                    if (areaGrid[p.x - 1][p.y] == areaGrid[p.x][p.y - 1]) {
                        n++;
                    }
                    if (areaGrid[p.x + 1][p.y] == areaGrid[p.x][p.y - 1]) {
                        n++;
                    }
                    if (areaGrid[p.x - 1][p.y + 1] == areaGrid[p.x][p.y - 1]) {
                        n++;
                    }
                    if (areaGrid[p.x][p.y + 1] == areaGrid[p.x][p.y - 1]) {
                        n++;
                    }
                    if (areaGrid[p.x + 1][p.y + 1] == areaGrid[p.x][p.y - 1]) {
                        n++;
                    }
                } else {
                    n--;
                }
                if ((areaGrid[p.x + 1][p.y - 1] > 0) && (areaGrid[p.x + 1][p.y - 1] < SimConstants.UNEXPLORED_NODE_ID)) {
                    if (areaGrid[p.x - 1][p.y - 1] == areaGrid[p.x + 1][p.y - 1]) {
                        ne++;
                    }
                    if (areaGrid[p.x][p.y - 1] == areaGrid[p.x + 1][p.y - 1]) {
                        ne++;
                    }
                    if (areaGrid[p.x - 1][p.y] == areaGrid[p.x + 1][p.y - 1]) {
                        ne++;
                    }
                    if (areaGrid[p.x + 1][p.y] == areaGrid[p.x + 1][p.y - 1]) {
                        ne++;
                    }
                    if (areaGrid[p.x - 1][p.y + 1] == areaGrid[p.x + 1][p.y - 1]) {
                        ne++;
                    }
                    if (areaGrid[p.x][p.y + 1] == areaGrid[p.x + 1][p.y - 1]) {
                        ne++;
                    }
                    if (areaGrid[p.x + 1][p.y + 1] == areaGrid[p.x + 1][p.y - 1]) {
                        ne++;
                    }
                } else {
                    ne--;
                }
                if ((areaGrid[p.x - 1][p.y] > 0) && (areaGrid[p.x - 1][p.y] < SimConstants.UNEXPLORED_NODE_ID)) {
                    if (areaGrid[p.x - 1][p.y - 1] == areaGrid[p.x - 1][p.y]) {
                        w++;
                    }
                    if (areaGrid[p.x][p.y - 1] == areaGrid[p.x - 1][p.y]) {
                        w++;
                    }
                    if (areaGrid[p.x + 1][p.y - 1] == areaGrid[p.x - 1][p.y]) {
                        w++;
                    }
                    if (areaGrid[p.x + 1][p.y] == areaGrid[p.x - 1][p.y]) {
                        w++;
                    }
                    if (areaGrid[p.x - 1][p.y + 1] == areaGrid[p.x - 1][p.y]) {
                        w++;
                    }
                    if (areaGrid[p.x][p.y + 1] == areaGrid[p.x - 1][p.y]) {
                        w++;
                    }
                    if (areaGrid[p.x + 1][p.y + 1] == areaGrid[p.x - 1][p.y]) {
                        w++;
                    }
                } else {
                    w--;
                }
                if ((areaGrid[p.x + 1][p.y] > 0) && (areaGrid[p.x + 1][p.y] < SimConstants.UNEXPLORED_NODE_ID)) {
                    if (areaGrid[p.x - 1][p.y - 1] == areaGrid[p.x + 1][p.y]) {
                        e++;
                    }
                    if (areaGrid[p.x][p.y - 1] == areaGrid[p.x + 1][p.y]) {
                        e++;
                    }
                    if (areaGrid[p.x + 1][p.y - 1] == areaGrid[p.x + 1][p.y]) {
                        e++;
                    }
                    if (areaGrid[p.x - 1][p.y] == areaGrid[p.x + 1][p.y]) {
                        e++;
                    }
                    if (areaGrid[p.x - 1][p.y + 1] == areaGrid[p.x + 1][p.y]) {
                        e++;
                    }
                    if (areaGrid[p.x][p.y + 1] == areaGrid[p.x + 1][p.y]) {
                        e++;
                    }
                    if (areaGrid[p.x + 1][p.y + 1] == areaGrid[p.x + 1][p.y]) {
                        e++;
                    }
                } else {
                    e--;
                }
                if ((areaGrid[p.x - 1][p.y + 1] > 0) && (areaGrid[p.x - 1][p.y + 1] < SimConstants.UNEXPLORED_NODE_ID)) {
                    if (areaGrid[p.x - 1][p.y - 1] == areaGrid[p.x - 1][p.y + 1]) {
                        sw++;
                    }
                    if (areaGrid[p.x][p.y - 1] == areaGrid[p.x - 1][p.y + 1]) {
                        sw++;
                    }
                    if (areaGrid[p.x + 1][p.y - 1] == areaGrid[p.x - 1][p.y + 1]) {
                        sw++;
                    }
                    if (areaGrid[p.x - 1][p.y] == areaGrid[p.x - 1][p.y + 1]) {
                        sw++;
                    }
                    if (areaGrid[p.x + 1][p.y] == areaGrid[p.x - 1][p.y + 1]) {
                        sw++;
                    }
                    if (areaGrid[p.x][p.y + 1] == areaGrid[p.x - 1][p.y + 1]) {
                        sw++;
                    }
                    if (areaGrid[p.x + 1][p.y + 1] == areaGrid[p.x - 1][p.y + 1]) {
                        sw++;
                    }
                } else {
                    sw--;
                }

                if ((areaGrid[p.x][p.y + 1] > 0) && (areaGrid[p.x][p.y + 1] < SimConstants.UNEXPLORED_NODE_ID)) {
                    if (areaGrid[p.x - 1][p.y - 1] == areaGrid[p.x][p.y + 1]) {
                        s++;
                    }
                    if (areaGrid[p.x][p.y - 1] == areaGrid[p.x][p.y + 1]) {
                        s++;
                    }
                    if (areaGrid[p.x + 1][p.y - 1] == areaGrid[p.x][p.y + 1]) {
                        s++;
                    }
                    if (areaGrid[p.x - 1][p.y] == areaGrid[p.x][p.y + 1]) {
                        s++;
                    }
                    if (areaGrid[p.x + 1][p.y] == areaGrid[p.x][p.y + 1]) {
                        s++;
                    }
                    if (areaGrid[p.x - 1][p.y + 1] == areaGrid[p.x][p.y + 1]) {
                        s++;
                    }
                    if (areaGrid[p.x + 1][p.y + 1] == areaGrid[p.x][p.y + 1]) {
                        s++;
                    }
                } else {
                    s--;
                }

                if ((areaGrid[p.x + 1][p.y + 1] > 0) && (areaGrid[p.x + 1][p.y + 1] < SimConstants.UNEXPLORED_NODE_ID)) {
                    if (areaGrid[p.x - 1][p.y - 1] == areaGrid[p.x + 1][p.y + 1]) {
                        se++;
                    }
                    if (areaGrid[p.x][p.y - 1] == areaGrid[p.x + 1][p.y + 1]) {
                        se++;
                    }
                    if (areaGrid[p.x + 1][p.y - 1] == areaGrid[p.x + 1][p.y + 1]) {
                        se++;
                    }
                    if (areaGrid[p.x - 1][p.y] == areaGrid[p.x + 1][p.y + 1]) {
                        se++;
                    }
                    if (areaGrid[p.x + 1][p.y] == areaGrid[p.x + 1][p.y + 1]) {
                        se++;
                    }
                    if (areaGrid[p.x - 1][p.y + 1] == areaGrid[p.x + 1][p.y + 1]) {
                        se++;
                    }
                    if (areaGrid[p.x][p.y + 1] == areaGrid[p.x + 1][p.y + 1]) {
                        se++;
                    }
                } else {
                    se--;
                }

                //The direction with the most occurences wins and may give the cell his nodeID
                if (nw >= n && nw >= ne && nw >= w && nw >= e && nw >= sw && nw >= s && nw >= se) {
                    areaGrid[p.x][p.y] = areaGrid[p.x - 1][p.y - 1];
                } else if (n >= nw && n >= ne && n >= w && n >= e && n >= sw && n >= s && n >= se) {
                    areaGrid[p.x][p.y] = areaGrid[p.x][p.y - 1];
                } else if (ne >= nw && ne >= n && ne >= w && ne >= e && ne >= sw && ne >= s && ne >= se) {
                    areaGrid[p.x][p.y] = areaGrid[p.x + 1][p.y - 1];
                } else if (w >= nw && w >= n && w >= ne && w >= e && w >= sw && w >= s && w >= se) {
                    areaGrid[p.x][p.y] = areaGrid[p.x - 1][p.y];
                } else if (e >= nw && e >= n && e >= ne && e >= w && e >= sw && e >= s && e >= se) {
                    areaGrid[p.x][p.y] = areaGrid[p.x + 1][p.y];
                } else if (sw >= nw && sw >= n && sw >= ne && sw >= w && sw >= e && sw >= s && sw >= se) {
                    areaGrid[p.x][p.y] = areaGrid[p.x - 1][p.y + 1];
                } else if (s >= nw && s >= n && s >= ne && s >= w && s >= e && s >= sw && s >= se) {
                    areaGrid[p.x][p.y] = areaGrid[p.x][p.y + 1];
                } else if (se >= nw && se >= n && se >= ne && se >= w && se >= e && se >= sw && se >= s) {
                    areaGrid[p.x][p.y] = areaGrid[p.x + 1][p.y + 1];
                }
                if (areaGrid[p.x][p.y] == -1) {
                    //unexplored won the competition... not useful!
                    areaGrid[p.x][p.y] = 0;
                } else if ((areaGrid[p.x][p.y] > 0) && (areaGrid[p.x][p.y] < SimConstants.UNEXPLORED_NODE_ID)) {
                    occGrid.setFinalTopologicalMapCell(p.x, p.y);
                    nodes.get(areaGrid[p.x][p.y]).addCell(p);
                }


                /*if (pointsOfInterest.size() > 1000000)
                {
                    // there have to be duplicates, let's prune
                    for (Point pnt: pointsOfInterest)
                    {
                        if (areaGrid[pnt.x][pnt.y] != 0)
                            pointsOfInterest.remove(pnt);
                    }
                }*/
                if (pointsOfInterest.size() < Math.max(10000000, (areaGrid.length * areaGrid[0].length) * 10)) {
                    if (areaGrid[p.x - 1][p.y] == 0) {
                        pointsOfInterest.add(new Point(p.x - 1, p.y));
                    }
                    if (areaGrid[p.x + 1][p.y] == 0) {
                        pointsOfInterest.add(new Point(p.x + 1, p.y));
                    }
                    if (areaGrid[p.x][p.y - 1] == 0) {
                        pointsOfInterest.add(new Point(p.x, p.y - 1));
                    }
                    if (areaGrid[p.x][p.y + 1] == 0) {
                        pointsOfInterest.add(new Point(p.x, p.y + 1));
                    }
                    if (areaGrid[p.x - 1][p.y - 1] == 0) {
                        pointsOfInterest.add(new Point(p.x - 1, p.y - 1));
                    }
                    if (areaGrid[p.x - 1][p.y + 1] == 0) {
                        pointsOfInterest.add(new Point(p.x - 1, p.y + 1));
                    }
                    if (areaGrid[p.x + 1][p.y - 1] == 0) {
                        pointsOfInterest.add(new Point(p.x + 1, p.y - 1));
                    }
                    if (areaGrid[p.x + 1][p.y + 1] == 0) {
                        pointsOfInterest.add(new Point(p.x + 1, p.y + 1));
                    }
                } else {
                    System.err.println("!!!!TOPOLOGICAL map might contain errors, pointsofinterest queue exhausted!");
                }
            }
        }
        if (SimConstants.DEBUG_OUTPUT) {
            System.out.println("Max queue size: " + maxQueueSize);
        }

        //Are there some areas we haven't assigned to a node yet?
        //Fill with unexplored to avoid holes in the map
        /*for (int i = 1; i < areaGrid.length - 1; i++) {
            for (int j = 1; j < areaGrid[0].length - 1; j++) {
                if (areaGrid[i][j] == 0) {
                    areaGrid[i][j] = SimConstants.UNEXPLORED_NODE_ID;
                }
            }
        }*/
        return areaGrid;
    }

    /**
     * test every cell if one surrounding cell has another id (not obstacle or freeSpace). Add these
     * cells to the returned List
     *
     * @param areaGrid
     * @return
     */
    public static LinkedList<Point> findKeyAreaBorders(int[][] areaGrid) {
        LinkedList<Point> borderPts = new LinkedList<Point>();
        for (int i = 0; i < areaGrid.length; i++) {
            for (int j = 0; j < areaGrid[0].length; j++) {
                if (areaGrid[i][j] > 0) {
                    int curCell = areaGrid[i][j];
                    if (areaGrid[i + 1][j] != curCell
                            || areaGrid[i - 1][j] != curCell
                            || areaGrid[i][j - 1] != curCell
                            || areaGrid[i][j + 1] != curCell
                            || areaGrid[i - 1][j - 1] != curCell
                            || areaGrid[i - 1][j + 1] != curCell
                            || areaGrid[i + 1][j - 1] != curCell
                            || areaGrid[i + 1][j + 1] != curCell) {
                        borderPts.add(new Point(i, j));
                    }

                }
            }
        }
        return borderPts;
    }

    public static boolean withinDistanceBySkeleton(int[][] skeleton, Point a, Point b, Point came_from, int distance, long timeStart) {
        // just to be 100% sure we don't freeze here
        long curRealTime = System.currentTimeMillis();
        if (curRealTime - timeStart > SimConstants.MAX_TIME_DISTANCE_BY_SKELETON) {
            return a.equals(b);
        }

        if (distance == 0) {
            return a.equals(b);
        }
        if (a.equals(b)) {
            return true;
        }
        if ((skeleton[a.x][a.y] != 1) || (skeleton[b.x][b.y] != 1)) {
            return false;
        }

        boolean top = false;
        //boolean topr = false;
        boolean right = false;
        //boolean bottomr = false;
        boolean bottom = false;
        //boolean bottoml = false;
        boolean left = false;
        //boolean topl = false;

        Point child = new Point(a.x, a.y - 1);
        if (!child.equals(came_from) && (skeleton[child.x][child.y] == 1)) {
            top = withinDistanceBySkeleton(skeleton, child, b, a, distance - 1, timeStart);
        }

        /*child = new Point(a.x+1, a.y-1);
        if (!child.equals(came_from) && (skeleton[child.x][child.y] == 1))
            topr = withinDistanceBySkeleton(skeleton, child, b, a, distance-1);*/
        child = new Point(a.x + 1, a.y);
        if (!child.equals(came_from) && (skeleton[child.x][child.y] == 1)) {
            right = withinDistanceBySkeleton(skeleton, child, b, a, distance - 1, timeStart);
        }

        /*child = new Point(a.x+1, a.y+1);
        if (!child.equals(came_from) && (skeleton[child.x][child.y] == 1))
            bottomr = withinDistanceBySkeleton(skeleton, child, b, a, distance-1);*/
        child = new Point(a.x, a.y + 1);
        if (!child.equals(came_from) && (skeleton[child.x][child.y] == 1)) {
            bottom = withinDistanceBySkeleton(skeleton, child, b, a, distance - 1, timeStart);
        }

        /*child = new Point(a.x-1, a.y+1);
        if (!child.equals(came_from) && (skeleton[child.x][child.y] == 1))
            bottoml = withinDistanceBySkeleton(skeleton, child, b, a, distance-1);*/
        child = new Point(a.x - 1, a.y);
        if (!child.equals(came_from) && (skeleton[child.x][child.y] == 1)) {
            left = withinDistanceBySkeleton(skeleton, child, b, a, distance - 1, timeStart);
        }

        /*child = new Point(a.x-1, a.y-1);
        if (!child.equals(came_from) && (skeleton[child.x][child.y] == 1))
            topl = withinDistanceBySkeleton(skeleton, child, b, a, distance-1);*/
 /*child = new Point(a.x+1, a.y-1);
        if (!child.equals(came_from) && (skeleton[child.x][child.y] == 1))
            topr = withinDistanceBySkeleton(skeleton, child, b, a, distance-1);*/
        return top || right || bottom || left;// || topl || topr || bottomr || bottoml;
    }

    public static LinkedList<Point> findBorderRVPoints(int[][] skeleton, OccupancyGrid occGrid) {
        int DISTANCE = 40;

        LinkedList<Point> rvPts = new LinkedList<Point>();
        boolean add;

        // Pass 1:  find key points (junctions)
        for (int i = 2; i < skeleton.length - 2; i++) {
            for (int j = 2; j < skeleton[0].length - 2; j++) {
                if (numNonzeroNeighbors(skeleton, i, j) >= 3 && neighborTraversal(skeleton, i, j) >= 3) {
                    rvPts.add(new Point(i, j));
                }
            }
        }

        // Pass 2:  fill in gaps
        LinkedList<Point> pts = gridToList(skeleton);
        boolean addToRVlist;
        for (Point p : pts) {
            // First check if it's an endpoint
            if (numNonzeroNeighbors(skeleton, p.x, p.y) < 2) {
                rvPts.add(p);
                continue;
            }

            // Second check if it's far away from all other rv points
            addToRVlist = true;
            for (Point q : rvPts) {
                if (p.distance(q) < DISTANCE) {
                    long realtimeStartSkeleton = System.currentTimeMillis();
                    if (withinDistanceBySkeleton(skeleton, p, q, p, DISTANCE, realtimeStartSkeleton)) {
                        addToRVlist = false;
                        break;
                    }
                }
            }

            if (addToRVlist) {
                rvPts.add(p);
            }
        }

        Point p;
        // Pass 3:  prune points too close to another rv point or too close to an obstacle
        for (int i = rvPts.size() - 1; i >= 0; i--) {
            p = rvPts.get(i);
            if (occGrid.obstacleWithinDistance(p.x, p.y, SimConstants.WALL_DISTANCE)) {
                rvPts.remove(i);
                continue;
            }
            for (int j = rvPts.size() - 1; j >= 0; j--) {
                if (p.distance(rvPts.get(j)) < DISTANCE && i != j) {
                    long realtimeStartSkeleton = System.currentTimeMillis();
                    if ((p.distance(rvPts.get(j)) <= 10)
                            || (withinDistanceBySkeleton(skeleton, p, rvPts.get(j), p, DISTANCE, realtimeStartSkeleton))) {
                        rvPts.remove(i);
                        break;
                    }
                }
            }
        }

        //System.out.println("---  Removed " + counter + "junctions that were too close");
        return rvPts;
    }

    public static LinkedList<Point> findSecondBorderRVPoints(LinkedList<Point> borderRVPoints, RealAgent agent, Point goal) {
        LinkedList<Point> secondRVPoints = new LinkedList<Point>();
        if (SimConstants.DEBUG_OUTPUT) {
            System.out.println("Border points: " + borderRVPoints.size());
        }
        long realtimeStart = System.currentTimeMillis();
        int counter = 1;
        for (Point rvPoint : borderRVPoints) {
            //System.out.println("Processing: " + counter + " / " + borderRVPoints.size());
            secondRVPoints.add(findSecondRVPoint(agent, rvPoint, goal, SimConstants.MIN_RV_THROUGH_WALL_ACCEPT_RATIO));
            counter++;
        }
        if (SimConstants.DEBUG_OUTPUT) {
            System.out.println("Checked all candidate points, took " + (System.currentTimeMillis() - realtimeStart) + "ms.");
        }
        return secondRVPoints;
    }

    /**
     * find the second RV point in a pair. (one agent goes to the first point, other goes to the
     * second) The second point is found through wall, within comm range, that gives an advantage
     * heading to the goal
     *
     * @param agent
     * @param firstRV
     * @param goal
     * @param minAcceptableRatio
     * @return
     */
    public static Point findSecondRVPoint(RealAgent agent, Point firstRV, Point goal, double minAcceptableRatio) {
        LinkedList<Point> candidatePoints = new LinkedList<Point>();

        OccupancyGrid occGrid = agent.getOccupancyGrid();

        int pointSkip = 1;

        Polygon commPoly = PropModel1.getRangeForRV(occGrid,
                firstRV.x, firstRV.y, 0, 200);

        int counter = 0;
        for (int i = 0; i < commPoly.npoints; i++) {
            Point p = new Point(commPoly.xpoints[i], commPoly.ypoints[i]);
            if (occGrid.freeSpaceAt(p.x, p.y) /*&& !env.directLinePossible(firstRV.x, firstRV.y, p.x, p.y)*/) {
                if (counter % pointSkip == 0) {
                    if (!occGrid.directLinePossible(firstRV, p, true, false)) {
                        candidatePoints.add(p);
                    }
                }
                counter++;
            }
        }
        // let's find which candidate point is closest to goal

        Point secondRV = firstRV;

        if (candidatePoints.size() > 0) {
            double minDistance = agent.calculatePath(firstRV, goal, false).getLength();

            for (Point p : candidatePoints) {
                double distance = agent.calculatePath(p, goal, false).getLength();
                if (distance < minDistance) {
                    minDistance = distance;
                    secondRV = p;
                }
            }

            double minDistanceDirect;
            minDistanceDirect = agent.calculatePath(firstRV, goal, false).getLength() - (agent.getCommRange() / SimConstants.DEFAULT_SPEED);

            //communication through the wall gives no advantage
            if ((minDistanceDirect < 0) || ((minDistance / minDistanceDirect) > minAcceptableRatio)) {
                secondRV = firstRV;
            }
        }
        return secondRV;
    }

    public static void writeToFile(int[][] grid) {
        try (PrintWriter outFile = new PrintWriter(new FileWriter("skeleton1.txt"))) {

            outFile.println(grid[0].length);
            outFile.println(grid.length);
            for (int i = 0; i < grid[0].length; i++) {
                for (int[] grid1 : grid) {
                    outFile.print(grid1[i]);
                }
                outFile.println();
            }

        } catch (IOException e) {
            System.err.println("Error writing to file!");
        }

    }

    public static void writeToImg(int[][] grid, LinkedList<Point> points, String filename) {
        BufferedImage image = new BufferedImage(grid.length, grid[0].length, BufferedImage.TYPE_INT_RGB);
        for (int i = 0; i < grid.length; i++) {
            for (int j = 0; j < grid[0].length; j++) {
                if (grid[i][j] == 1) {
                    image.setRGB(i, j, Color.white.getRGB());
                } else {
                    image.setRGB(i, j, Color.black.getRGB());
                }
            }
        }
        if (points != null) {
            for (Point p : points) {
                image.setRGB(p.x, p.y, Color.MAGENTA.getRGB());
            }
        }
        try {
            ImageIO.write(image, "png", new File(filename + ".png"));
        } catch (IOException ex) {
            System.err.println("Could not write file");
        }
    }

    public static void main(String args[]) {

        SimulatorConfig tempSimConfig = new SimulatorConfig();
        tempSimConfig.loadEnvironment(SimConstants.DEFAULT_ENV_DIRECTORY + "maze1.png");

        int[][] skel = findSkeleton(tempSimConfig.getEnvironment(), false, false, Integer.MAX_VALUE);
        LinkedList<Point> junc = findKeyPoints(skel, tempSimConfig.getEnvironment(), true, 40);
        System.out.println(junc.size());
        writeToImg(skel, junc, "1");

        tempSimConfig.loadEnvironment(SimConstants.DEFAULT_ENV_DIRECTORY + "corridor.png");
        skel = findSkeleton(tempSimConfig.getEnvironment(), false, false, Integer.MAX_VALUE);
        junc = findKeyPoints(skel, tempSimConfig.getEnvironment(), true, 40);
        System.out.println(junc.size());
        writeToImg(skel, junc, "2");
        tempSimConfig.loadEnvironment(SimConstants.DEFAULT_ENV_DIRECTORY + "library.png");
        skel = findSkeleton(tempSimConfig.getEnvironment(), false, false, Integer.MAX_VALUE);
        junc = findKeyPoints(skel, tempSimConfig.getEnvironment(), true, 40);
        System.out.println(junc.size());
        writeToImg(skel, junc, "3");
        /*update = findSkeleton(tempSimConfig.getEnvironment(), true, false, 50);
        writeToImg(update, "2");
        update = findSkeleton(tempSimConfig.getEnvironment(), false, true, 50);
        writeToImg(update, "3");
        update = findSkeleton(tempSimConfig.getEnvironment(), true, true, 50);
        writeToImg(update, "4");*/

    }

    private static int[][] distanceTransform(int[][] inputGrid) {
        int width = inputGrid.length;
        int height = inputGrid[0].length;
        int temp;

        //0 means white,1 means black
        // Initialize matrix
        int matrix[][] = new int[width][height];
        /*        for(int i=0; i<width; i++)
            for(int j=0; j<height; j++)
                if(grid.freeSpaceAt(i, j))
                    matrix[i][j] = 0;
                else
                    matrix[i][j] = 1;
         */
        for (int i = 0; i < width; i++) {
            System.arraycopy(inputGrid[i], 0, matrix[i], 0, height);
        }

        // Forward pass
        for (int j = 0; j < height; j++) {
            for (int i = 0; i < width; i++) {
                // if this is an obstacle, set to 0
                if (matrix[i][j] == 1) {
                    continue;
                }

                if ((j == 0) || (j == height - 1)) {
                    matrix[i][j] = 1;
                    continue;
                }

                if ((i == 0) || (i == width - 1)) {
                    matrix[i][j] = 1;
                    continue;
                }

                temp = Math.min(matrix[i - 1][j], matrix[i][j - 1]);
                matrix[i][j] = temp + 1;

            }
        }

        // Backward pass
        for (int j = height - 1; j >= 0; j--) {
            for (int i = width - 1; i >= 0; i--) {
                if (matrix[i][j] == 1) {
                    continue;
                }

                if ((j == 0) || (j == height - 1)) {
                    matrix[i][j] = 1;
                    continue;
                }

                if ((i == 0) || (i == width - 1)) {
                    matrix[i][j] = 1;
                    continue;
                }

                temp = Math.min(matrix[i][j + 1], matrix[i + 1][j]);

                if (temp > (matrix[i][j] - 1)) {
                    temp = matrix[i][j] - 1;
                }

                matrix[i][j] = temp + 1;
            }
        }

        return matrix;
    }
}
