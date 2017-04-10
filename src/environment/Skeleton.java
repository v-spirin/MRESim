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
import config.Constants;
import config.SimulatorConfig;
import exploration.rendezvous.Rendezvous;
import java.awt.Point;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import path.TopologicalNode;

/**
 *
 * @author julh
 */
public class Skeleton {

    // SOME CODE IN THIS FILE TAKEN FROM Sudhanshu Kumar:
    // http://www.sourcecodesworld.com/source/show.asp?ScriptID=692
    //1:newfndistrans.java
//2:A bmp monochrome image(preferably 512-512)
// the image from mspaint or any other source
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

                //temp = findmin(matrix[i-1][j], matrix[i-1][j-1], matrix[i][j-1], matrix[i+1][j-1]);
                temp = findmin(matrix[i - 1][j], matrix[i][j - 1]);
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

                //temp=findmin(matrix[i-1][j+1],matrix[i][j+1],matrix[i+1][j+1],matrix[i+1][j]);
                temp = findmin(matrix[i][j + 1], matrix[i + 1][j]);

                if (temp > (matrix[i][j] - 1)) {
                    temp = matrix[i][j] - 1;
                }

                matrix[i][j] = temp + 1;
            }
        }

        return matrix;
    }

    private static int findmin(int num1, int num2)//,int num3,int num4)
    {
        int min = 4000;
        if (min > num1) {
            min = num1;
        }
        if (min > num2) {
            min = num2;
        }
        //if(min>num3)min=num3;
        //if(min>num4)min=num4;
        return (min);
    }

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

    static int[][] skeletonize(int[][] grid) {
        return skeletonize(grid, Integer.MAX_VALUE);
    }

    static int[][] findCSpace(int[][] grid) {
        return skeletonize(grid, 0);
    }

    // this method finds the points that are within a margin of the obstacles
    static int[][] skeletonizeNearBorders(int[][] grid) {
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

    public static boolean obstacleWithinDistance(OccupancyGrid grid, int x, int y, int minDistance) {
        for (int i = x - minDistance; i <= x + minDistance; i++) {
            for (int j = y - minDistance; j <= y + minDistance; j++) {
                if (i >= 0 && j >= 0 && i < grid.width && j < grid.height
                        && new Point(x, y).distance(i, j) <= minDistance
                        && grid.obstacleAt(i, j)) {
                    return true;
                }
            }
        }
        return false;
    }

    static int[][] skeletonize(int[][] grid, int max) {
        long realtimeStart = System.currentTimeMillis();
        int width = grid.length;
        int height = grid[0].length;

        int u1[][] = new int[width][height];
        int u2[][] = new int[width][height];

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                u1[i][j] = grid[i][j];
                u2[i][j] = grid[i][j];
            }
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
                if (Constants.DEBUG_OUTPUT) {
                    System.out.println("skeletonize method took " + (System.currentTimeMillis() - realtimeStart) + "ms.");
                }
                return u2;
            }
            for (int i = 2; i < width - 2; i++) {
                System.arraycopy(u2[i], 2, u1[i], 2, height - 2 - 2);
            }

            counter++;
        }

        if (Constants.DEBUG_OUTPUT) {
            System.out.println("skeletonize method took " + (System.currentTimeMillis() - realtimeStart) + "ms.");
        }
        return null;
    }

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

    static void printMatrix(int[][] mat) {
        for (int j = 0; j < mat[0].length; j++) {
            for (int[] mat1 : mat) {
                System.out.print(mat1[j] + " ");
            }
            System.out.println();
        }
        System.out.println();
        System.out.println();
    }

    private static boolean inWideOpenSpace(int[][] grid, int x, int y) {
        for (int i = Math.max(0, x - 50); i <= Math.min(grid.length - 1, x + 50); i++) {
            for (int j = Math.max(0, y - 50); j <= Math.min(grid[0].length - 1, y + 50); j++) {
                if (grid[i][j] == 0) {
                    return false;
                }
            }
        }

        return true;
    }

    //<editor-fold defaultstate="collapsed" desc="Find Skeleton">
    public static int[][] findSkeleton(OccupancyGrid grid) {
        return findSkeleton(grid, true, false);
    }

    public static int[][] findSkeletonNearBorders(OccupancyGrid grid) {
        return findSkeleton(grid, false, true);
    }

    public static int[][] findSkeleton(OccupancyGrid grid, boolean treatWideOpenSpaceAsObstacle,
            boolean skeletonNearBorders) {
        long realtimeStart = System.currentTimeMillis();
        int[][] freeSpaceGrid = new int[grid.width][grid.height];

        for (int i = 0; i < grid.width; i++) {
            for (int j = 0; j < grid.height; j++) {
                if (grid.freeSpaceAt(i, j) && (!skeletonNearBorders || (!obstacleWithinDistance(grid, i, j, 5)))) {
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
                        freeSpaceGrid[i][j] = 0;
                    }
                }
            }
        }

        //skeleton = distanceTransform(skeleton);
        int[][] skeleton;
        if (!skeletonNearBorders) {
            skeleton = skeletonize(freeSpaceGrid);
        } else {
            skeleton = skeletonizeNearBorders(freeSpaceGrid);
        }
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("findSkeleton took " + (System.currentTimeMillis() - realtimeStart) + "ms.");
        }
        return skeleton;
    }

    public static int[][] findSkeleton(Environment.Status[][] status, int max) {
        int[][] freeSpaceGrid = new int[status.length][status[0].length];

        for (int i = 0; i < status.length; i++) {
            for (int j = 0; j < status[0].length; j++) {
                if (status[i][j] == Environment.Status.unexplored) {
                    freeSpaceGrid[i][j] = 1;
                } else {
                    freeSpaceGrid[i][j] = 0;
                }
            }
        }

        //skeleton = distanceTransform(skeleton);
        int[][] skeleton = skeletonize(freeSpaceGrid, max);
        return skeleton;
    }
    //</editor-fold>

    public static LinkedList<Point> findKeyPoints(int[][] skeleton, OccupancyGrid occGrid) {
        LinkedList<Point> rvPts = new LinkedList<Point>();
        boolean add;

        // Pass 1:  find key points (junctions)
        for (int i = 2; i < skeleton.length - 2; i++) {
            for (int j = 2; j < skeleton[0].length - 2; j++) {
                if (numNonzeroNeighbors(skeleton, i, j) >= 3 && neighborTraversal(skeleton, i, j) >= 3 && skeleton[i][j] != 0) {
                    rvPts.add(new Point(i, j));
                }
                //if (numNonzeroNeighbors(skeleton, i, j) == 1 && skeleton[i][j] != 0)
                //    rvPts.add(new Point(i,j));
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
                if (p.distance(q) < 40) {
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
            if (occGrid.obstacleWithinDistance(p.x, p.y, Constants.WALL_DISTANCE)) {
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

    public static int[][] fillKeyAreas(OccupancyGrid occGrid, LinkedList<Point> keyPoints, HashMap<Integer, TopologicalNode> nodes) {
        int[][] areaGrid = new int[occGrid.width][occGrid.height];
        LinkedHashMap<Point, Boolean> pointsOfInterest = new LinkedHashMap<Point, Boolean>();

        //Initialize areaGrid - set unexplored areas to UNEXPLORED_NODE_ID, obstacles to -1, free space to 0.
        for (int i = 0; i < areaGrid.length; i++) {
            for (int j = 0; j < areaGrid[0].length; j++) {
                if ((occGrid.obstacleAt(i, j)) /*|| ((!occGrid.obstacleAt(i, j)) && (!occGrid.freeSpaceAt(i, j)))*/ || (i == 0) || (j == 0) || (i == (areaGrid.length - 1)) || (j == (areaGrid[0].length - 1))) {
                    areaGrid[i][j] = -1;
                } else if ((!occGrid.obstacleAt(i, j)) && (!occGrid.freeSpaceAt(i, j))) {
                    areaGrid[i][j] = Constants.UNEXPLORED_NODE_ID; //unexplored space
                } else {
                    areaGrid[i][j] = 0;
                }
            }
        }

        // initialize the keypoint of each Node to the node's ID; add the immediate neighbour cells to "points of interest" list
        int index = 0;
        for (Point p : keyPoints) {
            index++;
            if ((areaGrid[p.x][p.y] >= 0) && (areaGrid[p.x][p.y] < Constants.UNEXPLORED_NODE_ID)) {
                areaGrid[p.x][p.y] = index;
                if (areaGrid[p.x - 1][p.y] == 0) {
                    pointsOfInterest.put(new Point(p.x - 1, p.y), true);
                }
                if (areaGrid[p.x + 1][p.y] == 0) {
                    pointsOfInterest.put(new Point(p.x + 1, p.y), true);
                }
                if (areaGrid[p.x][p.y - 1] == 0) {
                    pointsOfInterest.put(new Point(p.x, p.y - 1), true);
                }
                if (areaGrid[p.x][p.y + 1] == 0) {
                    pointsOfInterest.put(new Point(p.x, p.y + 1), true);
                }
                if (areaGrid[p.x - 1][p.y - 1] == 0) {
                    pointsOfInterest.put(new Point(p.x - 1, p.y - 1), true);
                }
                if (areaGrid[p.x - 1][p.y + 1] == 0) {
                    pointsOfInterest.put(new Point(p.x - 1, p.y + 1), true);
                }
                if (areaGrid[p.x + 1][p.y + 1] == 0) {
                    pointsOfInterest.put(new Point(p.x + 1, p.y + 1), true);
                }
                if (areaGrid[p.x + 1][p.y - 1] == 0) {
                    pointsOfInterest.put(new Point(p.x + 1, p.y - 1), true);
                }
            }
        }
        int maxQueueSize;

        maxQueueSize = 0;
        while (!pointsOfInterest.isEmpty()) {
            if (pointsOfInterest.size() > maxQueueSize) {
                maxQueueSize = pointsOfInterest.size();
            }
            Point p = pointsOfInterest.keySet().iterator().next();
            pointsOfInterest.remove(p);
            if (areaGrid[p.x][p.y] == 0) //cell unassigned to any node
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

                if ((areaGrid[p.x - 1][p.y - 1] > 0) && (areaGrid[p.x - 1][p.y - 1] < Constants.UNEXPLORED_NODE_ID)) {
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
                    nw = -1;
                }
                if ((areaGrid[p.x][p.y - 1] > 0) && (areaGrid[p.x][p.y - 1] < Constants.UNEXPLORED_NODE_ID)) {
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
                    n = -1;
                }
                if ((areaGrid[p.x + 1][p.y - 1] > 0) && (areaGrid[p.x + 1][p.y - 1] < Constants.UNEXPLORED_NODE_ID)) {
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
                    ne = -1;
                }
                if ((areaGrid[p.x - 1][p.y] > 0) && (areaGrid[p.x - 1][p.y] < Constants.UNEXPLORED_NODE_ID)) {
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
                    w = -1;
                }
                if ((areaGrid[p.x + 1][p.y] > 0) && (areaGrid[p.x + 1][p.y] < Constants.UNEXPLORED_NODE_ID)) {
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
                    e = -1;
                }
                if ((areaGrid[p.x - 1][p.y + 1] > 0) && (areaGrid[p.x - 1][p.y + 1] < Constants.UNEXPLORED_NODE_ID)) {
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
                    sw = -1;
                }

                if ((areaGrid[p.x][p.y + 1] > 0) && (areaGrid[p.x][p.y + 1] < Constants.UNEXPLORED_NODE_ID)) {
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
                    s = -1;
                }

                if ((areaGrid[p.x + 1][p.y + 1] > 0) && (areaGrid[p.x + 1][p.y + 1] < Constants.UNEXPLORED_NODE_ID)) {
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
                    se = -1;
                }

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
                    areaGrid[p.x][p.y] = 0;
                } else if ((areaGrid[p.x][p.y] > 0) && (areaGrid[p.x][p.y] < Constants.UNEXPLORED_NODE_ID)) {
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
                if (pointsOfInterest.size() < 10000000) {
                    if (areaGrid[p.x - 1][p.y] == 0) {
                        pointsOfInterest.put(new Point(p.x - 1, p.y), true);
                    }
                    if (areaGrid[p.x + 1][p.y] == 0) {
                        pointsOfInterest.put(new Point(p.x + 1, p.y), true);
                    }
                    if (areaGrid[p.x][p.y - 1] == 0) {
                        pointsOfInterest.put(new Point(p.x, p.y - 1), true);
                    }
                    if (areaGrid[p.x][p.y + 1] == 0) {
                        pointsOfInterest.put(new Point(p.x, p.y + 1), true);
                    }
                    if (areaGrid[p.x - 1][p.y - 1] == 0) {
                        pointsOfInterest.put(new Point(p.x - 1, p.y - 1), true);
                    }
                    if (areaGrid[p.x - 1][p.y + 1] == 0) {
                        pointsOfInterest.put(new Point(p.x - 1, p.y + 1), true);
                    }
                    if (areaGrid[p.x + 1][p.y - 1] == 0) {
                        pointsOfInterest.put(new Point(p.x + 1, p.y - 1), true);
                    }
                    if (areaGrid[p.x + 1][p.y + 1] == 0) {
                        pointsOfInterest.put(new Point(p.x + 1, p.y + 1), true);
                    }
                } else {
                    System.err.println("!!!!TOPOLOGICAL map might contain errors, pointsofinterest queue exhausted!");
                }
            }
        }
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("Max queue size: " + maxQueueSize);
        }

        //Are there some areas we haven't assigned to a node yet?
        /*for (int i = 1; i < areaGrid.length-1; i++)
            for (int j = 1; j < areaGrid[0].length-1; j++)
            {
                if (areaGrid[i][j] == 0)
                {
                    if ((areaGrid[i - 1][j] > 0) 
                        || (areaGrid[i + 1][j] > 0)
                        || (areaGrid[i][j - 1] > 0)
                        || (areaGrid[i][j + 1] > 0)
                        || (areaGrid[i - 1][j + 1] > 0)
                        || (areaGrid[i - 1][j - 1] > 0)
                        || (areaGrid[i + 1][j - 1] > 0)
                        || (areaGrid[i + 1][j + 1] > 0))
                        pointsOfInterest.add(new Point(i, j));  
                }
            }*/
        return areaGrid;
    }

    public static LinkedList<Point> findKeyAreaBorders(int[][] keyAreas) {
        LinkedList<Point> borderPts = new LinkedList<Point>();
        for (int i = 0; i < keyAreas.length; i++) {
            for (int j = 0; j < keyAreas[0].length; j++) {
                if (keyAreas[i][j] > 0) {
                    int curCell = keyAreas[i][j];
                    if (keyAreas[i + 1][j] != curCell
                            || keyAreas[i - 1][j] != curCell
                            || keyAreas[i][j - 1] != curCell
                            || keyAreas[i][j + 1] != curCell
                            || keyAreas[i - 1][j - 1] != curCell
                            || keyAreas[i - 1][j + 1] != curCell
                            || keyAreas[i + 1][j - 1] != curCell
                            || keyAreas[i + 1][j + 1] != curCell) {
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
        if (curRealTime - timeStart > Constants.MAX_TIME_DISTANCE_BY_SKELETON) {
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
            if (occGrid.obstacleWithinDistance(p.x, p.y, Constants.WALL_DISTANCE)) {
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

        //System.out.println("---  Removed " + counter + "rvPts that were too close");
        return rvPts;
    }

    public static LinkedList<Point> findSecondBorderRVPoints(LinkedList<Point> borderRVPoints, RealAgent agent, Point goal) {
        LinkedList<Point> secondRVPoints = new LinkedList<Point>();
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("Border points: " + borderRVPoints.size());
        }
        long realtimeStart = System.currentTimeMillis();
        int counter = 1;
        for (Point rvPoint : borderRVPoints) {
            //System.out.println("Processing: " + counter + " / " + borderRVPoints.size());
            secondRVPoints.add(Rendezvous.findSecondRVPoint(agent, rvPoint, goal, Constants.MIN_RV_THROUGH_WALL_ACCEPT_RATIO));
            counter++;
        }
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("Checked all candidate points, took " + (System.currentTimeMillis() - realtimeStart) + "ms.");
        }
        return secondRVPoints;
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

    public static void main(String args[]) {

        SimulatorConfig tempSimConfig = new SimulatorConfig();

        int[][] update = findSkeleton(tempSimConfig.getEnvironment().getFullStatus(), 70);
        writeToFile(update);


        /*
        int[][] transform, grid = new int[20][10];
        
        for(int i=0; i<20; i++)
            for(int j=0; j<10; j++)
                grid[i][j] = 1;
        
        for(int i=0; i<10; i++) {
            grid[0][i] = 0;
            grid[1][i] = 0;
            grid[18][i] = 0;
            grid[19][i] = 0;
        }
        
        for(int i=0; i<20; i++) {
            grid[i][0] = 0;
            grid[i][1] = 0;
            grid[i][7] = 0;
            grid[i][8] = 0;
            grid[i][9] = 0;
        }
        
        /*grid[13][1] = 0;
        grid[14][2] = 0;
        grid[15][3] = 0;
        grid[16][4] = 0;
        grid[17][5] = 0;
        grid[18][6] = 0;
        grid[19][7] = 0;*/
 /*
        printMatrix(grid);
        //transform = distanceTransform(grid);
        //printMatrix(transform);
        transform = skeletonize(grid);
        printMatrix(transform);
         */
    }
}
