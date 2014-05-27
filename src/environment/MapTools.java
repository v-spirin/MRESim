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

import config.EnvLoader;
import environment.Environment.Status;
import java.awt.Point;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Random;

/**
 *
 * @author juliandehoog
 */
public class MapTools {

    public static Environment generateRandomMap() {
        Random generate = new Random();
        Environment env = new Environment(600, 800);

        int[][] vals = new int[800][600];

        // Create outer rim
        for(int i=0; i<800; i++) {
            vals[i][0] = 1;
            vals[i][599] = 1;
            env.setStatus(i, 0, Status.obstacle);
            env.setStatus(i, 599, Status.obstacle);
        }
        for(int j=0; j<600; j++) {
            vals[0][j] = 1;
            vals[799][j] = 1;
            env.setStatus(0, j, Status.obstacle);
            env.setStatus(799, j, Status.obstacle);
        }
        for(int k=1; k<799; k++)
            for(int m=1; m<599; m++)
                vals[k][m] = -50;

        int numDarkNeighbours;
        int random, p;
/*
        for(int t=1; t<=10; t++)
        for(int x=1; x<799; x++)
            for(int y=1; y<599; y++) {
                numDarkNeighbours = 0;
                if(env.obstacleAt(x-1,y)) numDarkNeighbours++;
                if(env.obstacleAt(x-1,y-1)) numDarkNeighbours++;
                if(env.obstacleAt(x,y-1)) numDarkNeighbours++;
                /*p = 20*numDarkNeighbours + generate.nextInt(20);
                random = generate.nextInt(100);
                if(random < p)
                    env.setStatus(x, y, Status.obstacle);
                if(numDarkNeighbours == 1)
                    p=90;
                /*else if(numDarkNeighbours == 2)
                    p=85;
                else if(numDarkNeighbours == 3)
                    p=15;
                else
                    p=10;
                p += generate.nextInt(10)-5;
                random = generate.nextInt(100);
                if(random < p)
                    env.setStatus(x, y, Status.obstacle);

            }*/

        int avg;
/*
        for(int x=1; x<799; x++)
            for(int y=1; y<599; y++) {
                avg = (vals[x-1][y-1]+vals[x-1][y]+vals[x][y-1]) / 3;
                random = generate.nextInt(11)-5;
                avg += random;
                if(avg>20) avg=25;
                if(avg<-20) avg=-25;
                vals[x][y]=avg;
                if(avg>0) env.setStatus(x, y, Status.obstacle);
            }*/

        LinkedList open = new LinkedList();
        Queue closed = new LinkedList();

        Point start1 = new Point(400,300);
        vals[400][300] = -10;
        Point start2 = new Point(200,150);
        vals[200][150] = -10;
        Point start3 = new Point(600,150);
        vals[600][150] = -10;
        Point start4 = new Point(200,450);
        vals[200][450] = -10;
        Point start5 = new Point(600,450);
        vals[600][450] = -10;
        open.add(start1);
        open.add(start2);
        open.add(start3);
        open.add(start4);
        open.add(start5);

        Point curr, neighbour;
        
        while(!open.isEmpty()) {
            System.out.println(open.size() + " " + closed.size());
            curr = (Point)open.poll();
            closed.add(curr);
            for(int x=curr.x-1; x<=curr.x+1; x++)
                for(int y=curr.y-1; y<=curr.y+1; y++) {
                    neighbour = new Point(x,y);
                    if(vals[x][y] == -50) {
                            open.add(generate.nextInt(open.size()), neighbour);
                            random = generate.nextInt(5)-2;
                            vals[x][y] = vals[curr.x][curr.y]+random;
                            if(vals[x][y] > 40) vals[x][y] = 40;
                            if(vals[x][y] < -40) vals[x][y] = -40;
                    }
                }
        }


        // Now add / delete some chunks
        int x, y, width, length, change;
        for(int i=0; i<generate.nextInt(10) + 8; i++) {
            width = generate.nextInt(65) + 20;
            length = generate.nextInt(65) + 20;
            x = generate.nextInt(800);
            y = generate.nextInt(600);
            change = generate.nextInt(31)-15;
            for(int m=Math.max(1, x-width/2); m<Math.min(799, x+width/2); m++)
                for(int n=Math.max(1, y-length/2); n<Math.min(599, y+length/2); n++)
                    vals[m][n] += change;
        }

        // now convolve gaussian for blur
        // http://homepages.inf.ed.ac.uk/rbf/HIPR2/gsmooth.htm
        int runningTotal;
        int[][] newVals = new int[800][600];
        for(int i=0; i<800; i++) {
            newVals[i][0] = vals[i][0];
            newVals[i][1] = vals[i][1];
            newVals[i][598] = vals[i][598];
            newVals[i][599] = vals[i][599];
        }
        for(int j=0; j<600; j++) {
            newVals[0][j] = vals[0][j];
            newVals[1][j] = vals[1][j];
            newVals[798][j] = vals[798][j];
            newVals[799][j] = vals[799][j];
        }

        for(int i=2; i<=797; i++)
            for(int j=2; j<=597; j++) {
                runningTotal =  1 * vals[i-2][j-2] +
                                4 * vals[i-1][j-2] +
                                7 * vals[i][j-2] +
                                4 * vals[i+1][j-2] +
                                1 * vals[i+2][j-2] +
                                4 * vals[i-2][j-1] +
                                16* vals[i-1][j-1] +
                                26* vals[i][j-1] +
                                16* vals[i+1][j-1] +
                                4 * vals[i+2][j-1] +
                                7 * vals[i-2][j] +
                                26* vals[i-1][j] +
                                41* vals[i][j] +
                                26* vals[i+1][j] +
                                7 * vals[i+2][j] +
                                4 * vals[i-2][j+1] +
                                16* vals[i-1][j+1] +
                                26* vals[i][j+1] +
                                16* vals[i+1][j+1] +
                                4 * vals[i+2][j+1] +
                                1 * vals[i-2][j+2] +
                                4 * vals[i-1][j+2] +
                                7 * vals[i][j+2] +
                                4 * vals[i+1][j+2] +
                                1 * vals[i+2][j+2];
                newVals[i][j] = runningTotal / 273;

            }

        for(int i=0; i<800; i++)
            for(int j=0; j<600; j++)
                vals[i][j] = newVals[i][j];

        // dirty as.  do it all once more.
        for(int i=2; i<=797; i++)
            for(int j=2; j<=597; j++) {
                runningTotal =  1 * vals[i-2][j-2] +
                                4 * vals[i-1][j-2] +
                                7 * vals[i][j-2] +
                                4 * vals[i+1][j-2] +
                                1 * vals[i+2][j-2] +
                                4 * vals[i-2][j-1] +
                                16* vals[i-1][j-1] +
                                26* vals[i][j-1] +
                                16* vals[i+1][j-1] +
                                4 * vals[i+2][j-1] +
                                7 * vals[i-2][j] +
                                26* vals[i-1][j] +
                                41* vals[i][j] +
                                26* vals[i+1][j] +
                                7 * vals[i+2][j] +
                                4 * vals[i-2][j+1] +
                                16* vals[i-1][j+1] +
                                26* vals[i][j+1] +
                                16* vals[i+1][j+1] +
                                4 * vals[i+2][j+1] +
                                1 * vals[i-2][j+2] +
                                4 * vals[i-1][j+2] +
                                7 * vals[i][j+2] +
                                4 * vals[i+1][j+2] +
                                1 * vals[i+2][j+2];
                newVals[i][j] = runningTotal / 273;

            }

        for(int i=1; i<799; i++)
            for(int j=1; j<599; j++)
                if(newVals[i][j]>0) env.setStatus(i, j, Status.obstacle);

        // Recreate outer rim
        for(int i=0; i<800; i++) {
            env.setStatus(i, 0, Status.obstacle);
            env.setStatus(i, 1, Status.obstacle);
            env.setStatus(i, 598, Status.obstacle);
            env.setStatus(i, 599, Status.obstacle);
        }
        for(int j=0; j<600; j++) {
            env.setStatus(0, j, Status.obstacle);
            env.setStatus(1, j, Status.obstacle);
            env.setStatus(798, j, Status.obstacle);
            env.setStatus(799, j, Status.obstacle);
        }

        return env;
    }
    public static Environment generateRandomMap2() {
        Random generate = new Random();
        Environment env = new Environment(600, 800);

        int[][] vals = new int[800][600];

        // Create outer rim
        for(int i=0; i<800; i++) {
            vals[i][0] = 1;
            vals[i][599] = 1;
            env.setStatus(i, 0, Status.obstacle);
            env.setStatus(i, 599, Status.obstacle);
        }
        for(int j=0; j<600; j++) {
            vals[0][j] = 1;
            vals[799][j] = 1;
            env.setStatus(0, j, Status.obstacle);
            env.setStatus(799, j, Status.obstacle);
        }
        for(int k=1; k<799; k++)
            for(int m=1; m<599; m++)
                vals[k][m] = 1;

        // Now add / delete some chunks
        int x, y, width, length, change;
        for(int i=0; i<generate.nextInt(80) + 30; i++) {
            width = generate.nextInt(200) + 20;
            length = generate.nextInt(200) + 20;
            x = generate.nextInt(800);
            y = generate.nextInt(600);
            //change = generate.nextInt(31)-15;
            for(int m=Math.max(1, x-width/2); m<Math.min(799, x+width/2); m++)
                for(int n=Math.max(1, y-length/2); n<Math.min(599, y+length/2); n++)
                    vals[m][n] = -1;
        }

        for(int i=1; i<799; i++)
            for(int j=1; j<599; j++)
                if(vals[i][j]>0) env.setStatus(i, j, Status.obstacle);

        return env;
    }


    // Map metric calculation as discussed in part in
    // J.T.Bjorke: Framework for Entropy-based Map Evaluation (1996)
    public static double calculateMapMetric1(Environment env) {
        int t_s = 0;  // transition same
        int t_d = 0;  // transition different

        for(int i=1; i<(env.getColumns()-1); i++)
            for(int j=1; j<(env.getRows()-1); j++) {
                if(env.statusAt(i, j) == env.statusAt(i-1,j)) t_s++; else t_d++;    // NOTE statusAt changes as exploration unfolds
                if(env.statusAt(i, j) == env.statusAt(i,j-1)) t_s++; else t_d++;    // so as is, only measure metric before start of exploration
                if(env.statusAt(i, j) == env.statusAt(i+1,j)) t_s++; else t_d++;
                if(env.statusAt(i, j) == env.statusAt(i,j+1)) t_s++; else t_d++;
            }

        System.out.println(className() + "Total transitions same: " + t_s);
        System.out.println(className() + "Total transitions diff: " + t_d);

        int total = t_s + t_d;
        double t_s_frac = (double)t_s / (double)total;
        double t_d_frac = (double)t_d / (double)total;

        return -1*(t_s_frac * Math.log(t_s_frac) + t_d_frac * Math.log(t_d_frac));
    }

    public static double calculateMapMetric2(Environment env) {
        int t_s = 0;  // transition same
        int t_d = 0;  // transition different

        for(int i=1; i<(env.getColumns()-1); i++)
            for(int j=1; j<(env.getRows()-1); j++) {
                if(env.statusAt(i, j) == env.statusAt(i-1,j)) t_s++; else t_d++;    // NOTE statusAt changes as exploration unfolds
                if(env.statusAt(i, j) == env.statusAt(i,j-1)) t_s++; else t_d++;    // so as is, only measure metric before start of exploration
                if(env.statusAt(i, j) == env.statusAt(i+1,j)) t_s++; else t_d++;
                if(env.statusAt(i, j) == env.statusAt(i,j+1)) t_s++; else t_d++;
            }

        int total = t_s + t_d;

        return (double)t_d/(double)total*100;
    }

    public static void main(String args[]) {
        
        /* //String fileName = System.getProperty("user.dir") + "/config/lastWallConfig.txt";
        //String fileName = System.getProperty("user.dir") + "/environments/radish_vasche5.png";
        //String fileName = System.getProperty("user.dir") + "/environments/puremess.png";
        //String fileName = System.getProperty("user.dir") + "/environments/blank.png";
        //String fileName = System.getProperty("user.dir") + "/environments/fingers.png";
        //String fileName = System.getProperty("user.dir") + "/environments/maze1.png";
        //String fileName = System.getProperty("user.dir") + "/environments/Clutter_600x800.txt";
        String fileName = System.getProperty("user.dir") + "/environments/randomMap1.txt";
        //String fileName = System.getProperty("user.dir") + "/environments/Bare_600x800.txt";

        Environment env = EnvLoader.loadWallConfig(fileName);

        double metric1 = calculateMapMetric1(env);
        double metric2 = calculateMapMetric2(env);
        System.out.println(className() + "Map \"" + fileName + "\" has metric1: " + metric1 + ".");
        System.out.println(className() + "Map \"" + fileName + "\" has metric2: " + metric2 + ".");
         //*/

        for(int i=7; i<10; i++)
            EnvLoader.saveWallConfig(generateRandomMap2(), System.getProperty("user.dir") + "/environments/random" + i +".png");
    }

    public static String className() {
        return ("[MapTools] ");
    }
}
