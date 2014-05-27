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
import java.io.*;

/**
 *
 * @author julh
 */
public class Walls {
    
    public static int[][] walls;
    
    // wall set 2:  many rooms
    private static void createWallSet2() {
        
            walls = new int[700][500];
            for(int i=0; i<500; i++)
                for(int j=0; j<700; j++)
                    walls[j][i] = 0;

        //Outline
        
        for(int i=0; i<700; i++) {
            walls[i][0] = 1;
            walls[i][499] = 1;
        }
        
        for(int i=0; i<500; i++) {
            walls[0][i] = 1;
            walls[699][i] = 1;
        }
        
        //Rooms along top right
        for(int i=0; i<=80; i++)
            walls[200][i] = 1;

        for(int i=0; i<=80; i++)
            walls[300][i] = 1;

        for(int i=0; i<=80; i++)
            walls[400][i] = 1;

        for(int i=0; i<=80; i++)
            walls[500][i] = 1;

        for(int i=0; i<=80; i++)
            walls[600][i] = 1;

        for(int i=200; i<=260; i++)
            walls[i][80] = 1;

        for(int i=280; i<=360; i++)
            walls[i][80] = 1;

        for(int i=380; i<=460; i++)
            walls[i][80] = 1;

        for(int i=480; i<=560; i++)
            walls[i][80] = 1;

        for(int i=580; i<=660; i++)
            walls[i][80] = 1;

        for(int i=680; i<700; i++)
            walls[i][80] = 1;
        
        //Rooms in middle upper right

        for(int i=120; i<=200; i++)
            walls[200][i] = 1;
        
        for(int i=120; i<=200; i++)
            walls[300][i] = 1;
        
        for(int i=120; i<=200; i++)
            walls[400][i] = 1;
        
        for(int i=120; i<=200; i++)
            walls[500][i] = 1;
        
        for(int i=120; i<=200; i++)
            walls[600][i] = 1;
        
        for(int i=120; i<=140; i++)
            walls[660][i] = 1;
        
        for(int i=160; i<=200; i++)
            walls[660][i] = 1;
        
        for(int i=200; i<=260; i++)
            walls[i][120] = 1;

        for(int i=280; i<=360; i++)
            walls[i][120] = 1;

        for(int i=380; i<=460; i++)
            walls[i][120] = 1;

        for(int i=480; i<=560; i++)
            walls[i][120] = 1;

        for(int i=580; i<=660; i++)
            walls[i][120] = 1;

        for(int i=200; i<=660; i++)
            walls[i][200] = 1;

        // Rooms top left
        
        for(int i=120; i<=160; i++)
            walls[i][40] = 1;

        for(int i=0; i<=60; i++)
            walls[i][100] = 1;

        for(int i=100; i<=160; i++)
            walls[i][100] = 1;

        for(int i=0; i<=60; i++)
            walls[i][200] = 1;

        for(int i=100; i<=160; i++)
            walls[i][200] = 1;

        for(int i=0; i<=60; i++)
            walls[i][300] = 1;

        for(int i=20; i<=100; i++)
            walls[60][i] = 1;

        for(int i=120; i<=200; i++)
            walls[60][i] = 1;

        for(int i=220; i<=300; i++)
            walls[60][i] = 1;

        for(int i=40; i<=100; i++)
            walls[100][i] = 1;

        for(int i=120; i<=260; i++)
            walls[100][i] = 1;

        for(int i=40; i<=200; i++)
            walls[160][i] = 1;

        // Rooms Bottom Left
        
        for(int i=80; i<=140; i++)
            walls[i][300] = 1;

        for(int i=0; i<=60; i++)
            walls[i][400] = 1;

        for(int i=80; i<=140; i++)
            walls[i][400] = 1;

        for(int i=80; i<=240; i++)
            walls[i][460] = 1;
        
        for(int i=420; i<500; i++)
            walls[40][i] = 1;

        for(int i=320; i<=400; i++)
            walls[60][i] = 1;

        for(int i=300; i<=460; i++)
            walls[80][i] = 1;

        for(int i=300; i<=360; i++)
            walls[140][i] = 1;

        for(int i=380; i<=400; i++)
            walls[140][i] = 1;

        for(int i=400; i<=460; i++)
            walls[160][i] = 1;

        for(int i=400; i<=460; i++)
            walls[240][i] = 1;

        // Central rooms
        
        for(int i=200; i<=360; i++)
            walls[i][260] = 1;
        
        for(int i=420; i<=460; i++)
            walls[i][260] = 1;
        
        for(int i=400; i<=460; i++)
            walls[i][320] = 1;
        
        for(int i=180; i<=460; i++)
            walls[i][400] = 1;
        
        for(int i=260; i<=400; i++)
            walls[200][i] = 1;

        for(int i=260; i<=400; i++)
            walls[400][i] = 1;

        for(int i=260; i<=320; i++)
            walls[460][i] = 1;

        for(int i=340; i<=460; i++)
            walls[460][i] = 1;

        // Bottom Right
        
        for(int i=520; i<=600; i++)
            walls[i][260] = 1;
        
        for(int i=640; i<=699; i++)
            walls[i][300] = 1;
        
        for(int i=640; i<=699; i++)
            walls[i][380] = 1;
        
        for(int i=640; i<=699; i++)
            walls[i][460] = 1;
        
        for(int i=500; i<=560; i++)
            walls[i][460] = 1;
        
        for(int i=260; i<=460; i++)
            walls[500][i] = 1;

        for(int i=260; i<=460; i++)
            walls[600][i] = 1;

        for(int i=340; i<=380; i++)
            walls[640][i] = 1;

        for(int i=400; i<=460; i++)
            walls[640][i] = 1;

        

    }
    
    // wall set 1:  outline only
    private static void createWallSet1() {

            walls = new int[300][200];
            for(int i=0; i<200; i++)
                for(int j=0; j<300; j++)
                    walls[j][i] = 0;

        for(int i=0; i<300; i++) {
            walls[i][0] = 1;
            walls[i][199] = 1;
        }
        
        for(int i=0; i<200; i++) {
            walls[0][i] = 1;
            walls[299][i] = 1;
        }
        

    }
    
    public static void main(String args[]) {
        String fileName = System.getProperty("user.dir") + "/config/lastWallConfig.txt";
        
        try{
            PrintWriter outFile = new PrintWriter(new FileWriter(fileName));
            
            createWallSet2();    
            
            for(int i=0; i<walls[0].length; i++){
                for(int j=0; j<walls.length; j++)
                    outFile.print(walls[j][i]);
                outFile.println();
            }

            outFile.close();
        }
        catch(IOException e){
            System.out.println("Error writing to file " + fileName);
        }
    }
}
