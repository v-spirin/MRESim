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
package exploration;

import agents.BasicAgent.ExploreState;
import config.RobotConfig;
import java.awt.Point;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

/**
 *
 * @author julh
 */
public class RunFromLog {
    public static Point takeStep(int time, String fileName, int agentNo) {
        Point nextStep = new Point(0, 0);
        
        File file = new File(fileName); 

        if ( file.exists() )           
        {                                         
            try{
                BufferedReader inFile = new BufferedReader(new FileReader(file));
                for(int i=0; i<time; i++)
                    inFile.readLine();
                
                String tokens[] = inFile.readLine().split("\\s");
                nextStep.x = Integer.parseInt(tokens[(agentNo-1)*6 + 1]);
                nextStep.y = Integer.parseInt(tokens[(agentNo-1)*6 + 2]);                
                inFile.close();
            }
            catch (NullPointerException e) {
                // No more data, run finished.
                return null;
            }
            catch (IOException e) {
                System.out.println("RunFromLog: Error -- could not read data from " + fileName);
            }
            catch (NumberFormatException e) {
                System.out.println("RunFromLog: Error -- incorrect data format in file " + fileName);
            }
        }
        return nextStep;
    }
    
    public static Point getGoal(int time, String fileName, int agentNo) {
        Point nextStep = new Point(0, 0);
        
        File file = new File(fileName); 

        if ( file.exists() )           
        {                                         
            try{
                BufferedReader inFile = new BufferedReader(new FileReader(file));
                for(int i=0; i<time; i++)
                    inFile.readLine();
                
                String tokens[] = inFile.readLine().split("\\s");
                nextStep.x = (int)Double.parseDouble(tokens[(agentNo-1)*6 + 3]);
                nextStep.y = (int)Double.parseDouble(tokens[(agentNo-1)*6 + 4]);                
                inFile.close();
            }
            catch (NullPointerException e) {
                // No more data, run finished.
                return null;
            }
            catch (IOException e) {
                System.out.println("RunFromLog: Error -- could not read data from " + fileName);
            }
            catch (NumberFormatException e) {
                System.out.println("RunFromLog: Error -- incorrect data format in file " + fileName);
            }
        }
        return nextStep;
    }
    
    public static ExploreState getState(int time, String fileName, int agentNo) {
        Point nextStep = new Point(0, 0);
        
        File file = new File(fileName); 

        if ( file.exists() )           
        {                                         
            try{
                BufferedReader inFile = new BufferedReader(new FileReader(file));
                for(int i=0; i<time; i++)
                    inFile.readLine();
                
                String tokens[] = inFile.readLine().split("\\s");
                String state = tokens[(agentNo-1)*6 + 6];
                if (state.equals("GetInfoFromChild")) return ExploreState.GetInfoFromChild;
                if (state.equals("Explore")) return ExploreState.Explore;
                if (state.equals("GiveParentInfo")) return ExploreState.GiveParentInfo;
                if (state.equals("GoToChild")) return ExploreState.GoToChild;
                if (state.equals("Initial")) return ExploreState.Initial;
                if (state.equals("OutOfService")) return ExploreState.OutOfService;
                if (state.equals("ReturnToParent")) return ExploreState.ReturnToParent;
                if (state.equals("WaitForChild")) return ExploreState.WaitForChild;
                if (state.equals("WaitForParent")) return ExploreState.WaitForParent;
                inFile.close();
            }
            catch (NullPointerException e) {
                // No more data, run finished.
                return null;
            }
            catch (IOException e) {
                System.out.println("RunFromLog: Error -- could not read data from " + fileName);
            }
            catch (NumberFormatException e) {
                System.out.println("RunFromLog: Error -- incorrect data format in file " + fileName);
            }
        }
        return ExploreState.Initial;
    }
    
    public static RobotConfig.roletype getRole(int time, String fileName, int agentNo) {
        Point nextStep = new Point(0, 0);
        
        File file = new File(fileName); 

        if ( file.exists() )           
        {                                         
            try{
                BufferedReader inFile = new BufferedReader(new FileReader(file));
                for(int i=0; i<time; i++)
                    inFile.readLine();
                
                String tokens[] = inFile.readLine().split("\\s");
                String role = tokens[(agentNo-1)*6 + 5];
                if (role.equals("BaseStation")) return RobotConfig.roletype.BaseStation;
                if (role.equals("Explorer")) return RobotConfig.roletype.Explorer;
                if (role.equals("Relay")) return RobotConfig.roletype.Relay;            
                inFile.close();
            }
            catch (NullPointerException e) {
                // No more data, run finished.
                return null;
            }
            catch (IOException e) {
                System.out.println("RunFromLog: Error -- could not read data from " + fileName);
            }
            catch (NumberFormatException e) {
                System.out.println("RunFromLog: Error -- incorrect data format in file " + fileName);
            }
        }
        return RobotConfig.roletype.BaseStation;
    }
}
