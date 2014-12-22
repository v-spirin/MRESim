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

import agents.*;
import agents.BasicAgent.ExploreState;
import environment.*;
import config.*;
import gui.*;
import communication.*;
import config.RobotConfig.roletype;
import config.SimulatorConfig.exptype;
import environment.Environment.Status;
import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.Point;
import java.awt.Polygon;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Random;
import path.Path;

/**
 *
 * @author Julian de Hoog
 */


public class SimulationFramework implements ActionListener {

// <editor-fold defaultstate="collapsed" desc="Class variables and Constructors">
    
    boolean pauseSimulation;                    // For stepping through simulation one step at a time

    MainGUI mainGUI;                            // Allows simulator to change image, data
    ExplorationImage image;                     // Image of environment
    Environment env;                            // The environment (walls, obstacles)
    RealAgent agent[];                           // The agents
    int numRobots;

    SimulatorConfig simConfig;

    Polygon agentRange[];                       // For visualization of agents' comm ranges
    
    Timer timer;                                // Drives simulation steps
    Random random;                              // For generating random debris

    int[] debrisTimer;                          // For aisleRoom random debris exercise (AAMAS2010)
    
    // Communication
    int[][] directCommTable;
    int[][] multihopCommTable;

    // Interesting data
    int timeElapsed;
    int jointAreaKnown;
    double pctAreaKnown;
    int avgCycleTime;
    long simStartTime;
    int totalArea;
    double avgComStationKnowledge;
    double avgAgentKnowledge;
    double avgTimeLastCommand;
    double totalDistanceTraveled;
    int numSwaps;

    RobotTeamConfig robotTeamConfig;
    

    public SimulationFramework(MainGUI maingui, RobotTeamConfig newRobotTeamConfig, SimulatorConfig newSimConfig, ExplorationImage img) {
        random = new Random();
        mainGUI = maingui;
        image = img;
        simConfig = newSimConfig;
        env = simConfig.getEnv();
        robotTeamConfig = newRobotTeamConfig;
        
        reset();
    }

    private void reset() {
        pauseSimulation = false;
        env = simConfig.getEnv();

        timeElapsed = 0;
        jointAreaKnown = 1;             // to prevent divide by 0
        pctAreaKnown = 0;
        totalArea = simConfig.getEnv().getTotalFreeSpace();
        avgComStationKnowledge = 0;
        avgAgentKnowledge = 0;
        avgTimeLastCommand = 0;
        avgCycleTime = 0;
        totalDistanceTraveled = 0;
        numSwaps = 0;

        createAgents(robotTeamConfig);

        // Initialize Timer
        timer = new Timer((Constants.TIME_INCREMENT*10+1) - simConfig.getSimRate()*Constants.TIME_INCREMENT, this);
        timer.setInitialDelay(Constants.INIT_DELAY);
        timer.setCoalesce(true);

        // Initialize Debris timing
        debrisTimer = new int[6];
        for(int i=0; i<6; i++)
           debrisTimer[i] = 0;

        // Initialize Image
        updateImage(true);
    }
            
    private void createAgents(RobotTeamConfig robotTeamConfig) {
        numRobots = robotTeamConfig.getNumRobots();
        agent = new RealAgent[numRobots];
        TeammateAgent teammate[] = new TeammateAgent[numRobots];
        agentRange = new Polygon[numRobots];
        
        // Create ComStation
        agent[0] = new ComStation(env.getColumns(), env.getRows(), robotTeamConfig.getRobotTeam().get(1));
        teammate[0] = new TeammateAgent(robotTeamConfig.getRobotTeam().get(1));
        agent[0].setChild(1);
        agent[0].setState(RealAgent.ExploreState.GetInfoFromChild);

        for(int i=1; i<numRobots; i++) {
            agent[i] = new RealAgent(env.getColumns(), env.getRows(), robotTeamConfig.getRobotTeam().get(i+1));
            agentRange[i] = null;
            agent[i].setChildRendezvous(new RVLocation(agent[0].getLocation()));
            agent[i].setParentRendezvous(new RVLocation(agent[0].getLocation()));
            agent[i].setState(RealAgent.ExploreState.Initial);
            
            teammate[i] = new TeammateAgent(robotTeamConfig.getRobotTeam().get(i+1));
        }
        
        // Give each agent its teammates
        for(int i=0; i<numRobots; i++)
            for(int j=0; j<numRobots; j++)
                if(j != i)
                    agent[i].addTeammate(teammate[j].copy());
    }
    
// </editor-fold>     
    
    public int getTotalArea()
    {
        return totalArea;
    }
    
    public int getTimeElapsed()
    {
        return timeElapsed;
    }

// <editor-fold defaultstate="collapsed" desc="Simulation Cycle">

    private void simulationCycle() {
        long realtimeStartCycle; 
        realtimeStartCycle = System.currentTimeMillis();
        if (timeElapsed == 1) simStartTime = System.currentTimeMillis();
        System.out.println("\n" + this.toString() + "************** CYCLE " + timeElapsed + " ******************\n");
        
        //set exploration goals at the start of the mission
        if (timeElapsed == 0)
        {
            for (int i = 0; i < numRobots; i++)
                agent[i].setGoalArea(env.getTotalFreeSpace());
        }
        
        agentSteps();               // move agents, simulate sensor data
        System.out.println(this.toString() + "agentSteps took " + (System.currentTimeMillis()-realtimeStartCycle) + "ms.\n");
        //if(timeElapsed % 7 == 0 || timeElapsed % 7 == 1) {
        long timer = System.currentTimeMillis();
        simulateCommunication();    // simulate communication
        System.out.println(this.toString() + "simulateCommunication took " + (System.currentTimeMillis()-timer) + "ms.\n");
        //timer = System.currentTimeMillis();
        if ((simConfig != null) && (simConfig.getExpAlgorithm() == SimulatorConfig.exptype.RoleBasedExploration)
                && (simConfig.roleSwitchAllowed()))
        {
            //Role switch should ideally be done by individual agents as they communicate, rather than here.
            switchRoles();              // switch roles
            //System.out.println(this.toString() + "switchRoles took " + (System.currentTimeMillis()-timer) + "ms.\n");
            //timer = System.currentTimeMillis();
            // second role switch check (to avoid duplicate relays)
            for(int i=1; i<numRobots; i++)
                for(int j=1; j<numRobots; j++)
                    if(i != j)
                        if(agent[i].getState() == ExploreState.ReturnToParent && !agent[i].isExplorer() &&
                           agent[j].getState() == ExploreState.ReturnToParent && !agent[j].isExplorer() &&
                           agent[i].getTeammate(agent[j].getID()).isInRange() &&
                           agent[i].getPath().getLength() < agent[j].getPath().getLength()) {
                                agent[i].setState(ExploreState.GoToChild);
                                agent[i].setStateTimer(0);
                                agent[i].addDirtyCells(agent[i].getPath().getAllPathPixels());
                                //System.out.println("\nSecondary switch: " + agent[i].getName() + " and " + agent[j].getName() + "\n");
                                Path path = agent[i].calculatePath(agent[i].getLocation(), agent[i].getChildRendezvous().getParentLocation());
                                agent[i].setPath(path);
                                agent[i].setCurrentGoal(agent[i].getChildRendezvous().getParentLocation());
                        }
            //System.out.println(this.toString() + "Second switch roles check took " + (System.currentTimeMillis()-timer) + "ms.\n");

        }
        //timer = System.currentTimeMillis();   
        simulateDebris();           // simulate dynamic environment
        //System.out.println(this.toString() + "simulateDebris took " + (System.currentTimeMillis()-timer) + "ms.\n");
        timer = System.currentTimeMillis();
        if (timeElapsed % Constants.UPDATE_AGENT_KNOWLEDGE_INTERVAL == 0)
        {
            updateAgentKnowledgeData();
        }
        updateGlobalData();         // update data
        //System.out.println(this.toString() + "updateGlobalData took " + (System.currentTimeMillis()-realtimeStartCycle) + "ms.\n");
       
        System.out.println(this.toString() + "updateGlobalData took " + (System.currentTimeMillis()-timer) + "ms.\n");
        //timer = System.currentTimeMillis();
        
        //System.out.println(this.toString() + "updateAgentKnowledgeData took " + (System.currentTimeMillis()-timer) + "ms.\n");
        //timer = System.currentTimeMillis();
        updateGUI();                // update GUI
        //System.out.println(this.toString() + "updateGUI took " + (System.currentTimeMillis()-timer) + "ms.\n");
        //timer = System.currentTimeMillis();
        logging();                  // perform logging as required
        //if ((timeElapsed % 10) == 0) verifyNoInfoGotLost(); //verify relaying works fine
        //System.out.println(this.toString() + "logging took " + (System.currentTimeMillis()-timer) + "ms.\n");
        int currentCycleTime = (int)(System.currentTimeMillis()-realtimeStartCycle);
        System.out.println(this.toString() + "Cycle complete, took " + currentCycleTime + "ms.\n");
        //avgCycleTime = (((timeElapsed - 1) * avgCycleTime) + currentCycleTime) / timeElapsed;
        checkPause();               // check whether user wanted to pause
        avgCycleTime = (int)(System.currentTimeMillis() - simStartTime) / timeElapsed;
        checkRunFinish();           // for scripting multiple runs, to max number of cycles
    }
    
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Start, Run and Stop">

    int runNumber;
    int runNumMax;

    private void updateRunConfig() {
        // This method can be used to set up batch simulations
        // TODO: replace with XML batch run configuration
        
        /*String root = "C:\\RAS2014\\simulations\\";
        String configs[] = {"M2_L4_R4_RB", "M2_L2_R4", 
                            "M2_L3_R4", "M2_L4_R4",
                            "M3_L1_R4", "M3_L2_R4",
                            "M3_L3_R4", "M3_L4_R4",
                            "M4_L1_R4", "M4_L2_R4",
                            "M4_L3_R4", "M4_L4_R4",
                            "M1_L1_R8", "M1_L2_R8",
                            "M1_L3_R8", "M1_L4_R8",};
        int map = 2;
        if (runNumber == 1)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.RoleBasedExploration);
            simConfig.setRoleSwitchAllowed(true);
            simConfig.setReplanningAllowed(false);
            simConfig.setStrictRoleSwitch(false);
            simConfig.setUseImprovedRendezvous(true);
            robotTeamConfig.loadConfig(root + "configs\\" + configs[0]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[0] + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[0] + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[0])).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[0]);
        }
        if (runNumber == 2)
        {
            map = 2;
            String config = configs[1];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 3)
        {
            map = 2;
            String config = configs[2];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 4)
        {
            map = 2;
            String config = configs[3];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 5)
        {
            map = 3;
            String config = configs[4];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 6)
        {
            map = 3;
            String config = configs[5];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 7)
        {
            map = 3;
            String config = configs[6];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 8)
        {
            map = 3;
            String config = configs[7];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 9)
        {
            map = 4;
            String config = configs[8];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 10)
        {
            map = 4;
            String config = configs[9];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 11)
        {
            map = 4;
            String config = configs[10];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 12)
        {
            map = 4;
            String config = configs[11];
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.95;
            robotTeamConfig.loadConfig(root + "configs\\" + config);
            simConfig.setLogDataFilename(root + "output\\sim\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + config + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + config + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 13)
        {
            runNumber = 33;
            /*
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.RoleBasedExploration);
            simConfig.setRoleSwitchAllowed(true);
            simConfig.setReplanningAllowed(false);
            simConfig.setStrictRoleSwitch(false);
            simConfig.setUseImprovedRendezvous(true);
            robotTeamConfig.loadConfig(root + "configs\\" + configs[7]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[7] + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[7] + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[7])).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[7]);
        }
        if (runNumber == 14)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.90;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[6]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 15)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.75;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[6]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 16)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.3;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[6]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[6] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        
        if (runNumber == 17)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.RoleBasedExploration);
            simConfig.setRoleSwitchAllowed(true);
            simConfig.setReplanningAllowed(false);
            simConfig.setStrictRoleSwitch(false);
            simConfig.setUseImprovedRendezvous(true);
            robotTeamConfig.loadConfig(root + "configs\\" + configs[9]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[9] + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[9] + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[9])).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[9]);
        }
        if (runNumber == 18)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.90;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[8]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 19)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.75;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[8]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 20)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.3;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[8]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[8] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 21)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.RoleBasedExploration);
            simConfig.setRoleSwitchAllowed(true);
            simConfig.setReplanningAllowed(false);
            simConfig.setStrictRoleSwitch(false);
            simConfig.setUseImprovedRendezvous(true);
            robotTeamConfig.loadConfig(root + "configs\\" + configs[11]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[11] + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[11] + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[11])).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[11]);
        }
        if (runNumber == 22)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.90;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[10]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 23)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.75;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[10]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 24)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.3;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[10]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[10] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 25)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.RoleBasedExploration);
            simConfig.setRoleSwitchAllowed(true);
            simConfig.setReplanningAllowed(false);
            simConfig.setStrictRoleSwitch(false);
            simConfig.setUseImprovedRendezvous(true);
            robotTeamConfig.loadConfig(root + "configs\\" + configs[13]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[13] + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[13] + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[13])).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[13]);
        }
        if (runNumber == 26)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.90;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[12]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 27)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.75;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[12]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 28)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.3;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[12]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[12] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 29)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.RoleBasedExploration);
            simConfig.setRoleSwitchAllowed(true);
            simConfig.setReplanningAllowed(false);
            simConfig.setStrictRoleSwitch(false);
            simConfig.setUseImprovedRendezvous(true);
            robotTeamConfig.loadConfig(root + "configs\\" + configs[15]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[15] + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[15] + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[15])).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[15]);
        }
        if (runNumber == 30)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.90;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[14]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 31)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.75;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[14]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        if (runNumber == 32)
        {
            simConfig.loadWallConfig(root + "maps\\" +  map + ".png");
            simConfig.setExpAlgorithm(exptype.FrontierExploration);
            simConfig.setFrontierAlgorithm(frontiertype.UtilReturn);
            simConfig.TARGET_INFO_RATIO = 0.3;
            robotTeamConfig.loadConfig(root + "configs\\" + configs[14]);
            simConfig.setLogDataFilename(root + "output\\sim\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            simConfig.setLogAgentsFilename(root + "output\\agent\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO + ".txt");
            try
            {
                (new File(root + "output\\video\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO)).mkdirs();
            } catch (Exception e)
            {

            }
            simConfig.setLogScreenshotsDirname(root + "output\\video\\" + configs[14] + "U" + simConfig.TARGET_INFO_RATIO);
        }
        
        mainGUI.updateFromRobotTeamConfig();*/

    }

    public void start() {
        /*runNumber = 1;
        runNumMax = 33;
        updateRunConfig();
        reset();*/
        //simConfig.TARGET_INFO_RATIO = 0.90;
        RandomWalk.generator.setSeed(Constants.RANDOM_SEED);
        System.out.println(this.toString() + "Starting exploration!");
        timer.start();
        simStartTime = System.currentTimeMillis();
    }

    private void restart() {        
        /*updateRunConfig();
        reset();*/
        System.out.println(this.toString() + "Restarting exploration!");
        simStartTime = System.currentTimeMillis();
        timer.start();
    }

    public void takeOneStep() {
        pauseSimulation = true;
    }
    
    private void checkPause() {
        if(pauseSimulation || timeElapsed % 15000 == 0) {
            pauseSimulation = false;
            this.pause();
        }
    }

    private boolean allAgentsDone() {
        /*for(RealAgent a: agent) {
            if(a.getID() == 1)
                continue;
            if(!a.isMissionComplete())
                return false;
            if(!a.getTeammate(1).isInRange())
                return false;
        }
        return true;*/
        return (((double)agent[0].getAreaKnown()/(double)totalArea) >= Constants.TERRITORY_PERCENT_EXPLORED_GOAL);
    }

    private void checkRunFinish() {
        if(timeElapsed >= 10000 || allAgentsDone()) {
            timer.stop();
            runNumber++;
            if(runNumber < runNumMax)
                restart();
        }
    }
    
    public void pause() {
        timer.stop();
        System.out.println(this.toString() + "Pausing exploration!");
    }
    
    public void kill() {
        timer.stop();
        System.out.println(this.toString() + "Resetting exploration!");
    }
    
    public void actionPerformed(ActionEvent e) {
        simulationCycle();
    }
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Agent Steps, Sensor Data">

    private void agentSteps() {
        //long realtimeStartAgentCycle;        
        //Point nextStep = new Point(0,0);         // The next location that an agent wants to go to
        //double[] sensorData;    // Sensor data for an agent at its next location
        
        agent[0].flush();
        
        List<Thread> threads = new ArrayList<Thread>();
        
        for(int i=1; i<agent.length; i++) {
            Runnable task = new AgentStepRunnable(agent[i], simConfig, timeElapsed, env, this);
            Thread worker = new Thread(task);
            worker.setName(agent[i].toString());
            worker.start();
            threads.add(worker);
            
            // <editor-fold defaultstate="collapsed" desc="NoThreads">
            /*
            //realtimeStartAgentCycle = System.currentTimeMillis();
            double distance_left = agent[i].getSpeed();
            while (distance_left > 0)
            {
                System.out.println(agent[i].toString() + " distance left: " + distance_left);
                nextStep = agent[i].takeStep(timeElapsed, simConfig);
                if(nextStep == null) {   
                    //mainGUI.runComplete();  // run complete
                    //return;
                    System.out.println("ERROR: agent " + i + " chose NULL step!");
                    nextStep = agent[i].getLocation();
                }
                agent[i].flush();

                // Check to make sure step is legal
                if(env.directLinePossible(agent[i].getX(), agent[i].getY(), nextStep.x, nextStep.y)) {
                    //check here we don't 'teleport'
                    double dist = agent[i].getLocation().distance(nextStep);
                    if (dist < 0.01) break;
                    if (dist > 5)
                    {
                        System.out.println(agent[i].toString() + " !!!!! SOMETHING SERIOUSLY WRONG !!!!!");
                        System.out.println(agent[i].toString() + " dist is " + dist);
                        System.out.println(agent[i].toString() + " location is (" + agent[i].getX() + ", " + agent[i].getY() + ")");
                        System.out.println(agent[i].toString() + " next step is " + nextStep);
                        System.out.println(agent[i].toString() + " goal is " + agent[i].getCurrentGoal());                        
                    }
                    System.out.println(agent[i].toString() + " distance to next path point: " + dist);
                    if (dist > distance_left) {
                        System.out.println(agent[i].toString() + " exceeded speed. Distance left: " + distance_left + ", dist to next path point: " + dist);
                        double ratio = distance_left / dist;
                        nextStep.x = agent[i].getX() + (int)Math.round((nextStep.x - agent[i].getX()) * ratio);
                        nextStep.y = agent[i].getY() + (int)Math.round((nextStep.y - agent[i].getY()) * ratio);
                        if (!env.directLinePossible(agent[i].getX(), agent[i].getY(), nextStep.x, nextStep.y))
                        {
                            nextStep.x = agent[i].getX();
                            nextStep.y = agent[i].getY();
                            System.out.println(agent[i].toString() + " directLinePossible returned wrong result!");
                        }
                        System.out.println(agent[i].toString() + " speed corrected. Now is: " + agent[i].getLocation().distance(nextStep));
                        distance_left = 0;
                    } else
                    {
                        distance_left = distance_left - dist;
                    }
                    sensorData = findSensorData(agent[i], nextStep);
                    agent[i].writeStep(nextStep, sensorData);
                }
                else
                    agent[i].setEnvError(true);

                //System.out.println(agent[i].toString() + "Agent cycle complete, took " + (System.currentTimeMillis()-realtimeStartAgentCycle) + "ms.");
            }
            */
            //</editor-fold>                       
        }
        
        for(int i=0; i<threads.size(); i++) {
            try
            {
                threads.get(i).join();
            } catch (Exception e)
            {
                System.out.println("Thread " + i + " threw exception " + e.getMessage());
            }
        }
    }
    
    // Simulates data from laser range finder
    protected double[] findSensorData(RealAgent agent, Point nextLoc) {
        double currRayAngle, heading;
        int prevRayX, prevRayY;
        int currRayX, currRayY;
        double sensorData[] = new double[181];
        
        // Quick check: if agent hasn't moved, no new sensor data 
        // 22.04.2010 Julian commented this out to make frontier exp work
        // if(agent.getLocation().equals(nextLoc))
        //    return null;
    
        if(agent.getLocation().equals(nextLoc))
            heading = agent.getHeading();
        else
            heading = Math.atan2(nextLoc.y - agent.getY(), nextLoc.x - agent.getX());
        
        //For every degree
        for(int i=0; i<=180; i+=1) {
            prevRayX = nextLoc.x;
            prevRayY = nextLoc.y;
            
            currRayAngle = heading - Math.PI/2 + Math.PI/180*i;
            
            for(double m=1; m<=agent.getSenseRange(); m+=1) {
                currRayX = nextLoc.x + (int)(m * Math.cos(currRayAngle));
                currRayY = nextLoc.y + (int)(m * Math.sin(currRayAngle));
                
                if(!env.locationExists(currRayX, currRayY)){
                    sensorData[i] = nextLoc.distance(prevRayX, prevRayY);
                    break;
                }
                else if(env.statusAt(currRayX, currRayY) == Environment.Status.obstacle) {
                    sensorData[i] = nextLoc.distance(currRayX, currRayY);
                    break;
                }
                else if(m >= agent.getSenseRange()) {
                    sensorData[i] = nextLoc.distance(currRayX, currRayY);
                    break;
                }
                else {
                    prevRayX = currRayX;
                    prevRayY = currRayY;
                }
            }
        }
        
        return sensorData;
    }    
    
    // update area known if needed
    private void updateAgentKnowledgeData()
    {
        if (simConfig.getExpAlgorithm() == exptype.RunFromLog)
            return; //Nothing to do here
        
        for(int i = 0; i < agent.length; i++) {
            agent[i].updateAreaKnown();
        }
        
    }
// </editor-fold>     
  
// <editor-fold defaultstate="collapsed" desc="Communication">

    private void simulateCommunication() {
        //long realtimeStart = System.currentTimeMillis();
        //long realtimeStart2;
        //System.out.println(this.toString() + "Simulating communication ... ");

        detectCommunication();
        //System.out.println(Constants.INDENT + "detectCommunication took " + (System.currentTimeMillis()-realtimeStart) + "ms.");

        // Exchange data
        for(int i=0; i<numRobots-1; i++)
            for(int j=i+1; j<numRobots; j++)
                if(multihopCommTable[i][j] == 1) {
                        //realtimeStart2 = System.currentTimeMillis();
                        DataMessage msgFromFirst = new DataMessage(agent[i], directCommTable[i][j]);
                        DataMessage msgFromSecond = new DataMessage(agent[j], directCommTable[i][j]);
                                                
                        agent[i].receiveMessage(msgFromSecond);
                        agent[j].receiveMessage(msgFromFirst);
                        
                        //System.out.println(Constants.INDENT + "Communication between " +
                        //                    agent[i].getName() + " and " +
                        //                    agent[j].getName() + " took " +
                        //                    (System.currentTimeMillis() - realtimeStart2) + "ms.");
                        // For periodic return frontier exp
                        if(simConfig.getExpAlgorithm() == SimulatorConfig.exptype.FrontierExploration &&
                           simConfig.getFrontierAlgorithm() == SimulatorConfig.frontiertype.PeriodicReturn &&
                           i == 0 && agent[j].frontierPeriodicState == 1) {
                            agent[j].frontierPeriodicState = 0;
                            agent[j].periodicReturnInterval += 10;
                        }
                    }


        //realtimeStart = System.currentTimeMillis();
        // Update post comm
        for(int q=0; q<numRobots; q++)
            agent[q].updateAfterCommunication();
        
        //verifyNoInfoGotLost();
        //System.out.println(Constants.INDENT + "updateAfterCommunication took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
        //System.out.println(Constants.INDENT + "Communication complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
    
    // <editor-fold defaultstate="collapsed" desc="DELETE">
    /*private void verifyNoInfoGotLost()
    {
        //build common occgrid of all agents
        int counter = 0;
        OccupancyGrid commonGrid = new OccupancyGrid(agent[0].getOccupancyGrid().width, agent[0].getOccupancyGrid().height);
        for (int j = 0; j < commonGrid.width; j++)
        {
            for (int k = 0; k < commonGrid.height; k++)
            {
                if (!agent[0].getOccupancyGrid().isKnownAtBase(j, k))
                {
                    commonGrid.setGotRelayed(j, k);
                    for (int i = 1; i < numRobots; i++)
                    {
                        boolean isRelayed = commonGrid.isGotRelayed(j, k);
                        commonGrid.setByte(j, k, (byte)(commonGrid.getByte(j,k) | agent[i].getOccupancyGrid().getByteNoRelay(j,k)));
                        if (isRelayed != commonGrid.isGotRelayed(j, k))
                            System.out.println("THIS SHOULD NEVER HAPPEN!");
                        if (!agent[i].getOccupancyGrid().isGotRelayed(j, k))
                            commonGrid.setGotUnrelayed(j, k);
                    }
                    //check that someone is relaying information known to agents but unknown at base
                    if (commonGrid.freeSpaceAt(j, k) && !commonGrid.isKnownAtBase(j, k) && commonGrid.isGotRelayed(j, k))
                        counter++;
                }
            }
        } 
        if (counter > 0)
        {
            int res = verifyNoInfoGotLost2();
            if (res > 0)
            {
                System.out.println("ERROR: INFORMATION LOST!! " + counter);
                System.out.println("@@@@@@@@@@@@@@  DIFF = " + res + "     @@@@@@@@@@@@@@@@@");
            }
        }
    }
    
    public int verifyNoInfoGotLost2()
    {
        //build common occgrid of all agents
        int counter_allagents = 0;
        int counter_base = 0;
        int newCell = 0;
        OccupancyGrid commonGrid = new OccupancyGrid(agent[0].getOccupancyGrid().width, agent[0].getOccupancyGrid().height);
        for (int j = 0; j < commonGrid.width; j++)
        {
            for (int k = 0; k < commonGrid.height; k++)
            {
                boolean newCellFound = false;
                commonGrid.setBit(j, k, OccupancyGrid.OccGridBit.FreeSpace, 0);
                for (int i = 0; i < numRobots; i++)
                {
                    if (agent[i].getOccupancyGrid().freeSpaceAt(j, k))
                    {
                        commonGrid.setFreeSpaceAt(j, k);                    
                        break;
                    }
                }
                
                for (int i = 1; i < numRobots; i++)
                {
                    if (agent[i].getOccupancyGrid().freeSpaceAt(j, k))
                    {
                        if ((!agent[i].getOccupancyGrid().isKnownAtBase(j, k)) && 
                                (!agent[i].getOccupancyGrid().isGotRelayed(j, k)))
                        {
                            newCell++;
                            newCellFound = true;
                            break;
                        }
                    }
                }
                
                //if ((commonGrid.freeSpaceAt(j, k)) &&
                //        !(agent[0].getOccupancyGrid().freeSpaceAt(j, k)) &&
                //        !newCellFound)
                //    System.out.println("~~~ CELL LOST: (" + j + ", " + k + ")");
                //    
                //check that someone is relaying information known to agents but unknown at base
                if (commonGrid.freeSpaceAt(j, k))
                    counter_allagents++;
                if (agent[0].getOccupancyGrid().freeSpaceAt(j, k))
                    counter_base++;
            }
        } 
        int result = (counter_allagents - counter_base - newCell);
        return result;        
    }*/
    // </editor-fold>
    
    private static int[][] detectMultiHopLinks(int commTable[][]) {
        for(int i=0; i<commTable.length; i++)
            for(int j=0; j<commTable[0].length; j++)
                if(commTable[i][j] == 1 || commTable[j][i] == 1)
                    for(int k=0; k<commTable.length; k++)
                        if(commTable[k][i] == 1 || commTable[i][k] == 1){
                            commTable[k][j] = 1;
                            commTable[j][k] = 1;
                        }
        
        return commTable;
    }
    
    private void detectCommunication() {
        directCommTable = new int[numRobots][numRobots];
        multihopCommTable = new int[numRobots][numRobots];
        
        switch(simConfig.getCommModel()) {
            case StaticCircle:          directCommTable = StaticCircle.detectCommunication(env, agent);
                                        break;
            case DirectLine:            directCommTable = DirectLine.detectCommunication(env, agent);
                                        break;
            case PropModel1:            directCommTable = PropModel1.detectCommunication(env, agent);
                                        for(int i=0; i<numRobots; i++)
                                            if(mainGUI.getRobotPanel(i).showCommRange())
                                                agentRange[i] = PropModel1.getRange(env, agent[i]);
                                        break;
            default:                    break;
        }
        
        multihopCommTable = detectMultiHopLinks(directCommTable);
    }

    // </editor-fold>     
   
// <editor-fold defaultstate="collapsed" desc="Role Switch">
    
    private void switchRoles(RealAgent agent1, RealAgent agent2, Path p1, Path p2) {
        //System.out.println(this.toString() + "Switching " + agent1.getName() + " and " + agent2.getName() + "... ");

        // create temp copy of each agent for teammate maintenance
        TeammateAgent copyAgent1 = agent2.getTeammate(agent1.getID()).copy();
        TeammateAgent copyAgent2 = agent1.getTeammate(agent2.getID()).copy();

        // switch ID
        int tempID = agent1.getID();
        agent1.setID(agent2.getID());
        agent2.setID(tempID);

        // reinit time since last plan 
        //agent1.setTimeSinceLastPlan(15);
        //agent2.setTimeSinceLastPlan(15);
        
        // reinit state timer
        //agent1.setStateTimer(15);
        //agent2.setStateTimer(15);
        
        // exchange frontiers
        PriorityQueue<Frontier> tempFrontiers = agent1.getFrontiers();
        agent1.setFrontiers(agent2.getFrontiers());
        agent2.setFrontiers(tempFrontiers);
        
        // exchange childRV
        RVLocation tempChildRV = agent1.getChildRendezvous();
        agent1.setChildRendezvous(agent2.getChildRendezvous());
        agent2.setChildRendezvous(tempChildRV);

        // exchange parentRV
        RVLocation tempParentRV = agent1.getParentRendezvous();
        agent1.setParentRendezvous(agent2.getParentRendezvous());
        agent2.setParentRendezvous(tempParentRV);

        // exchange exploreState
        ExploreState tempExploreState = agent1.getState();
        agent1.setState(agent2.getState());
        agent2.setState(tempExploreState);

        // exchange parent
        int tempParent = agent1.getParent();
        agent1.setParent(agent2.getParent());
        agent2.setParent(tempParent);

        // exchange child
        int tempChild = agent1.getChild();
        agent1.setChild(agent2.getChild());
        agent2.setChild(tempChild);

        // exchange role
        roletype tempRole = agent1.getRole();
        agent1.setRole(agent2.getRole());
        agent2.setRole(tempRole);
        
        // exchange current goal (important!)
        Point tempCurrGoal = agent1.getCurrentGoal();
        agent1.setCurrentGoal(agent2.getCurrentGoal());
        agent2.setCurrentGoal(tempCurrGoal);

        // set newly calculated path
        agent1.setPath(p1);
        agent1.addDirtyCells(p1.getAllPathPixels());
        agent2.setPath(p2);
        agent2.addDirtyCells(p2.getAllPathPixels());

        // exchange lastFrontier
        Frontier tempLastFrontier = agent1.getLastFrontier();
        agent1.setLastFrontier(agent2.getLastFrontier());
        agent2.setLastFrontier(tempLastFrontier);

        // replace Teammate agents
        agent1.removeTeammate(agent2.getID());
        agent1.addTeammate(copyAgent1);
        agent2.removeTeammate(agent1.getID());
        agent2.addTeammate(copyAgent2);
    }

    private boolean checkRoleSwitch(int first, int second) {
        RealAgent agent1 = agent[first];
        RealAgent agent2 = agent[second];
        
        if(agent1.getTimeSinceLastRoleSwitch() < 4 ||
           agent2.getTimeSinceLastRoleSwitch() < 4)
            return false;

        // Specific scenario which leads to oscillation must be avoided
        /*if((agent1.isExplorer() && agent1.getState() == ExploreState.Explore &&
           !agent2.isExplorer() && agent2.getState() == ExploreState.GoToChild) ||
           (agent2.isExplorer() && agent2.getState() == ExploreState.Explore &&
           !agent1.isExplorer() && agent1.getState() == ExploreState.GoToChild))
             return false;*/


       /* path.Path path1 = new path.Path(agent1.getOccupancyGrid(), agent1.getLocation(), agent1.getCurrentGoal());
        path.Path path2 = new path.Path(agent2.getOccupancyGrid(), agent2.getLocation(), agent2.getCurrentGoal());
        path.Path path3 = new path.Path(agent1.getOccupancyGrid(), agent1.getLocation(), agent2.getCurrentGoal());
        path.Path path4 = new path.Path(agent2.getOccupancyGrid(), agent2.getLocation(), agent1.getCurrentGoal());

        if(Math.max(path1.getLength(), path2.getLength()) > Math.max(path3.getLength(), path4.getLength())) {
            // Apply dynamic hierarchy rule, switch roles
            switchRoles(agent1, agent2);
            return true;
        }*/


        try{   
            Path path_a1g2 = agent1.calculatePath(agent1.getLocation(), agent2.getCurrentGoal());
            Path path_a2g1 = agent2.calculatePath(agent2.getLocation(), agent1.getCurrentGoal());
            double agent1_goal1 = agent1.getPath().recalcLength();
            double agent2_goal2 = agent2.getPath().recalcLength();
            double agent1_goal2 = path_a1g2.getLength();
            double agent2_goal1 = path_a2g1.getLength();


            //System.out.println(Constants.INDENT + "ROLE SWITCH DEBUG" +
            //                   "\n       " + agent1.getName() + " to goal1 = " + (int)agent1_goal1 +
            //                   "\n       " + agent2.getName() + " to goal2 = " + (int)agent2_goal2 +
            //                   "\n       " + agent1.getName() + " to goal2 = " + (int)agent1_goal2 +
            //                   "\n       " + agent2.getName() + " to goal1 = " + (int)agent2_goal1);

            // First check:  is it possible to eliminate the longest path?  If not, don't switch.
            if(Math.max(agent1_goal1, agent2_goal2) <=
               Math.max(agent1_goal2, agent2_goal1))
                    return false;

            // Second check (if desired): does the overall responsiveness improve?  If not, don't switch.
            if(simConfig.strictRoleSwitch()) {

                    // Case 1:  Two explorers both in state explore
                    if(agent1.isExplorer() && agent1.getState() == ExploreState.Explore &&
                       agent2.isExplorer() && agent2.getState() == ExploreState.Explore) {

                        Path rv1ToCS = agent1.calculatePath(agent1.getParentRendezvous().getParentLocation(), 
                                agent1.getTeammate(Constants.BASE_STATION_ID).getLocation());
                        Path rv2ToCS = agent2.calculatePath(agent2.getParentRendezvous().getParentLocation(), 
                                agent2.getTeammate(Constants.BASE_STATION_ID).getLocation());

                        Path a1ToRV2 = agent1.calculatePath(agent1.getLocation(), 
                                agent2.getParentRendezvous().getChildLocation());
                        Path a2ToRV1 = agent2.calculatePath(agent2.getLocation(), 
                                agent1.getParentRendezvous().getChildLocation());

                        double noRoleSwitch = Math.max(  agent1.getTimeUntilRendezvous()+rv1ToCS.getLength(),
                                                         agent2.getTimeUntilRendezvous()+rv2ToCS.getLength());
                        double yesRoleSwitch = Math.min( a1ToRV2.getLength() + rv2ToCS.getLength(),
                                                         a2ToRV1.getLength() + rv1ToCS.getLength());

                            //System.out.println("\n\n\n\n\n*********************************");
                            //System.out.println("     No role switch: " + noRoleSwitch);
                            //System.out.println("    Yes role switch: " + yesRoleSwitch);
                            //System.out.println("*********************************\n\n\n\n");

                        if(noRoleSwitch <= yesRoleSwitch) {
                            //System.out.println("\n\n*********************************");
                            //System.out.println("    SWITCH DENIED");
                            //System.out.println("*********************************\n");
                            return false;
                        }
                    }


            }

            // If we reach this point, we want to switch roles.
            switchRoles(agent1, agent2, path_a1g2, path_a2g1);
            agent1.setTimeSinceLastRoleSwitch(0);
            agent2.setTimeSinceLastRoleSwitch(0);
            return true;
        }
        catch(NullPointerException e) {
        }

        return false;
    }
    
    private void switchRoles() {
        // Quick check if role switching is desired
        if(!simConfig.roleSwitchAllowed())
            return;
        
        long realtimeStart = System.currentTimeMillis();
        //System.out.println(this.toString() + "Checking for role switch ... ");

        // Note, need timeout on number of swaps a robot can do (at least 3 timestep gap?)
        if(timeElapsed > 5 && timeElapsed % 10 == 0)
            for(int i=1; i<numRobots-1; i++)
                for(int j=i+1; j<numRobots; j++)
                    if(agent[i].getTeammate(agent[j].getID()).isInRange()) {
                            // Only one role switch per time step allowed at the moment
                            if(checkRoleSwitch(i, j)) {
                                // exit loops
                                numSwaps++;
                                j=numRobots;
                                i=numRobots;
                            }
                        }
                
        //System.out.println(this.toString() + "Role switch check complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

// </editor-fold>     
  
// <editor-fold defaultstate="collapsed" desc="Logging">

    private void logging() {
        // Note, logging of data is performed in updateGlobalData, should change to here when i have the time

        // Log screenshot
        if(simConfig.logScreenshots())
            logScreenshot();
        
        if (simConfig.getExpAlgorithm() == exptype.RunFromLog)
            return; //Nothing to do here
        
        // Log agent positions
        if(simConfig.logAgents())
            logAgents();

        
    }

    private void logAgents() {        
        try{
            PrintWriter outFile = new PrintWriter(new FileWriter(simConfig.getLogAgentFilename(), true));

            outFile.print(timeElapsed + " ");
            for(int i=0; i<numRobots; i++) {
                outFile.print(agent[i].getX() + " ");
                outFile.print(agent[i].getY() + " ");
                outFile.print(agent[i].getCurrentGoal().getX() + " ");
                outFile.print(agent[i].getCurrentGoal().getY() + " ");
                outFile.print(agent[i].getRole() + " ");
                outFile.print(agent[i].getState() + " ");                
            }
            outFile.println();
            outFile.close();
        }
        catch(IOException e){
            System.out.println(this.toString() + "Agent logging - error writing data to file!" + e);
        }
    }

    private void logScreenshot() {
        image.fullUpdate(mainGUI.getShowSettings(), mainGUI.getShowSettingsAgents(), env, agent, agentRange);
        image.saveScreenshot(simConfig.getLogScreenshotsDirname(), timeElapsed);
    }
    
// </editor-fold>     
    
// <editor-fold defaultstate="collapsed" desc="GUI Interaction">
    
    public void simRateChanged(int newSimRate, MainGUI.runMode RUNMODE) {
        if(newSimRate == 0)
            timer.stop();
        else {
            if(!timer.isRunning() && !RUNMODE.equals(MainGUI.runMode.paused))
                timer.start();
            timer.setDelay((Constants.TIME_INCREMENT*10+1) - newSimRate*Constants.TIME_INCREMENT);
            //System.out.println(this.toString() + "Timer set to " + ((Constants.TIME_INCREMENT*10+1) - newSimRate*Constants.TIME_INCREMENT));
        }
    }
    
    private void updateGUI() {
        //long realtimeStart = System.currentTimeMillis();
        //System.out.print(this.toString() + "Updating GUI ... ");
        
        mainGUI.updateFromData(agent, timeElapsed, pctAreaKnown, avgCycleTime);
        updateImage(false); //was false
        
        //System.out.println(this.toString() + "GUI Update complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }

    public void updateImage(boolean full)
    {
        long realtimeStart = System.currentTimeMillis();
        
        if(full){
            //System.out.print(this.toString() + "Full Image Update ... ");
            image.fullUpdate(mainGUI.getShowSettings(), mainGUI.getShowSettingsAgents(), env, agent, agentRange);
        }
        else{
            //System.out.print(this.toString() + "Dirty Image Update ... ");
            image.dirtyUpdate(mainGUI.getShowSettings(), mainGUI.getShowSettingsAgents(), env, agent, agentRange);
        }
        mainGUI.getLabelImageHolder().setIcon(new ImageIcon(image.getImage()));

        //System.out.println("Complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
    
    public int getTrueJointAreaKnown()
    {
        int known = 0;
        for(int j=0; j<env.getColumns(); j++)
            for(int k=0; k<env.getRows(); k++)
                for(int i=0; i<agent.length; i++) 
                    if ((agent[i].getOccupancyGrid().freeSpaceAt(j,k))
                            && (env.statusAt(j, k) != Status.obstacle)){
                        known++; //"true" area known, excluding false empty spaces
                        i = agent.length;
                    }
        return known;
    }
    
    private void updateGlobalData() {
        long realtimeStart = System.currentTimeMillis();
        //System.out.print(this.toString() + "Updating Global Data ... ");
        timeElapsed++;
        pctAreaKnown = 100 * (double)agent[Constants.BASE_STATION_ID].getAreaKnown() / (double)totalArea;
        if(simConfig.logData()) {
            avgAgentKnowledge = 0;
            avgTimeLastCommand = 0;
            totalDistanceTraveled = 0;
            //jointAreaKnown = 1;

            if ((timeElapsed % Constants.RECALC_JOINT_AREA) == 1)
                jointAreaKnown = getTrueJointAreaKnown();
            else jointAreaKnown = Math.max(agent[Constants.BASE_STATION_ID].getAreaKnown(), jointAreaKnown);
            
            jointAreaKnown = Math.max(agent[Constants.BASE_STATION_ID].getAreaKnown(), jointAreaKnown);

            for(int i=1; i<agent.length; i++) {
                avgAgentKnowledge += agent[i].getAreaKnown();
                avgTimeLastCommand += agent[i].getTimeLastCentralCommand();
                totalDistanceTraveled += agent[i].getDistanceTraveled();
            }
            avgAgentKnowledge /= (agent.length-1);  //ComStation not included in calculation
            avgAgentKnowledge = 100 * avgAgentKnowledge / jointAreaKnown;
            avgTimeLastCommand /= (agent.length-1);

            if(jointAreaKnown == 0)
                avgComStationKnowledge = 0;
            else
                if(timeElapsed < 2)
                    avgComStationKnowledge = 100 * agent[0].getAreaKnown() / jointAreaKnown;
                else
                    avgComStationKnowledge = ((timeElapsed-1)*avgComStationKnowledge + 
                                                    (100 * agent[0].getAreaKnown() / jointAreaKnown)) / 
                                                   timeElapsed;
            pctAreaKnown = 100 * (double)jointAreaKnown / (double)totalArea;


            try{
                PrintWriter outFile = new PrintWriter(new FileWriter(simConfig.getLogDataFilename(), true));

                outFile.print(timeElapsed + " ");
                outFile.print(System.currentTimeMillis() + " ");
                outFile.print(pctAreaKnown + " ");
                outFile.print(100*(double)agent[0].getAreaKnown()/(double)totalArea + " ");
                outFile.print(avgAgentKnowledge + " ");
                outFile.print(avgTimeLastCommand + " ");
                outFile.print((double)totalDistanceTraveled / (double)(agent.length-1) + " ");                
                outFile.print(numSwaps + " ");
                outFile.print(jointAreaKnown + " ");
                outFile.print(agent[0].getAreaKnown() + " ");
                outFile.print(100 * (double)agent[0].getAreaKnown() / (double)jointAreaKnown + " ");
                for (int i = 1; i < agent.length; i++)
                {
                    outFile.print(agent[i].getAreaKnown() + " ");
                    outFile.print(agent[i].getNewInfo() + " ");
                    outFile.print(agent[i].getMaxRateOfInfoGatheringBelief() + " ");
                    outFile.print(agent[i].getCurrentTotalKnowledgeBelief() + " ");
                    outFile.print(agent[i].getCurrentBaseKnowledgeBelief() + " ");
                }
                outFile.println();
                outFile.close();
            }
            catch(IOException e){
                System.out.println(this.toString() + "Error writing data to file!" + e);
            }
        }        
        //System.out.println("Complete, took " + (System.currentTimeMillis()-realtimeStart) + "ms.");
    }
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Utility">
    
    @Override
    public String toString() {
        return("[Simulator] ");
    }
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Debris">

    private void simulateDebris() {
        int debrisSize, currX, currY, nextX, nextY;

        /* The below puts random debris anywhere
        // according to constant NEW_DEBRIS_LIKELIHOOD, add debris
        if(random.nextInt(100) < (int)(Constants.NEW_DEBRIS_LIKELIHOOD * 100)) {
            debrisSize = random.nextInt(Constants.NEW_DEBRIS_MAX_SIZE) + 1;

            System.out.println(this.toString() + "Adding random debris of size " + debrisSize + "!");
            
            currX = random.nextInt(env.getColumns());
            currY = random.nextInt(env.getRows());
            
            for(int i=0; i<debrisSize; i++) {
                env.setStatus(currY, currX, Status.obstacle);
                do {
                    nextX = currX + random.nextInt(3) - 1;
                    nextY = currY + random.nextInt(3) - 1;
                }
                while(!env.locationExists(nextX, nextY));
                currX = nextX;
                currY = nextY;
            }
        } */

        /* The below is purely for the aisleRoom environment */
        // Gate 1
        /*if(debrisTimer[0] <= 0) {
            if(random.nextInt(100) < 5) {
                closeGate(46);
                debrisTimer[0] = 10;
            }
        }
        else if(debrisTimer[0] == 1) {
            openGate(46);
            debrisTimer[0] = 0;
        }
        else
            debrisTimer[0]--;

        if(debrisTimer[1] <= 0) {
            if(random.nextInt(100) < 5) {
                closeGate(121);
                debrisTimer[0] = 10;
            }
        }
        else if(debrisTimer[1] == 1) {
            openGate(121);
            debrisTimer[1] = 0;
        }
        else
            debrisTimer[1]--;

        if(debrisTimer[2] <= 0) {
            if(random.nextInt(100) < 5) {
                closeGate(196);
                debrisTimer[0] = 10;
            }
        }
        else if(debrisTimer[2] == 1) {
            openGate(196);
            debrisTimer[2] = 0;
        }
        else
            debrisTimer[2]--;*/


    }

    private void closeGate(int yTop) {
        for(int i=yTop; i<yTop+67; i++)
            for(int j=250; j<258; j++)
                env.setStatus(j, i, Status.obstacle);
    }

    private void openGate(int yTop) {
        for(int i=yTop; i<yTop+67; i++)
            for(int j=250; j<258; j++)
                env.setStatus(j, i, Status.unexplored);
    }

    
// </editor-fold>   
   
}
