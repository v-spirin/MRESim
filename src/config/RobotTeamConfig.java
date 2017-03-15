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
package config;

import gui.RobotConfigTableModel;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.HashMap;

/**
 *
 * @author Julian de Hoog
 */
public class RobotTeamConfig {

// <editor-fold defaultstate="collapsed" desc="Variables and Constructor">
    private int numRobots;
    private HashMap<Integer, RobotConfig> robotTeam;

    public RobotTeamConfig() {
        boolean oldConfigFound = loadOldConfig();
        if (!oldConfigFound) {
            numRobots = 1;
            robotTeam = new HashMap<Integer, RobotConfig>();
            RobotConfig comStation = new RobotConfig("1", "ComStation", "1,1,0", "0", "100", "1000", "BaseStation", "1", "1", "2", "1", "10");
            robotTeam.put(comStation.getRobotNumber(), comStation);
        }
    }
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Set and Get">
    public int getNumRobots() {
        return this.numRobots;
    }

    public HashMap<Integer, RobotConfig> getRobotTeam() {
        return this.robotTeam;
    }

    public String[] getAllRoles() {
        String[] roles = new String[4];
        roles[0] = RobotConfig.roletype.BaseStation.name();
        roles[1] = RobotConfig.roletype.Explorer.name();
        roles[2] = RobotConfig.roletype.Relay.name();
        roles[3] = RobotConfig.roletype.RelayStation.name();
        return roles;
    }

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="GUI interaction">
    public void updateFromGUI(RobotConfigTableModel model) {
        //erase old data
        robotTeam = new HashMap();

        numRobots = model.getRowCount();
        RobotConfig currRobot;
        for (int i = 0; i < numRobots; i++) {
            currRobot = new RobotConfig((String) (model.getValueAt(i, 0)),
                    (String) (model.getValueAt(i, 1)),
                    (String) (model.getValueAt(i, 2)),
                    (String) (model.getValueAt(i, 3)),
                    (String) (model.getValueAt(i, 4)),
                    (String) (model.getValueAt(i, 5)),
                    (String) (model.getValueAt(i, 6)),
                    (String) (model.getValueAt(i, 7)),
                    (String) (model.getValueAt(i, 8)),
                    (String) (model.getValueAt(i, 9)),
                    (String) (model.getValueAt(i, 10)),
                    (String) (model.getValueAt(i, 11)));
            robotTeam.put(currRobot.getRobotNumber(), currRobot);
        }
    }
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Load and Save">
    public boolean saveConfig() {
        //default save file
        return this.saveConfig(System.getProperty("user.dir") + "/config/lastTeamConfig.txt");
    }

    public boolean saveConfig(String fileName) {

        try (PrintWriter outFile = new PrintWriter(new FileWriter(fileName))) {
            RobotConfig currRobot;

            for (int i = 1; i <= numRobots; i++) {
                currRobot = (RobotConfig) (robotTeam.get(i));
                //Debug: System.out.println(currRobot.toString());
                outFile.println(currRobot.toString());
            }

            return true;
        } catch (IOException e) {
            System.err.println("Error writing to file " + fileName);
        }

        return false;
    }

    private boolean loadOldConfig() {
        String oldConfigFilename = System.getProperty("user.dir") + "/config/lastTeamConfig.txt";
        File file = new File(oldConfigFilename);
        if (file.exists()) {
            return loadConfig(oldConfigFilename);
        } else {
            return false;
        }
    }

    public boolean loadConfig(String fileName) {
        File file = new File(fileName);
        int lineNum = 1;

        if (file.exists()) {
            try (BufferedReader inFile = new BufferedReader(new FileReader(file))) {
                String line = inFile.readLine();

                // Since a new team will be loaded, must delete old
                this.robotTeam = new HashMap();
                this.numRobots = 0;

                while (line != null) {
                    processLine(line, lineNum, fileName);
                    lineNum++;
                    line = inFile.readLine();
                }

                return true;
            } catch (IOException e) {
                System.err.println("Error: could not read data from " + fileName);
            }
        }
        return false;
    }

    public void processLine(String inputLine, int lineNum, String fileName) {
        String tokens[] = inputLine.split("\\s");

        if (tokens.length != (RobotConfig.NUMROBOTCONFIGFIELDS + 2)) {
            System.err.println("Error: incorrect number of data for robot on line " + lineNum + " in file " + fileName);
            return;
        }

        // Could definitely do with some improved error checking here
        try {
            RobotConfig currRobot = new RobotConfig(Integer.parseInt(tokens[0]),
                    tokens[1],
                    Integer.parseInt(tokens[2]),
                    Integer.parseInt(tokens[3]),
                    Double.parseDouble(tokens[4]),
                    Integer.parseInt(tokens[5]),
                    Integer.parseInt(tokens[6]),
                    Integer.parseInt(tokens[7]),
                    tokens[8],
                    Integer.parseInt(tokens[9]),
                    Integer.parseInt(tokens[10]),
                    Integer.parseInt(tokens[11]),
                    Integer.parseInt(tokens[12]),
                    Integer.parseInt(tokens[13])
            );
            numRobots++;
            robotTeam.put(currRobot.getRobotNumber(), currRobot);
        } catch (NumberFormatException | ArrayIndexOutOfBoundsException nfe) {
            System.err.println("Error: Could not understand input data for robot on line " + lineNum + " in file " + fileName);
        }
    }

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Other">
    public void printDetails() {
        RobotConfig currRobot;
        System.out.println("-------------------------------------------------------------");
        for (int i = 1; i <= numRobots; i++) {
            currRobot = (RobotConfig) (robotTeam.get(i));
            System.out.print("Robot " + currRobot.getRobotNumber() + ": ");
            System.out.print(currRobot.getName() + " "
                    + currRobot.getStartX() + " "
                    + currRobot.getStartY() + " "
                    + currRobot.getStartHeading() + " "
                    + currRobot.getSensingRange() + " "
                    + currRobot.getCommRange() + " "
                    + currRobot.getBatteryLife() + " "
                    + currRobot.getRole() + " "
                    + currRobot.getParent() + " "
                    + currRobot.getChild() + " "
                    + currRobot.getAbility() + " "
                    + currRobot.getComStationLimit() + " "
                    + currRobot.getSpeed() + "\n");
        }
        System.out.println("-------------------------------------------------------------");
    }
    // </editor-fold>
}
