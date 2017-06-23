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

/**
 *
 * @author Julian de Hoog
 */
public class RobotConfig {

    // start x, y and heading count as a single field, comma delimited.
    public static int NUMROBOTCONFIGFIELDS = 13;

    private int robotNumber;
    private String name;
    private int startX;
    private int startY;
    private double startHeading;
    private int sensingRange;
    private int commRange;
    private int batteryLife;
    private boolean loggingState;
    private int ability;
    private int comStationLimit;
    private int speed;
    private int energyConsumption;

    public static enum roletype {
        BaseStation, Relay, Explorer, RelayStation
    }
    private roletype role;

    private int parent;
    private int child;

    public RobotConfig() {
        robotNumber = 1;
        name = "Noname";
        startX = 1;
        startY = 1;
        startHeading = 0;
        sensingRange = 0;
        commRange = 0;
        batteryLife = 0;
        role = roletype.Relay;
        parent = 1;
        child = 1;
        ability = 2;
        comStationLimit = 1;
        speed = 10;
        energyConsumption = 1;
    }

    public RobotConfig(int newRobotNo, String newName, int newStartX, int newStartY, double newStartHeading, int newSensingRange, int newCommRange, int newBatteryLife, String newRole, int newParent, int newChild, int newAbility, int newComStationLimit, int newSpeed, int newEnergyConsumption) {
        robotNumber = newRobotNo;
        name = newName;
        startX = newStartX;
        startY = newStartY;
        startHeading = newStartHeading;
        sensingRange = newSensingRange;
        commRange = newCommRange;
        batteryLife = newBatteryLife;
        role = roletype.valueOf(newRole);
        parent = newParent;
        child = newChild;
        loggingState = false;
        ability = newAbility;
        comStationLimit = newComStationLimit;
        speed = newSpeed;
        energyConsumption = newEnergyConsumption;
    }

    public RobotConfig(String newRobotNumber, String newName, String start, String newSensingRange, String newCommRange, String newBatteryLife, String newRole, String newParent, String newChild, String newAbility, String newComStationLimit, String newSpeed, String newEnergyConsumption) {
        robotNumber = Integer.parseInt(newRobotNumber);
        name = newName;

        String tokens[] = start.split(",");

        if (tokens.length != 3) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Error: incorrect number of data for start location of robot " + newName);
            }
            return;
        }

        startX = Integer.parseInt(tokens[0]);
        startY = Integer.parseInt(tokens[1]);
        startHeading = Double.parseDouble(tokens[2]);
        sensingRange = Integer.parseInt(newSensingRange);
        commRange = Integer.parseInt(newCommRange);
        batteryLife = Integer.parseInt(newBatteryLife);
        role = roletype.valueOf(newRole);
        parent = Integer.parseInt(newParent);
        child = Integer.parseInt(newChild);
        ability = Integer.parseInt(newAbility);
        comStationLimit = Integer.parseInt(newComStationLimit);
        speed = Integer.parseInt(newSpeed);
        energyConsumption = Integer.parseInt(newEnergyConsumption);
    }

    public int getSpeed() {
        return speed;
    }

    public void setSpeed(int speed) {
        this.speed = speed;
    }

    public int getComStationLimit() {
        return comStationLimit;
    }

    public void setComStationLimit(int comStationLimit) {
        this.comStationLimit = comStationLimit;
    }

    public int getAbility() {
        return ability;
    }

    public void setAbility(int ability) {
        this.ability = ability;
    }

    public int getEnergyConsumption() {
        return energyConsumption;
    }

    public void setEnergyConsumption(int energyConsumption) {
        this.energyConsumption = energyConsumption;
    }

    public void setLoggingState(boolean loggingState) {
        this.loggingState = loggingState;
    }

    public boolean getLoggingState() {
        return this.loggingState;
    }

    public int getRobotNumber() {
        return this.robotNumber;
    }

    public String getName() {
        return this.name;
    }

    public int getStartX() {
        return this.startX;
    }

    public int getStartY() {
        return this.startY;
    }

    public double getStartHeading() {
        return this.startHeading;
    }

    public int getSensingRange() {
        return this.sensingRange;
    }

    public int getCommRange() {
        return this.commRange;
    }

    public int getBatteryLife() {
        return this.batteryLife;
    }

    public roletype getRole() {
        return role;
    }

    public int getParent() {
        return parent;
    }

    public int getChild() {
        return child;
    }

    @Override
    public String toString() {
        return (String.valueOf(robotNumber) + " "
                + name + " "
                + String.valueOf(startX) + " "
                + String.valueOf(startY) + " "
                + String.valueOf(startHeading) + " "
                + String.valueOf(sensingRange) + " "
                + String.valueOf(commRange) + " "
                + String.valueOf(batteryLife) + " "
                + role + " "
                + String.valueOf(parent) + " "
                + String.valueOf(child) + " "
                + String.valueOf(ability) + " "
                + String.valueOf(comStationLimit) + " "
                + String.valueOf(speed) + " "
                + String.valueOf(speed));

    }
}
