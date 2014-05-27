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
package config;

/**
 *
 * @author Julian de Hoog
 */
public class RobotConfig {
    
    // start x, y and heading count as a single field, comma delimited.
    public static int NUMROBOTCONFIGFIELDS = 9;
    
    private int robotNumber;
    private String name;
    private int startX;
    private int startY;
    private double startHeading;
    private int sensingRange;
    private int commRange;
    private int batteryLife;
        
    public static enum roletype {BaseStation, Relay, Explorer}
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
    }

    public RobotConfig(int newRobotNo, String newName, int newStartX, int newStartY, double newStartHeading, int newSensingRange, int newCommRange, int newBatteryLife, String newRole, int newParent, int newChild) {
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
    }
    
    public RobotConfig(String newRobotNumber, String newName, String start, String newSensingRange, String newCommRange, String newBatteryLife, String newRole, String newParent, String newChild) {
        robotNumber = Integer.parseInt(newRobotNumber);
        name = newName;
        
        String tokens[] = start.split(",");

        if(tokens.length != 3) {
            System.out.println("Error: incorrect number of data for start location of robot " + newName);
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
        return (String.valueOf(robotNumber) + " " +
                name + " " +
                String.valueOf(startX) + " " +
                String.valueOf(startY) + " " +
                String.valueOf(startHeading) + " " +
                String.valueOf(sensingRange) + " " +
                String.valueOf(commRange) + " " +
                String.valueOf(batteryLife) + " " +
                role + " " +
                String.valueOf(parent) + " " +
                String.valueOf(child));

    }
}
