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
package agents;

import config.Constants;
import config.RobotConfig;
import config.RobotConfig.roletype;
import environment.OccupancyGrid;
import exploration.rendezvous.Rendezvous;
import exploration.rendezvous.RendezvousAgentData;
import java.awt.Point;

/**
 * The TeammateAgent class is used to store the knowledge of an agent about their teammates.
 * Fields in this class represent what the agent can know about their teammates.
 * @author julh
 */
public class TeammateAgent extends BasicAgent implements Agent {
    
    int timeLastCentralCommand;   /* units of time elapsed since command 
                                     received from ComStation */
    int lastContactAreaKnown;
    private boolean inRange;
    private boolean inDirectRange;

    private int timeSinceLastComm;
    OccupancyGrid occGrid;
    private Rendezvous childRendezvous;
    private Rendezvous parentRendezvous;
    private Point frontierCentre;
    private double pathLength;
    int relayID;
    public int newInfo;
    
    private RendezvousAgentData rendezvousAgentData;

    public TeammateAgent(RobotConfig robot) {
        super(robot.getRobotNumber(), 
              robot.getName(), 
              robot.getRobotNumber(), 
              robot.getStartX(), 
              robot.getStartY(), 
              robot.getStartHeading(),
              robot.getSensingRange(), 
              robot.getCommRange(), 
              robot.getBatteryLife(), 
              robot.getRole(),
              robot.getParent(),
              robot.getChild(),
              Constants.STEP_SIZE);
        
        inRange = false;
        timeSinceLastComm = 0;
        pathLength = 0;
        rendezvousAgentData = new RendezvousAgentData(this);
    }
    
    public TeammateAgent(TeammateAgent toCopy) {
        super(toCopy.getRobotNumber(), 
              toCopy.getName(), 
              toCopy.getID(),
              toCopy.getX(), 
              toCopy.getY(), 
              toCopy.getHeading(),
              toCopy.getSenseRange(),
              toCopy.getCommRange(), 
              toCopy.getBatteryPower(), 
              toCopy.getRole(),
              toCopy.getParent(),
              toCopy.getChild(),
              toCopy.getSpeed());
        this.childRendezvous = toCopy.childRendezvous;
        this.parentRendezvous = toCopy.parentRendezvous;
        this.timeLastCentralCommand = toCopy.timeLastCentralCommand;
        this.lastContactAreaKnown = toCopy.lastContactAreaKnown;
        this.rendezvousAgentData = new RendezvousAgentData(toCopy.rendezvousAgentData);

    }

    public int getTimeLastCentralCommand() {
        return this.timeLastCentralCommand;
    }
    
    public void setTimeLastCentralCommand(int t) {
        this.timeLastCentralCommand = t;
    }
    
    public int getTimeSinceLastComm() {
        return this.timeSinceLastComm;
    }
    
    public void setTimeSinceLastComm(int t) {
        this.timeSinceLastComm = t;
    }
    
    public boolean isInDirectRange() {
        return inDirectRange;
    }

    public void setInDirectRange(boolean r) {
        inDirectRange = r;
    }

    public boolean isInRange() {
        if (inRange) {
            //System.out.println("I am " + this.getName() + "[" + this.getID() + "], in range with my teammate.");
        }
        return inRange;
    }

    public void setInRange(boolean r) {
        inRange = r;
    }

    public OccupancyGrid getOccupancyGrid() {
        return occGrid;
    }
    
    public void setOccupancyGrid(OccupancyGrid og) {
        this.occGrid = og;
    }
    
    public TeammateAgent copy() {
        return new TeammateAgent(this);
    }
    
    public boolean isExplorer() {
        return role.equals(roletype.Explorer);
    }
    
    public void setExplorer() {
        role = roletype.Explorer;
    }
    
    public void setRelay() {
        role = roletype.Relay;
    }
    
    public Point getFrontierCentre() {
        return frontierCentre;
    }
    
    public void setFrontierCentre(Point frontierCentre) {
        this.frontierCentre = frontierCentre;
    }
    
    public double getPathLength() {
        return pathLength;
    }
    
    public void setPathLength(double lgth) {
        pathLength = lgth;
    }
    
    public RendezvousAgentData getRendezvousAgentData() {
        return rendezvousAgentData;
    }
    
    public void setLastContactAreaKnown(int lastContactArea) {
        lastContactAreaKnown = lastContactArea;
    }
    
    public int getLastContactAreaKnown() {
        return lastContactAreaKnown;
    }
    
    public void setRelayID(int relayID) {
        this.relayID = relayID;
    }
    
    public int getID() {
        return ID;
    }
}
