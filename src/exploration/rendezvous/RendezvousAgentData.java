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
package exploration.rendezvous;

import agents.BasicAgent;
import config.Constants;

/**
 *
 * @author Victor
 */
public class RendezvousAgentData {

    private int timeSinceLastRVCalc;  // keeps track of time since last rendezvous calculation
    private Rendezvous parentRendezvous;  // location of parent rendezvous
    private Rendezvous parentBackupRendezvous; // location of parent backup rendezvous
    private Rendezvous childRendezvous;   // location of child rendezvous
    private Rendezvous childBackupRendezvous;   // location of child backup rendezvous
    private int timeUntilRendezvous; // estimated time left until due to rendezvous
    private int timeSinceLastRoleSwitch;  // keeps track of time since last switch

    public RendezvousAgentData(BasicAgent agent) {
        childRendezvous = new Rendezvous(agent.getLocation());
        childBackupRendezvous = new Rendezvous(agent.getLocation());
        parentRendezvous = new Rendezvous(agent.getLocation());
        parentBackupRendezvous = new Rendezvous(agent.getLocation());
        timeUntilRendezvous = 0;
        timeSinceLastRVCalc = Constants.MAX_TIME;
        timeSinceLastRoleSwitch = 0;
    }

    public RendezvousAgentData(RendezvousAgentData toCopy) {
        this.timeSinceLastRVCalc = toCopy.timeSinceLastRVCalc;
        this.timeUntilRendezvous = toCopy.timeUntilRendezvous;
        this.timeSinceLastRoleSwitch = toCopy.timeSinceLastRoleSwitch;
        this.parentRendezvous = toCopy.parentRendezvous.copy();
        this.parentBackupRendezvous = toCopy.parentBackupRendezvous.copy();
        this.childRendezvous = toCopy.childRendezvous.copy();
        this.childBackupRendezvous = toCopy.childBackupRendezvous.copy();
    }

    @Override
    public String toString() {
        return "pR: " + parentRendezvous + ", cR: " + childRendezvous
                + ", pBR: " + parentBackupRendezvous + ", cBR: " + childBackupRendezvous
                + ", tUR: " + timeUntilRendezvous + ", tSLRS: " + timeSinceLastRoleSwitch
                + ", tSLRVC: " + timeSinceLastRVCalc;
    }

    //<editor-fold defaultstate="collapsed" desc="Getters and setters">
    public int getTimeUntilRendezvous() {
        return timeUntilRendezvous;
    }

    public void setTimeUntilRendezvous(int n) {
        timeUntilRendezvous = n;
    }

    public int getTimeSinceLastRVCalc() {
        return timeSinceLastRVCalc;
    }

    public void setTimeSinceLastRVCalc(int t) {
        timeSinceLastRVCalc = t;
    }

    public int getTimeSinceLastRoleSwitch() {
        return timeSinceLastRoleSwitch;
    }

    public void setTimeSinceLastRoleSwitch(int t) {
        timeSinceLastRoleSwitch = t;
    }

    public Rendezvous getChildRendezvous() {
        return childRendezvous;
    }

    public void setChildRendezvous(Rendezvous r) {
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("Setting child RV to " + r);
        }
        childRendezvous = r;
    }

    public Rendezvous getChildBackupRendezvous() {
        return childBackupRendezvous;
    }

    public void setChildBackupRendezvous(Rendezvous r) {
        if (r != null) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Setting child backupRV to " + r);
            }
            childBackupRendezvous = r.copy();
        } else {
            childBackupRendezvous = null;
        }
    }

    public Rendezvous getParentRendezvous() {
        return parentRendezvous;
    }

    public void setParentRendezvous(Rendezvous r) {
        if (Constants.DEBUG_OUTPUT) {
            System.out.println("Setting parent RV to " + r);
        }
        parentRendezvous = r;
    }

    public Rendezvous getParentBackupRendezvous() {
        return parentBackupRendezvous;
    }

    public void setParentBackupRendezvous(Rendezvous r) {
        if (r != null) {
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Setting parent backupRV to " + r);
            }
            parentBackupRendezvous = r.copy();
        } else {
            parentBackupRendezvous = null;
        }
    }
//</editor-fold>

}
