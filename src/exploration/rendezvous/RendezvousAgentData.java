/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package exploration.rendezvous;

import agents.BasicAgent;

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
        timeSinceLastRVCalc = 0;
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
        childRendezvous = r;
    }
    
    public Rendezvous getChildBackupRendezvous() {
        return childBackupRendezvous;
    }
    
    public void setChildBackupRendezvous(Rendezvous r) {
        childBackupRendezvous = r.copy();
    }
    
    public Rendezvous getParentRendezvous() {
        return parentRendezvous;
    }
    
    public void setParentRendezvous(Rendezvous r) {
        parentRendezvous = r;
    }
    
    public Rendezvous getParentBackupRendezvous() {
        return parentBackupRendezvous;
    }
    
    public void setParentBackupRendezvous(Rendezvous r) {
        parentBackupRendezvous = r.copy();
    }
//</editor-fold>
 
}
