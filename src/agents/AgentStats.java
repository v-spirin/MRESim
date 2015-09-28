/* Copyright 2010, 2015 Julian de Hoog (julian@dehoog.ca), Victor Spirin (victor.spirin@cs.ox.ac.uk)

    This file is part of MRESim 2.2, a simulator for testing the behaviour
    of multiple robots exploring unknown environments.

    If you use MRESim, I would appreciate an acknowledgement and/or a citation
    of our papers:

    @inproceedings{deHoog2009,
        title = "Role-Based Autonomous Multi-Robot Exploration",
        author = "Julian de Hoog, Stephen Cameron and Arnoud Visser",
        year = "2009",
        booktitle = "International Conference on Advanced Cognitive Technologies and Applications (COGNITIVE)",
        location = "Athens, Greece",
        month = "November",
    }

    @incollection{spirin2015mresim,
      title={MRESim, a Multi-robot Exploration Simulator for the Rescue Simulation League},
      author={Spirin, Victor and de Hoog, Julian and Visser, Arnoud and Cameron, Stephen},
      booktitle={RoboCup 2014: Robot World Cup XVIII},
      pages={106--117},
      year={2015},
      publisher={Springer}
    }

    MRESim is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MRESim is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along with MRESim.
    If not, see <http://www.gnu.org/licenses/>.
*/

package agents;

import java.util.LinkedList;

/**
 *
 * @author Victor
 */


public class AgentStats {
    //stores the stats on agent's progress; this is mostly used
    //for logging / to observe agent's behaviour
    
    private int timeLastCentralCommand;   /* units of time elapsed since command 
                                     received from ComStation */
    private double distanceTraveled;      // Distance traveled
    private int areaKnown;                // How much area this agent knows about
    private int areaGoal;                 // How much free space the agent needs to explore in this mission
    private int lastContactAreaKnown;     // How much area this agent knew at the time of last contact with base
    private int newInfo;                  // How much area that we know we think is not known at base station
    private int timeLastDirectContactCS; // time since last contact with base station
    private double maxRateOfInfoGatheringBelief;
    private double currentTotalKnowledgeBelief;    
    private double currentBaseKnowledgeBelief;
    private double percentageKnown;
    private int timeSinceLastPlan;        // time passed since last plan was made
    
    private int timeSensing;
    private int lastIncrementedTimeSensing;
    private int timeReturning;
    private int lastIncrementedTimeReturning;
    private int timeDoubleSensing;
    private int lastIncrementedTimeDoubleSensing;
    private final LinkedList<Integer> timeBaseMessageReceived;
    
    public AgentStats() {
        timeLastCentralCommand = 0;
        distanceTraveled = 0;
        areaKnown = 0;
        newInfo = 0;
        lastContactAreaKnown = 0;
        timeLastDirectContactCS = 0;
        maxRateOfInfoGatheringBelief = 0;
        currentTotalKnowledgeBelief = 0;
        currentBaseKnowledgeBelief = 0;
        timeSinceLastPlan = 0;
        
        lastIncrementedTimeSensing = 0;
        lastIncrementedTimeReturning = 0;
        lastIncrementedTimeDoubleSensing = 0;
        
        timeSensing = 0;
        timeReturning = 0;
        timeDoubleSensing = 0;
        timeBaseMessageReceived = new LinkedList<Integer>();
    }
    
    public int getTimeLastCentralCommand() {
        return this.timeLastCentralCommand;
    }
    
    public void incrementTimeLastCentralCommand() {
        timeLastCentralCommand++;
    }
    
    public double getDistanceTraveled() {
        return this.distanceTraveled;
    }
    
    public double getPercentageKnown() {
        return percentageKnown;
    }
    
    public void setPercentageKnown(double pct) {
        percentageKnown = pct;
    }
    
    public void setGoalArea(int goalArea) {
        areaGoal = goalArea;
    }
    
    public int getGoalArea() {
        return areaGoal;
    }
    
    public void addDistanceTraveled(double newDistance) {
        distanceTraveled += newDistance;
    }
    
    public int getAreaKnown() {
        return this.areaKnown;
    }
    
    public void setAreaKnown(int area) {
        areaKnown = area;
    }
    
    public int getNewInfo() {
        return this.newInfo;
    }
    
    public void setNewInfo(int info) {
        newInfo = info;
    }
    
    public int getTimeLastDirectContactCS() {
        return timeLastDirectContactCS;
    }
    
    public void setTimeLastDirectContactCS(int val) {
        timeLastDirectContactCS = val;
    }
    
    public void incrementLastDirectContactCS() {
        timeLastDirectContactCS++;
    }
    
    public int getLastContactAreaKnown() {
        return this.lastContactAreaKnown;
    }
    
    public void setLastContactAreaKnown(int val) {
        this.lastContactAreaKnown = val;
    }
    
    public int getTimeSinceLastPlan() {
        return timeSinceLastPlan;
    }

    public void setTimeSinceLastPlan(int t) {
        timeSinceLastPlan = t;
    }
    
    public void incrementTimeSinceLastPlan() {
        timeSinceLastPlan++;
    }
    
    public double getMaxRateOfInfoGatheringBelief() {
        return maxRateOfInfoGatheringBelief;
    }
    
    public void setMaxRateOfInfoGatheringBelief(double rate) {
        maxRateOfInfoGatheringBelief = rate;
    }
    
    public double getCurrentTotalKnowledgeBelief() {
        return currentTotalKnowledgeBelief;
    }
    
    public void setCurrentTotalKnowledgeBelief(double val) {
        currentTotalKnowledgeBelief = val;
    }
    
    // used in utility exploration
    // returns: how much information/utility we assume the base station will already know
    // by the time we deliver out information.
    public double getCurrentBaseKnowledgeBelief() {
        return currentBaseKnowledgeBelief;        
    }
    
    public void setCurrentBaseKnowledgeBelief(double val) {
        currentBaseKnowledgeBelief = val;
    }
    
    // general logging/reporting stats
    public void incrementTimeSensing(int curTimestep) {
        if (curTimestep > lastIncrementedTimeSensing) {
            timeSensing++;
            lastIncrementedTimeSensing = curTimestep;
        }
    }
    
    //If we have sensed new in this timestep, we cannot also doublesense in the same timestep.
    public void incrementTimeDoubleSensing(int curTimestep) {
        if ((curTimestep > lastIncrementedTimeDoubleSensing) && (curTimestep > lastIncrementedTimeSensing)) {
            timeDoubleSensing++;
            lastIncrementedTimeDoubleSensing = curTimestep;
        }
    }
    
    public void incrementTimeReturning(int curTimestep) {
        if (curTimestep > lastIncrementedTimeReturning) {
            timeReturning++;
            lastIncrementedTimeReturning = curTimestep;
        }
    }
    
    // NEW sensing (this does not count double-sensing
    public int getTimeSensing() {
        return timeSensing;
    }
    
    public int getTimeDoubleSensing() {
        return timeDoubleSensing;
    }
    
    public int getTimeReturning() {
        return timeReturning;
    }
    
    public void commWithBaseStation(int curTimestep) {
        for (int i = timeBaseMessageReceived.size(); i < curTimestep; i++)
            timeBaseMessageReceived.add(curTimestep);
    }
    
    public void commWithTeammate(int curTimestep, int teammateBaseMessageListSize) {
        for (int i = timeBaseMessageReceived.size(); i < teammateBaseMessageListSize; i++)
            timeBaseMessageReceived.add(curTimestep);
    }
    
    public int getTimeBaseMessageListSize() {
        return timeBaseMessageReceived.size();
    }
    
    public int getMaxLatency() {
        int maxLatency = 0;
        int curTime = 0;
        for (Integer item: timeBaseMessageReceived) {
            curTime++;
            int latency = item - curTime;
            if (latency > maxLatency) maxLatency = latency;
        }
        return maxLatency;
    }
    
    public double getAvgLatency() {
        int totalLatency = 0;
        int curTime = 0;
        for (Integer item: timeBaseMessageReceived) {
            curTime++;
            totalLatency += item - curTime;
        }
        return (double)totalLatency / (double)curTime;
    }
}
