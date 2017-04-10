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
//import java.io.*;
//import environment.*;

import environment.Environment;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Describes the model of the simulator configuration.
 *
 * @author julh
 */
public class SimulatorConfig {

// <editor-fold defaultstate="collapsed" desc="Variables and Constructor">
    private boolean logAgents;
    private String logAgentsFilename;
    private boolean logData;
    private String logDataFilename;
    private boolean logScreenshots;
    private String logScreenshotsDirname;
    private String runFromLogFilename;
    private String batchFilename;
    private boolean useComStations;
    private double comStationDropChance;

    public static enum commtype {
        StaticCircle, DirectLine, PropModel1
    }
    private commtype commModel;
    private int simRate;
    private Environment env;

    public static enum exptype {
        BatchRun, RunFromLog, LeaderFollower, FrontierExploration,
        RoleBasedExploration, Testing, Random, WallFollow
    }

    public static enum frontiertype {
        RangeConstrained, PeriodicReturn, ReturnWhenComplete, UtilReturn
    }
    public double TARGET_INFO_RATIO;
    private exptype expAlgorithm;
    private frontiertype frontierAlgorithm;
    private boolean useImprovedRendezvous;
    private boolean allowReplanning;
    private boolean allowRoleSwitch;
    private boolean useStrictRoleSwitch;
    private boolean RVThroughWallsEnabled;
    private boolean RVCommRangeEnabled;

    private boolean timeStampTeammateData; //timestamp our teammate data; update with newest when communicating
    //with other agents
    private boolean useTeammateNextFrontierAsLocationWhenOutOfRange; //when we are out of range with a teammate,
    //instead of their location at the time of last communication, use the
    //frontier they said they were heading towards as their assumed location
    private boolean keepAssigningRobotsToFrontiers; //if there are more robots than frontiers, some robots will not have
    //a frontier allocated to them. If this is false, they will get a frontier assigned to them without regard for other
    //agents, i.e. usually the nearest frontier to them (even if there is another unassigned robot that is closer to that
    //frontier. If it is set to True, then unassigned robots will still bid for all the frontiers and assign themselves
    //to frontiers more evenly.

    private boolean baseRange; //if true, agents will try to navigate to base station range rather than to base station
    //itself. This can yield great benefits as the actual path to base can be very convoluted, but the base comm range
    //can actually be very near to agent's current position (but there may be obstacles between agent and base station.
    private double samplingDensity; //when sampling points from the environment, how dense they need to be (1 point per
    //samplingDensity pixels. E.g. if samplingDensity = 100, we will roughly get one point per 10x10 pixel square.
    private boolean exploreReplan; //Should explorer replan the meeting with relay to take into account comm range
    private boolean relayExplore; //Should relay also explore if it gets a chance in RBE
    private boolean tryToGetToExplorerRV;
    private boolean useSingleMeetingTime;

    public SimulatorConfig() {
        boolean oldEnvVariableConfigFound = loadOldSimulatorConfig();
        if (!oldEnvVariableConfigFound) {
            //set defaults
            expAlgorithm = exptype.RoleBasedExploration;
            frontierAlgorithm = frontiertype.PeriodicReturn;
            runFromLogFilename = null;
            commModel = commtype.DirectLine;
            logAgents = false;
            logAgentsFilename = System.getProperty("user.dir") + "/logs/defaultAgentLog.txt";
            logData = false;
            logDataFilename = System.getProperty("user.dir") + "/logs/defaultDataLog.txt";
            simRate = 5;
            useImprovedRendezvous = false;
            allowReplanning = false;
            allowRoleSwitch = false;
            useStrictRoleSwitch = false;
            RVThroughWallsEnabled = false;
            RVCommRangeEnabled = false;
            timeStampTeammateData = true;
            useTeammateNextFrontierAsLocationWhenOutOfRange = false;
            keepAssigningRobotsToFrontiers = true;
            baseRange = false;
            samplingDensity = 400;
            exploreReplan = false;
            relayExplore = false;
            tryToGetToExplorerRV = false;
            useSingleMeetingTime = false;
            comStationDropChance = 0;
            useComStations = false;
        }

        boolean oldWallConfigFound = loadOldWallConfig();
        if (!oldWallConfigFound) {
            env = new Environment(Constants.MAX_ROWS, Constants.MAX_COLS);
        }
    }
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Get and Set">
    public boolean useComStations() {
        return useComStations;
    }

    public double getComStationDropChance() {
        return comStationDropChance;
    }

    public void setUseComStations(boolean selected) {
        this.useComStations = selected;
    }

    public void setComStationDropChance(double parseDouble) {
        this.comStationDropChance = parseDouble;
    }

    public commtype getCommModel() {
        return commModel;
    }

    public void setCommModel(int n) {
        commModel = commtype.values()[n];
    }

    public void setCommModel(commtype com) {
        commModel = com;
    }

    public exptype getExpAlgorithm() {
        return expAlgorithm;
    }

    public frontiertype getFrontierAlgorithm() {
        return frontierAlgorithm;
    }

    public String getRunFromLogFilename() {
        return runFromLogFilename;
    }

    public void setExpAlgorithm(exptype eType) {
        expAlgorithm = eType;
    }

    public void setRunFromLogFilename(String f) {
        runFromLogFilename = f;
    }

    public void setBatchFilename(String f) {
        batchFilename = f;
    }

    public String getBatchFilename() {
        return batchFilename;
    }

    public void setExpAlgorithm(int n) {
        expAlgorithm = exptype.values()[n];
    }

    public void setFrontierAlgorithm(int n) {
        frontierAlgorithm = frontiertype.values()[n];
    }

    public void setFrontierAlgorithm(frontiertype fType) {
        frontierAlgorithm = fType;
    }

    public int getSimRate() {
        return simRate;
    }

    public void setSimRate(int s) {
        simRate = s;
    }

    public Environment getEnvironment() {
        return env;
    }

    public boolean logAgents() {
        return logAgents;
    }

    public void setLogAgents(boolean log) {
        logAgents = log;
    }

    public String getLogAgentFilename() {
        return logAgentsFilename;
    }

    public void setLogAgentsFilename(String f) {
        logAgentsFilename = f;
    }

    public boolean logData() {
        return logData;
    }

    public void setLogData(boolean log) {
        logData = log;
    }

    public String getLogDataFilename() {
        return logDataFilename;
    }

    public void setLogDataFilename(String f) {
        logDataFilename = f;
    }

    public boolean logScreenshots() {
        return logScreenshots;
    }

    public void setLogScreenshots(boolean log) {
        logScreenshots = log;
    }

    public String getLogScreenshotsDirname() {
        return logScreenshotsDirname;
    }

    public void setLogScreenshotsDirname(String f) {
        logScreenshotsDirname = f;
    }

    public boolean roleSwitchAllowed() {
        return allowRoleSwitch;
    }

    public void setRoleSwitchAllowed(boolean rs) {
        allowRoleSwitch = rs;
    }

    public boolean strictRoleSwitch() {
        return useStrictRoleSwitch;
    }

    public void setStrictRoleSwitch(boolean rs) {
        useStrictRoleSwitch = rs;
    }

    public boolean replanningAllowed() {
        return allowReplanning;
    }

    public void setReplanningAllowed(boolean rs) {
        allowReplanning = rs;
    }

    public boolean useImprovedRendezvous() {
        return useImprovedRendezvous;
    }

    public void setUseImprovedRendezvous(boolean rv) {
        useImprovedRendezvous = rv;
    }

    public boolean RVCommRangeEnabled() {
        return RVCommRangeEnabled;
    }

    public void setRVCommRangeEnabled(boolean rvCommRangeEnabled) {
        this.RVCommRangeEnabled = rvCommRangeEnabled;
    }

    public boolean RVThroughWallsEnabled() {
        return RVThroughWallsEnabled;
    }

    public void setRVThroughWallsEnabled(boolean rvThroughWallsEnabled) {
        this.RVThroughWallsEnabled = rvThroughWallsEnabled;
    }

    public boolean timeStampTeammateDataEnabled() {
        return timeStampTeammateData;
    }

    public void setTimeStampTeammateDataEnabled(boolean setting) {
        timeStampTeammateData = setting;
    }

    public boolean useTeammateNextFrontierAsLocationWhenOutOfRange() {
        return useTeammateNextFrontierAsLocationWhenOutOfRange;
    }

    public void setUseTeammateNextFrontierAsLocationWhenOutOfRange(boolean setting) {
        useTeammateNextFrontierAsLocationWhenOutOfRange = setting;
    }

    public boolean keepAssigningRobotsToFrontiers() {
        return keepAssigningRobotsToFrontiers;
    }

    public void setKeepAssigningRobotsToFrontiers(boolean setting) {
        keepAssigningRobotsToFrontiers = setting;
    }

    public void setBaseRange(boolean br) {
        baseRange = br;
    }

    public boolean getBaseRange() {
        return baseRange;
    }

    public void setSamplingDensity(double density) {
        samplingDensity = density;
    }

    public double getSamplingDensity() {
        return samplingDensity;
    }

    public void setRelayExplore(boolean s) {
        relayExplore = s;
    }

    public boolean getRelayExplore() {
        return relayExplore;
    }

    public void setTryToGetToExplorerRV(boolean s) {
        tryToGetToExplorerRV = s;
    }

    public boolean getTryToGetToExplorerRV() {
        return tryToGetToExplorerRV;
    }

    public void setUseSingleMeetingTime(boolean s) {
        useSingleMeetingTime = s;
    }

    public boolean getUseSingleMeetingTime() {
        return useSingleMeetingTime;
    }

    public void setExploreReplan(boolean s) {
        exploreReplan = s;
    }

    public boolean getExploreReplan() {
        return exploreReplan;
    }
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Load and Save">
    private boolean loadOldSimulatorConfig() {
        String oldConfigFilename = System.getProperty("user.dir") + "/config/lastSimulatorConfig.txt";
        File file = new File(oldConfigFilename);
        if (file.exists()) {
            return loadSimulatorConfig(oldConfigFilename);
        } else {
            return false;
        }
    }

    public boolean loadSimulatorConfig(String fileName) {
        File file = new File(fileName);

        if (file.exists()) {
            try (BufferedReader inFile = new BufferedReader(new FileReader(file))) {

                expAlgorithm = exptype.valueOf(inFile.readLine());
                frontierAlgorithm = frontiertype.valueOf(inFile.readLine());
                runFromLogFilename = String.valueOf(inFile.readLine());
                commModel = commtype.valueOf(inFile.readLine());
                logAgents = Boolean.parseBoolean(inFile.readLine());
                logAgentsFilename = String.valueOf(inFile.readLine());
                logData = Boolean.parseBoolean(inFile.readLine());
                logDataFilename = String.valueOf(inFile.readLine());
                logScreenshots = Boolean.parseBoolean(inFile.readLine());
                logScreenshotsDirname = String.valueOf(inFile.readLine());
                useImprovedRendezvous = Boolean.parseBoolean(inFile.readLine());
                allowReplanning = Boolean.parseBoolean(inFile.readLine());
                allowRoleSwitch = Boolean.parseBoolean(inFile.readLine());
                useStrictRoleSwitch = Boolean.parseBoolean(inFile.readLine());
                simRate = Integer.parseInt(inFile.readLine());
                RVCommRangeEnabled = Boolean.parseBoolean(inFile.readLine());
                RVThroughWallsEnabled = Boolean.parseBoolean(inFile.readLine());
                try {
                    TARGET_INFO_RATIO = Double.parseDouble(inFile.readLine());
                } catch (IOException | NumberFormatException e) {
                    TARGET_INFO_RATIO = 0.9;
                }
                try {
                    timeStampTeammateData = Boolean.parseBoolean(inFile.readLine());
                } catch (Exception e) {
                    timeStampTeammateData = true;
                }
                try {
                    useTeammateNextFrontierAsLocationWhenOutOfRange = Boolean.parseBoolean(inFile.readLine());
                } catch (Exception e) {
                    useTeammateNextFrontierAsLocationWhenOutOfRange = false;
                }

                try {
                    keepAssigningRobotsToFrontiers = Boolean.parseBoolean(inFile.readLine());
                } catch (Exception e) {
                    keepAssigningRobotsToFrontiers = true;
                }

                try {
                    batchFilename = String.valueOf(inFile.readLine());
                } catch (Exception e) {
                    batchFilename = "";
                }

                try {
                    baseRange = Boolean.parseBoolean(inFile.readLine());
                } catch (Exception e) {
                    baseRange = false;
                }

                try {
                    samplingDensity = Double.parseDouble(inFile.readLine());
                } catch (IOException | NumberFormatException e) {
                    samplingDensity = 400;
                }

                try {
                    relayExplore = Boolean.parseBoolean(inFile.readLine());
                } catch (Exception e) {
                    relayExplore = false;
                }

                try {
                    exploreReplan = Boolean.parseBoolean(inFile.readLine());
                } catch (Exception e) {
                    exploreReplan = false;
                }

                try {
                    tryToGetToExplorerRV = Boolean.parseBoolean(inFile.readLine());
                } catch (Exception e) {
                    tryToGetToExplorerRV = false;
                }

                try {
                    useSingleMeetingTime = Boolean.parseBoolean(inFile.readLine());
                } catch (Exception e) {
                    useSingleMeetingTime = false;
                }
                try {
                    useComStations = Boolean.parseBoolean(inFile.readLine());
                } catch (Exception e) {
                    useComStations = false;
                }
                try {
                    comStationDropChance = Double.parseDouble(inFile.readLine());
                } catch (IOException | NumberFormatException e) {
                    comStationDropChance = 0.02;
                }

            } catch (IOException e) {
                System.err.println(this.toString() + "Error: could not read data from " + fileName);
            } catch (NumberFormatException e) {
                System.err.println(this.toString() + "Error: incorrect data format in file " + fileName);
            }
            return true;
        }
        return false;
    }

    private boolean loadOldWallConfig() {
        String oldConfigFilename = System.getProperty("user.dir") + "/config/lastEnvironment.png";
        File file = new File(oldConfigFilename);
        if (file.exists()) {
            return loadEnvironment(oldConfigFilename);
        } else {
            return false;
        }
    }

    public boolean loadEnvironment(String fileName) {
        env = EnvLoader.loadWallConfig(fileName);

        return env != null;
    }

    public boolean saveSimulatorConfig() {
        return (this.saveSimulatorConfig(System.getProperty("user.dir") + "/config/lastSimulatorConfig.txt"));
    }

    public boolean saveSimulatorConfig(String fileName) {
        try (PrintWriter outFile = new PrintWriter(new FileWriter(fileName))) {

            outFile.println(expAlgorithm.toString());
            outFile.println(frontierAlgorithm.toString());
            outFile.println(runFromLogFilename);
            outFile.println(commModel.toString());
            outFile.println(logAgents);
            outFile.println(logAgentsFilename);
            outFile.println(logData);
            outFile.println(logDataFilename);
            outFile.println(logScreenshots);
            outFile.println(logScreenshotsDirname);
            outFile.println(useImprovedRendezvous);
            outFile.println(allowReplanning);
            outFile.println(allowRoleSwitch);
            outFile.println(useStrictRoleSwitch);
            outFile.println(simRate);
            outFile.println(RVCommRangeEnabled);
            outFile.println(RVThroughWallsEnabled);
            outFile.println(TARGET_INFO_RATIO);
            outFile.println(timeStampTeammateData);
            outFile.println(useTeammateNextFrontierAsLocationWhenOutOfRange);
            outFile.println(keepAssigningRobotsToFrontiers);
            outFile.println(batchFilename);
            outFile.println(baseRange);
            outFile.println(samplingDensity);
            outFile.println(relayExplore);
            outFile.println(exploreReplan);
            outFile.println(tryToGetToExplorerRV);
            outFile.println(useSingleMeetingTime);
            outFile.println(useComStations);
            outFile.println(comStationDropChance);

        } catch (IOException e) {
            System.err.println(this.toString() + "Error writing to file " + fileName);
            return false;
        }
        return true;

    }

    public boolean saveWallConfig() {
        return EnvLoader.saveWallConfig(env);
    }

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Utility">
    @Override
    public String toString() {
        return ("[SimulatorConfig] expAlgorithm: " + expAlgorithm.toString()
                + "\n frontierAlgorithm: " + frontierAlgorithm.toString()
                + "\n runFromLogFilename: " + runFromLogFilename
                + "\n commModel: " + commModel.toString()
                + "\n logAgents: " + logAgents
                + "\n logAgentsFilename: " + logAgentsFilename
                + "\n logData: " + logData
                + "\n logDataFilename: " + logDataFilename
                + "\n logScreenshots: " + logScreenshots
                + "\n logScreenshotsDirname: " + logScreenshotsDirname
                + "\n useImprovedRendezvous: " + useImprovedRendezvous
                + "\n allowReplanning: " + allowReplanning
                + "\n allowRoleSwitch: " + allowRoleSwitch
                + "\n useStrictRoleSwitch: " + useStrictRoleSwitch
                + "\n simRate: " + simRate
                + "\n RVCommRangeEnabled: " + RVCommRangeEnabled
                + "\n RVThroughWallsEnabled: " + RVThroughWallsEnabled
                + "\n TARGET_INFO_RATIO: " + TARGET_INFO_RATIO
                + "\n timeStampTeammateData: " + timeStampTeammateData
                + "\n useTeammateNextFrontierAsLocationWhenOutOfRange: " + useTeammateNextFrontierAsLocationWhenOutOfRange
                + "\n keepAssigningRobotsToFrontiers: " + keepAssigningRobotsToFrontiers
                + "\n batchFilename: " + batchFilename
                + "\n baseRange: " + baseRange
                + "\n samplingDensity: " + samplingDensity
                + "\n relayExplore: " + relayExplore
                + "\n exploreReplan: " + exploreReplan);
    }
// </editor-fold>

}
