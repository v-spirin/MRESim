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

package gui;

import Logging.ExplorationLogger;
import agents.RealAgent;
import config.Constants;
import config.RobotTeamConfig;
import config.SimulatorConfig;
import java.io.File;
import java.util.logging.Level;
import java.util.logging.Logger;
import simulator.ExplorationImage;
import simulator.SimulationFramework;

/**
 * Run simulation without GUI. Main-method, startable
 *
 * @author Christian Clausen
 */
public class MainConsole extends MainGUI implements Runnable {

    private static final Logger LOGGER = Logger.getLogger(MainConsole.class.getName());
    private boolean batch;
    private String threadName = "unnamed";
    private ExplorationLogger exploreLog;
    private boolean loaded = false;

    /**
     * Start simulation without GUI
     *
     * @param args none
     */
    public static void main(String args[]) {
        MainConsole mainConsole = new MainConsole(false, "SingleThread");
        mainConsole.load();
        try {
            mainConsole.start();
        } catch (InterruptedException ex) {
            LOGGER.log(Level.SEVERE, null, ex);
        }
    }

    /**
     * Constructor for ConsoleMode
     *
     * @param batch TRUE if used in batch-execution
     * @param name Name of the run, used in threadnaming and logging
     */
    public MainConsole(boolean batch, String name) {
        this.batch = batch;
        this.threadName = name;
        robotTeamConfig = new RobotTeamConfig();
        simConfig = new SimulatorConfig();
        explorationImage = new ExplorationImage(simConfig.getEnvironment());
        new File(Constants.DEFAULT_IMAGE_LOG_DIRECTORY + this.threadName).mkdir();
    }

    /**
     * Call after creating! Needs to be called to load the simulation in a save state.
     */
    public void load() {
        explorationImage.redrawEnvAndAgents(this, robotTeamConfig, simConfig);
        simulation = new SimulationFramework(this, robotTeamConfig, simConfig, explorationImage);
        loaded = true;
    }

    /**
     * Starts the simulation
     *
     * @throws InterruptedException if called before load()
     */
    public void start() throws InterruptedException {
        if (loaded) {
            simulation.start();
        } else {
            throw new IllegalStateException("Need to call load() first!");
        }
    }

    /**
     * Loads a specific simulation-config. Otherwise the defaultconfig is used
     *
     * @param simConf specific simulation-config
     */
    public void loadConfig(SimulatorConfig simConf) {
        this.simConfig = simConf;
    }

    /**
     * Gets called by threads, DO NOT CALL YOURSELF!
     *
     * @param agent
     * @param timeElapsed
     * @param pctAreaKnownTeam
     * @param avgCycleTime
     */
    @Override
    public void runComplete(RealAgent[] agent, int timeElapsed, double pctAreaKnownTeam, int avgCycleTime) {
        simulation.logScreenshot(Constants.DEFAULT_IMAGE_LOG_DIRECTORY + this.threadName + File.separatorChar);

        System.out.format("%s\n"
                + "\tCycle: %d\n"
                + "\tAreaKnown: %d%%\n"
                + "\tAvgTime/Cycle: %d\n\n",
                new Object[]{threadName, timeElapsed, Math.round(pctAreaKnownTeam), avgCycleTime});
        LOGGER.log(Level.FINE, "{0} finished", this.threadName);
        this.exploreLog.writeLog(timeElapsed);
        if (!batch) {
            System.exit(0);
        } else {
            synchronized (this) {
                this.notify();
            }
        }
    }

    /**
     * Overrides the inherited updateRobotConfig() which would update Robot-data in GUI. DO NOT CALL
     * YOURSELF
     */
    @Override
    public void updateRobotConfig() {
    }

    /**
     * Gets called by threads, DO NOT CALL YOURSELF!
     *
     * @param agent
     * @param timeElapsed
     * @param pctAreaKnown
     * @param avgCycleTime
     */
    @Override
    public void updateFromData(RealAgent agent[],
            int timeElapsed,
            double pctAreaKnown,
            int avgCycleTime) {
        if (this.exploreLog == null) {
            this.exploreLog = new ExplorationLogger(agent, threadName, simConfig);
        }
        //this.exploreLog.writeLog(timeElapsed);
        LOGGER.log(Level.FINE, "Name: {0}\n"
                + "Cycle: {1}\n"
                + "AreaKnown: {2}%\n"
                + "AvgTime/Cycle: {3}",
                new Object[]{this.threadName, timeElapsed, Math.round(pctAreaKnown), (int) avgCycleTime});
        //simulation.logScreenshot(Constants.DEFAULT_IMAGE_LOG_DIRECTORY + this.threadName + File.separatorChar);
        if ((timeElapsed % 50) == 0) {
            simulation.logScreenshot(Constants.DEFAULT_IMAGE_LOG_DIRECTORY + this.threadName + File.separatorChar);
            System.out.format("Name: %s"
                    + "\nCycle: %d\n"
                    + "AreaKnown: %d%%\n"
                    + "AvgTime/Cycle: %d\n",
                    new Object[]{this.threadName, timeElapsed, Math.round(pctAreaKnown), avgCycleTime});
        }
        exploreLog.log(timeElapsed, agent);

    }

    /**
     * Needed in batchmode for multithreading. Only calls start, cares about multithread-logging and
     * Exceptions DO NOT CALL YOURSELF!
     */
    @Override
    public void run() {
        LOGGER.log(Level.FINE, "{0} started", this.threadName);
        try {
            start();
        } catch (InterruptedException ex) {
            Logger.getLogger(MainConsole.class.getName()).log(Level.SEVERE, null, ex);
        }
        synchronized (this) {
            try {
                wait();
            } catch (InterruptedException ex) {
                Logger.getLogger(MainConsole.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

}
