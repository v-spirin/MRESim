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

package Logging;

import agents.RealAgent;
import config.Constants;
import config.SimulatorConfig;
import gui.MainConsole;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author christian
 */
public class ExplorationLogger {

    private static final Logger LOGGER = Logger.getLogger(MainConsole.class.getName());
    HashMap<RealAgent, HashMap<Integer, AgentStats>> log;
    //HashMap<RealAgent, HashMap<Integer, Integer>> log;
    RealAgent[] agents;
    String name;
    private final SimulatorConfig simConfig;

    public ExplorationLogger(RealAgent[] agents, String name, SimulatorConfig simConfig) {
        this.log = new HashMap();
        this.agents = agents;
        this.name = name;
        this.simConfig = simConfig;
        for (RealAgent agt : agents) {
            log.put(agt, new HashMap());
        }
    }

    public void log(int time, RealAgent[] agents) {
        for (RealAgent agent : agents) {
            log.get(agent).put(time, new AgentStats(agent.getStats()));
        }
    }

    public void log(int time, RealAgent agent) {
        log.get(agent).put(time, new AgentStats(agent.getStats()));
    }

    public void writeLog(int timeElapsed) {
        double total = simConfig.getEnvironment().getTotalFreeSpace();
        PrintWriter exploration;
        try {
            exploration = new PrintWriter(Constants.DEFAULT_LOG_DIRECTORY + "exploration" + this.name + ".csv");

            //exploration.printf("Cycle,%s,%s,%s,ComStationsInUse,EnergyConsumption,TotalCommunications\n", agents[0].getName(), agents[1].getName(), agents[2].getName());
            String header = "";
            header += "Cycle" + ",";
            for (RealAgent agt : agents) {
                header += agt.getName() + ",";
            }
            header += "ComStationsInUse" + ",";
            header += "EnergyConsumption" + ",";
            header += "TotalCommunications";

            exploration.println(header);

            for (int i = 1; i < timeElapsed; i++) {
                /*long area1 = Math.round(100 * (double) this.log.get(agents[0]).get(i).getAreaKnown() / (double) total);
                long area2 = Math.round(100 * (double) this.log.get(agents[1]).get(i).getAreaKnown() / (double) total);
                long area3 = Math.round(100 * (double) this.log.get(agents[2]).get(i).getAreaKnown() / (double) total);
                int coms = this.log.get(agents[0]).get(i).getComStationsDropped();
                 */
                //int energy = log.get(agents[0]).get(i).getBatteryPower();
                //energy += log.get(agents[1]).get(i).getBatteryPower();
                //energy += log.get(agents[2]).get(i).getBatteryPower();
                //int communications = log.get(agents[0]).get(i).getCommunications();
                //communications += log.get(agents[1]).get(i).getCommunications();
                //communications += log.get(agents[2]).get(i).getCommunications();
                //exploration.printf("%d,%d,%d,%d,%d,%d,%d\n", i, area1, area2, area3, coms, energy, communications);
                String line = "";
                //Cycle
                line += i + ",";
                //Agent agre known
                for (RealAgent agt : agents) {
                    line += Math.round(100 * (double) this.log.get(agt).get(i).getAreaKnown() / (double) total) + ",";
                }
                //ComStations
                line += this.log.get(agents[0]).get(i).getComStationsDropped() + ",";
                //EnergyUsage
                int energy = 0;
                for (RealAgent agt : agents) {
                    energy += log.get(agt).get(i).getBatteryPower();
                }
                line += energy + ",";
                //Communication
                int communications = 0;
                for (RealAgent agt : agents) {
                    communications += log.get(agt).get(i).getCommunications();
                }
                line += communications;
                exploration.println(line);

            }
//            exploration.flush();
            exploration.close();
        } catch (FileNotFoundException ex) {
            LOGGER.log(Level.SEVERE, null, ex);
            System.out.println("Mist");
        }

    }

}
