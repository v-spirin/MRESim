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

package batch;

import config.Constants;
import config.RobotTeamConfig;
import config.SimulatorConfig;
import gui.MainConsole;
import java.io.File;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Christian Clausen
 */
public class BatchExecution {

    private static final Logger LOGGER = Logger.getLogger(BatchExecution.class.getName());

    private MainConsole console;
    private List<SimulatorConfig> configs;
    private RobotTeamConfig team;
    int num_threads = 2;
    int num_sims = 1;

    public BatchExecution() {
        configs = new ArrayList<>();
        SimulatorConfig config = new SimulatorConfig();

        team = new RobotTeamConfig();

        config.setExpAlgorithm(SimulatorConfig.exptype.LeaderFollower);
        config.setCommModel(SimulatorConfig.commtype.StaticCircle);
        team.loadConfig(Constants.DEFAULT_CONF_DIRECTORY + "leaderFollower_1-1_maze1_100");
        boolean loaded = config.loadEnvironment(Constants.DEFAULT_ENV_DIRECTORY + "maze1.png");

        //Fixed settings
        /*config.setExpAlgorithm(SimulatorConfig.exptype.Testing);
        config.setCommModel(SimulatorConfig.commtype.DirectLine);
        config.setUseComStations(true);
        config.setRelayAlgorithm(SimulatorConfig.relaytype.Random);
        for (int i = 0; i < num_threads; i++) {
            config.setComStationDropChance(0.1 + (0.1 * i));
            configs.add(config);
        }*/
        //Team
        //team.loadConfig("maze_hill_2robots_8relays");
        //team.loadConfig("maze_hill_2robots");
        //boolean loaded = config.loadEnvironment("maze_with_hill");
        if (!loaded) {
            System.err.println("Could not load env");
        }
        /*
        team.loadConfig("maze1_2robots_8relays");
        config.loadEnvironment("maze1");
         */
        for (int i = 0; i < num_sims; i++) {
            configs.add(config);
        }
    }

    public void run() {
        List<Thread> threads = new ArrayList<Thread>();
        int batch_counter = 0;
        int counter_threads = 0;
        Iterator<SimulatorConfig> c_it = configs.iterator();
        while (c_it.hasNext()) {
            while (c_it.hasNext() && counter_threads < num_threads) {
                SimulatorConfig conf = c_it.next();
                String name = "Batch " + batch_counter;
                new File(Constants.DEFAULT_IMAGE_LOG_DIRECTORY + name).mkdir();
                //console = new MainConsole(true, name);
                console = new MainConsole(true, name);
                if (team != null) {
                    console.setRobotTeamConfig(team);
                }
                console.loadConfig(conf);
                console.load();
                Thread worker = new Thread(console, name);
                worker.setName(name);
                worker.start();
                threads.add(worker);
                batch_counter++;
                counter_threads++;
            }
            for (int i = batch_counter - counter_threads; i < batch_counter; i++) {
                try {
                    threads.get(i).join();
                    LOGGER.log(Level.INFO, "Thread {0} joined", i);
                } catch (Exception e) {
                    LOGGER.log(Level.SEVERE, "Thread {0} threw exception {1}: {2}", new Object[]{i, e.getMessage(), e});
                }
            }
            counter_threads = 0;
        }
    }

    public static void main(String args[]) {
        BatchExecution batch = new BatchExecution();
        batch.run();
        System.exit(0);
    }
}
