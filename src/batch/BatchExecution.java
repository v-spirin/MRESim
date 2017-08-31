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

import config.SimConstants;
import config.RobotTeamConfig;
import config.SimulatorConfig;
import gui.MainConsole;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
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

    //private MainConsole console;
    //private List<SimulatorConfig> configs;
    private List<String[]> configFiles;
    //private RobotTeamConfig team;
    int num_sims = 1;
    int num_threads = 2;

    public BatchExecution(String batchfile) {
        configFiles = new ArrayList<>();
        if (batchfile != null) {
            if (!loadBatchConfig(batchfile)) {
                System.exit(1);
            }
        } else {
            String[] co = {SimConstants.DEFAULT_SIMCONF_DIRECTORY + "frontierbased_periodicReturn",
                SimConstants.DEFAULT_TEAMCONF_DIRECTORY + "frontierbased_1_maze1_100",
                SimConstants.DEFAULT_ENV_DIRECTORY + "maze1.png"
            };
            configFiles.add(co);
        }
    }

    public void run() {
        List<Thread> threads = new ArrayList<Thread>();
        int batch_counter = 0;
        int counter_threads = 0;
        Iterator<String[]> c_it = configFiles.iterator();
        while (c_it.hasNext()) {
            while (c_it.hasNext() && counter_threads < num_threads) {
                String[] confs = c_it.next();
                String name = "Batch " + batch_counter;

                SimulatorConfig conf = new SimulatorConfig();
                conf.loadSimulatorConfig(confs[0]);
                RobotTeamConfig team = new RobotTeamConfig();
                team.loadConfig(confs[1]);
                boolean loaded = conf.loadEnvironment(confs[2]);
                if (!loaded) {
                    System.err.println(name + ": Could not load env: " + confs[2]);
                    continue;
                }

                new File(SimConstants.DEFAULT_IMAGE_LOG_DIRECTORY + name).mkdir();
                MainConsole console = new MainConsole(true, name);
                console.setRobotTeamConfig(team);
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
        System.out.println("MRESim GUI-less Execution");
        String batchfile = null;
        if (args.length >= 1) {
            batchfile = args[0];
            System.out.println(batchfile);
        }
        BatchExecution batch = new BatchExecution(batchfile);
        batch.run();
        System.exit(0);
    }

    private boolean loadBatchConfig(String fileName) {
        File file = new File(fileName);

        if (file.exists()) {
            try (BufferedReader inFile = new BufferedReader(new FileReader(file))) {

                num_sims = Integer.parseInt(inFile.readLine());
                num_threads = Integer.parseInt(inFile.readLine());
                String line;
                while (inFile.ready()) {
                    String[] co = {SimConstants.DEFAULT_SIMCONF_DIRECTORY + String.valueOf(inFile.readLine()),
                        SimConstants.DEFAULT_TEAMCONF_DIRECTORY + String.valueOf(inFile.readLine()),
                        SimConstants.DEFAULT_ENV_DIRECTORY + String.valueOf(inFile.readLine())
                    };
                    configFiles.add(co);
                }

            } catch (IOException ex) {
                Logger.getLogger(BatchExecution.class.getName()).log(Level.SEVERE, null, ex);
                return false;
            }
        }
        return true;
    }

}
