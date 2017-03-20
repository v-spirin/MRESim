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

import agents.RealAgent;
import config.RobotTeamConfig;
import config.SimulatorConfig;
import exploration.SimulationFramework;

/**
 *
 * @author christian
 */
public class MainConsole extends MainGUI {
//    private boolean loop;
    
    public static void main(String args[]){
        MainConsole mainConsole = new MainConsole();
    }
    
    public MainConsole(){
        robotTeamConfig = new RobotTeamConfig();
        simConfig = new SimulatorConfig();
        explorationImage = new ExplorationImage(simConfig.getEnv());
        explorationImage.redrawEnvAndAgents(this, robotTeamConfig, simConfig);
        simulation = new SimulationFramework(this, robotTeamConfig, simConfig, explorationImage);
        simulation.start();
//        loop = true;
//        while(loop){
//            loop = !simulation.simulationCycle();
//            //explorationImage.redrawEnvAndAgents(this, robotTeamConfig, simConfig);
//        }
    }
    
    @Override
    public void runComplete() {
        System.out.println("Finished");
        System.exit(0);
//        loop = false;
    }
    
    @Override
    public void updateRobotConfig(){}
    
    @Override
    public void updateFromData(RealAgent agent[], int timeElapsed, double pctAreaKnown, int avgCycleTime){
         System.out.println("Cycle: " + timeElapsed + "\nAreaKnown: " + Math.round(pctAreaKnown) + "\nAvgTime/Cycle: " + avgCycleTime);

    }
    
}
