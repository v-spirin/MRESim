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

package exploration;

import agents.Agent.AgentState;
import config.RobotConfig;
import java.awt.Point;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

/**
 *
 * @author julh
 */
public class RunFromLog implements Exploration {

    String fileName;
    int agentNo;

    public RunFromLog(String fileName, int agentNo) {
        this.fileName = fileName;
        this.agentNo = agentNo;
    }

    @Override
    public Point takeStep(int time) {
        Point nextStep = new Point(0, 0);

        File file = new File(fileName);

        if (file.exists()) {
            try (BufferedReader inFile = new BufferedReader(new FileReader(file))) {
                for (int i = 0; i < time; i++) {
                    inFile.readLine();
                }

                String tokens[] = inFile.readLine().split("\\s");
                nextStep.x = Integer.parseInt(tokens[(agentNo - 1) * 6 + 1]);
                nextStep.y = Integer.parseInt(tokens[(agentNo - 1) * 6 + 2]);
            } catch (NullPointerException e) {
                // No more data, run finished.
                return null;
            } catch (IOException e) {
                System.out.println("RunFromLog: Error -- could not read data from " + fileName);
            } catch (NumberFormatException e) {
                System.out.println("RunFromLog: Error -- incorrect data format in file " + fileName);
            }
        }
        return nextStep;
    }

    public Point getGoal(int time) {
        Point nextStep = new Point(0, 0);

        File file = new File(fileName);

        if (file.exists()) {
            try (BufferedReader inFile = new BufferedReader(new FileReader(file))) {
                for (int i = 0; i < time; i++) {
                    inFile.readLine();
                }

                String tokens[] = inFile.readLine().split("\\s");
                nextStep.x = (int) Double.parseDouble(tokens[(agentNo - 1) * 6 + 3]);
                nextStep.y = (int) Double.parseDouble(tokens[(agentNo - 1) * 6 + 4]);
            } catch (NullPointerException e) {
                // No more data, run finished.
                return null;
            } catch (IOException e) {
                System.out.println("RunFromLog: Error -- could not read data from " + fileName);
            } catch (NumberFormatException e) {
                System.out.println("RunFromLog: Error -- incorrect data format in file " + fileName);
            }
        }
        return nextStep;
    }

    public AgentState getState(int time) {
        /*Point nextStep = new Point(0, 0);

        File file = new File(fileName);

        if (file.exists()) {
            try (BufferedReader inFile = new BufferedReader(new FileReader(file))) {
                for (int i = 0; i < time; i++) {
                    inFile.readLine();
                }

                String tokens[] = inFile.readLine().split("\\s");
                String state = tokens[(agentNo - 1) * 6 + 6];
                if (state.equals("GetInfoFromChild")) {
                    return AgentState.GetInfoFromChild;
                }
                if (state.equals("Explore")) {
                    return AgentState.Explore;
                }
                if (state.equals("GiveParentInfo")) {
                    return AgentState.GiveParentInfo;
                }
                if (state.equals("GoToChild")) {
                    return AgentState.GoToChild;
                }
                if (state.equals("Initial")) {
                    return AgentState.Initial;
                }
                if (state.equals("OutOfService")) {
                    return AgentState.OutOfService;
                }
                if (state.equals("ReturnToParent")) {
                    return AgentState.ReturnToBaseStation;
                }
                if (state.equals("WaitForChild")) {
                    return AgentState.WaitForChild;
                }
                if (state.equals("WaitForParent")) {
                    return AgentState.WaitForParent;
                }
            } catch (NullPointerException e) {
                // No more data, run finished.
                return null;
            } catch (IOException e) {
                System.out.println("RunFromLog: Error -- could not read data from " + fileName);
            } catch (NumberFormatException e) {
                System.out.println("RunFromLog: Error -- incorrect data format in file " + fileName);
            }
        }*/
        return AgentState.Initial;
    }

    public RobotConfig.roletype getRole(int time) {
        Point nextStep = new Point(0, 0);

        File file = new File(fileName);

        if (file.exists()) {
            try (BufferedReader inFile = new BufferedReader(new FileReader(file))) {
                for (int i = 0; i < time; i++) {
                    inFile.readLine();
                }

                String tokens[] = inFile.readLine().split("\\s");
                String role = tokens[(agentNo - 1) * 6 + 5];
                if (role.equals("BaseStation")) {
                    return RobotConfig.roletype.BaseStation;
                }
                if (role.equals("Explorer")) {
                    return RobotConfig.roletype.Explorer;
                }
                if (role.equals("Relay")) {
                    return RobotConfig.roletype.Relay;
                }
            } catch (NullPointerException e) {
                // No more data, run finished.
                return null;
            } catch (IOException e) {
                System.out.println("RunFromLog: Error -- could not read data from " + fileName);
            } catch (NumberFormatException e) {
                System.out.println("RunFromLog: Error -- incorrect data format in file " + fileName);
            }
        }
        return RobotConfig.roletype.BaseStation;
    }

}
