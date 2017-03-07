/*
 *     Copyright 2010, 2015 Julian de Hoog (julian@dehoog.ca), Victor Spirin (victor.spirin@cs.ox.ac.uk)
 *
 *     This file is part of MRESim 2.2, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our papers:
 *
 *     @inproceedings{deHoog2009,
 *         title = "Role-Based Autonomous Multi-Robot Exploration",
 *         author = "Julian de Hoog, Stephen Cameron and Arnoud Visser",
 *         year = "2009",
 *         booktitle = "International Conference on Advanced Cognitive Technologies and Applications (COGNITIVE)",
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
package gui.ShowSettings;

/**
 *
 * @author Victor
 */
public class ShowSettingsAgent {
    public boolean showAgent;
    public boolean showCommRange;
    public boolean showFrontiers;
    public boolean showPath;
    public boolean showRVCandidatePointInfo;
    public boolean showTopologicalMap;
    public boolean showRendezvous;
    public boolean showFreeSpace;
    public boolean showBaseSpace;
    public boolean showSafeSpace;
    public boolean showTestSpace;
    public boolean showBorderSkel;
    public boolean showRVWalls;
    public boolean saveOccupancyGrid;
    
    public ShowSettingsAgent()
    {
        showAgent = false;
        showCommRange = false;
        showFrontiers = false;
        showPath = false;
        showRVCandidatePointInfo = false;
        showTopologicalMap = false;
        showRendezvous = false;
        showFreeSpace = false;
        showBaseSpace = true;
        showSafeSpace = false;
        showTestSpace = false;
        showBorderSkel = false;
        showRVWalls = false;
        saveOccupancyGrid = false;
    }
}
