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
package exploration.rendezvous;

/**
 *
 * @author Victor
 */
public class MultiPointRendezvousStrategySettings {
    //if enabled, agents will try to move to area that they estimate will have better signal strength with the
    //other agent rendezvous position, while they are waiting for that agent
    public static enum strategy{RelayCloseToBase, RelayMostRange, RelayMostRangeExplorerThreshold, Utility}
    /*
    RelayCloseToBase: Explorer's point fixed as AAMAS paper, Relay's point sampled to be closest to base;
    RelayMostRange: Explorer's point fixed as AAMAS paper. Relay's point sampled to have the most sampled points in comm range, including explorer's point.
    RelayMostRangeExplorerThreshold: Same as above, except try all explorer points within N meters threshold, pick the one that has the most points in range of relay.
    Utility: Neither point fixed, using utility function described above.
    */
    public strategy MPRVStrategy;
    public boolean moveToBetterCommsWhileWaiting;
    public double SamplePointDensity;
    public boolean replanOurMeetingPoint;
    public boolean attemptExplorationByRelay; //should relay try to explore some frontiers if otherwise it will arrive at RV too early
    public boolean tryToGetToExplorerRV; //if the relay has time, should it try to go to explorer's RV location instead of its own?
                                         //this improves the chances that explorer will enter relay's range sooner when it heads back to RV
    public boolean useSingleMeetingTime; //should we plan to arrive at RV at the same time as if we were using a single RV point?
}
