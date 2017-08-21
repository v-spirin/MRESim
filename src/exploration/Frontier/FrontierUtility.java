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
package exploration.Frontier;

import agents.Agent;
import agents.RealAgent;
import environment.Frontier;
import java.awt.Point;
import path.Path;

/**
 *
 * @author christian
 */
public class FrontierUtility implements Comparable<FrontierUtility> {

    private Agent agent;
    private Frontier frontier;
    private double utility;
    private Path path;
    private boolean exact;

    public FrontierUtility(Agent agent, Frontier f) {
        this.agent = agent;
        frontier = f;
        utility = utilityEstimate(agent.getLocation(), frontier);
        path = null;
        exact = false;

    }

    @Override
    public int compareTo(FrontierUtility other) {
        if (other.utility > this.utility) {
            return 1;
        } else if (other.utility == this.utility) {
            return 0;
        } else {
            return -1;
        }
    }

    public Path getPath() {
        return path;
    }

    public double getUtility() {
        return utility;
    }

    public Frontier getFrontier() {
        return frontier;
    }

    public Agent getAgent() {
        return agent;
    }

    public boolean isExact() {
        return exact;
    }

    public double getEcaxtUtility(RealAgent calcAgent) {
        if (!exact) {
            calculateUtilityExact(calcAgent);
        }
        return this.utility;
    }

    @Override
    public String toString() {
        return "FrontierUtility ID: " + agent.getID() + ", agentLocation: (" + (int) agent.getLocation().getX() + "," + (int) agent.getLocation().getY() + "), frontier: " + frontier
                + ", utility: " + utility;
    }

    private double utilityEstimate(Point agentLoc, Frontier frontier) {
        if (agentLoc.getX() == frontier.getCentre().x
                && agentLoc.getY() == frontier.getCentre().y) {
            return -1001;
        }
        return ((frontier.getArea() * 100000000) / Math.pow(agentLoc.distance(frontier.getCentre()), 4));
    }

    private void calculateUtilityExact(RealAgent calcAgent) {
        if (agent.getLocation() == frontier.getCentre()) {
            utility = -1001;
            exact = true;
            return;
        }
        path = calcAgent.calculatePath(agent.getLocation(), frontier.getCentre(), false/*ute.frontier.getClosestPoint(start, agent.getOccupancyGrid())*/);

        if (path.found) {
            utility = (frontier.getArea() * 100000000) / Math.pow(path.getLength(), 4);
        } else {
            utility = -1000;
        }
        exact = true;
    }
}
