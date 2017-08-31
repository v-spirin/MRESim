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

package exploration.rendezvous;

import agents.RealAgent;
import agents.TeammateAgent;
import java.awt.Point;
import java.util.List;
import java.util.PriorityQueue;
import path.Path;

/**
 *
 * @author Christian Clausen <christian.clausen@uni-bremen.de>
 */
class SimpleRendezvousStrategy implements IRendezvousStrategy {

    private final RealAgent agent;
    private final SinglePointRendezvousStrategyDisplayData displayData;
    private final SimpleRendezvousStrategySettings settings;

    public SimpleRendezvousStrategy(RealAgent agent, SimpleRendezvousStrategySettings settings) {
        this.agent = agent;
        displayData = new SinglePointRendezvousStrategyDisplayData();
        this.settings = settings;
    }

    @Override
    public Rendezvous calculateRendezvous(int timeElapsed, TeammateAgent mate) {
        return new Rendezvous(calculateRVPoint(agent, mate));
    }

    @Override
    public Rendezvous calculateRendezvousRelayWithRelay(int timeElapsed, TeammateAgent mate) {
        return new Rendezvous(calculateRVPoint(agent, mate));
    }

    @Override
    public void processExplorerStartsHeadingToRV() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void processReturnToParentReplan() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void processExplorerCheckDueReturnToRV() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public Path processGoToChildReplan() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public Point processWaitForParent() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public Point processWaitForChild() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public Point processWaitForChildTimeoutNoBackup() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void processJustGotIntoParentRange(int timeElapsed, TeammateAgent parent) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void processAfterGiveParentInfoExplorer(int timeElapsed) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void processAfterGiveParentInfoRelay() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void processAfterGetInfoFromChild() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public IRendezvousDisplayData getRendezvousDisplayData() {
        return displayData;
    }

    @Override
    public RealAgent getAgent() {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    @Override
    public void setAgent(RealAgent ag) {
        throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
    }

    public Point calculateRVPoint(RealAgent agent, TeammateAgent mate) {
        List<Point> pts = agent.getOccupancyGrid().getSkeletonList();

        if (pts == null || pts.isEmpty()) {
            return agent.getLocation();
        } else if (mate.isStationary()) {
            return mate.getLocation();
        } else {
            PriorityQueue<NearRVPoint> tempPoints = new PriorityQueue<>();
            for (Point p : pts) {

                tempPoints.add(new NearRVPoint(p.x, p.y, p.distance(mate.getLocation())));
            }
            return tempPoints.peek().getLocation();
        }
    }

}
