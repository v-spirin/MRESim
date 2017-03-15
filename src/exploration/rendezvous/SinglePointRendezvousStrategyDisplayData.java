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
import config.Constants;
import gui.ExplorationImage;
import java.awt.Point;
import java.util.LinkedList;
import java.util.List;

/**
 *
 * @author Victor
 */
public class SinglePointRendezvousStrategyDisplayData implements IRendezvousDisplayData {

    private LinkedList<Point> skeleton;
    private LinkedList<Point> rvPoints;

    public SinglePointRendezvousStrategyDisplayData() {
        skeleton = new LinkedList<Point>();
        rvPoints = new LinkedList<Point>();
    }

    public LinkedList<Point> getSkeleton() {
        return skeleton;
    }

    public void setSkeleton(LinkedList<Point> list) {
        skeleton = list;
    }

    public LinkedList<Point> getRVPoints() {
        return rvPoints;
    }

    public void setRVPoints(LinkedList<Point> list) {
        rvPoints = list;
    }

    @Override
    public List<Point> getDirtyCells(ExplorationImage image, RealAgent agent) {
        List<Point> dirtyCells = new LinkedList<Point>();
        // Erase old skeleton
        getSkeleton().stream().forEach((p) -> {
            dirtyCells.add(p);
        });

        // Erase old RV points
        for (Point rv : getRVPoints()) {
            for (int i = Math.max(rv.x - 4, 0); i <= Math.min(rv.x + 4, image.getWidth() - 1); i++) {
                for (int j = Math.max(rv.y - 4, 0); j <= Math.min(rv.y + 4, image.getHeight() - 1); j++) {
                    dirtyCells.add(new Point(i, j));
                }
            }
        }

        //Erase text over agents
        for (int i = agent.getX(); i <= agent.getX() + 100; i++) {
            for (int j = agent.getY() - Constants.AGENT_RADIUS - 25; j <= agent.getY() - Constants.AGENT_RADIUS; j++) {
                if (agent.getOccupancyGrid().locationExists(i, j)) {
                    agent.getDirtyCells().add(new Point(i, j));
                }
            }
        }

        return dirtyCells;
    }

    @Override
    public void drawCandidatePointInfo(ExplorationImage image) {
        try {
            getSkeleton().stream().forEach((p) -> {
                image.setPixel(p.x, p.y, Constants.MapColor.skeleton());
            });

            getRVPoints().stream().forEach((p) -> {
                image.drawPoint(p.x, p.y, Constants.MapColor.rvPoints());
            });
        } catch (java.lang.NullPointerException e) {
        }
    }

    @Override
    public void drawRendezvousLocation(ExplorationImage image, RealAgent agent) {
        int x, y;
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        // Draw Child RV
        try {
            x = (int) rvd.getChildRendezvous().getParentLocation().getX();
            y = (int) rvd.getChildRendezvous().getParentLocation().getY();
            image.drawPoint(x, y, Constants.MapColor.childRV());
            for (int i = Math.max(0, x - 4); i <= Math.min(x + 4, image.getWidth() - 1); i++) {
                for (int j = Math.max(0, y - 4); j <= Math.min(y + 4, image.getHeight() - 1); j++) {
                    agent.getDirtyCells().add(new Point(i, j));
                }
            }
            image.drawText("c:" + rvd.getChildRendezvous().getTimeMeeting() + ":" + rvd.getChildRendezvous().getTimeWait(),
                    agent.getLocation().x, agent.getLocation().y - 10, Constants.MapColor.text());

        } catch (java.lang.NullPointerException e) {
        }

        // Draw Parent RV
        try {
            x = (int) rvd.getParentRendezvous().getChildLocation().getX();
            y = (int) rvd.getParentRendezvous().getChildLocation().getY();
            image.drawPoint(x, y, Constants.MapColor.parentRV());
            for (int i = Math.max(0, x - 4); i <= Math.min(x + 4, image.getWidth() - 1); i++) {
                for (int j = Math.max(0, y - 4); j <= Math.min(y + 4, image.getHeight() - 1); j++) {
                    agent.getDirtyCells().add(new Point(i, j));
                }
            }
            image.drawText("p:" + rvd.getParentRendezvous().getTimeMeeting() + ":" + rvd.getParentRendezvous().getTimeWait(),
                    agent.getLocation().x, agent.getLocation().y - 20, Constants.MapColor.text());
        } catch (java.lang.NullPointerException e) {
        }

    }
}
