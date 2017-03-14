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
import exploration.NearRVPoint;
import gui.ExplorationImage;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;

/**
 *
 * @author Victor
 */
public class MultiPointRendezvousStrategyDisplayData implements IRendezvousDisplayData {
    private PriorityQueue<NearRVPoint> pointsNearFrontierReal;
    private List<NearRVPoint> generatedPoints;
    private List<Point> curDirtyCells;
    private SinglePointRendezvousStrategyDisplayData sprDisplayData;
    
    public MultiPointRendezvousStrategyDisplayData() {
        pointsNearFrontierReal = new PriorityQueue<NearRVPoint>();
        generatedPoints = new LinkedList<NearRVPoint>();
        curDirtyCells = new LinkedList<Point>();
    }
    
    public void setSPRDisplayData(SinglePointRendezvousStrategyDisplayData data) {
        sprDisplayData = data;
    }
    
    public PriorityQueue<NearRVPoint> getPointsNearFrontier() {
        return pointsNearFrontierReal;
    }
    
    public void setPointsNearFrontier(PriorityQueue<NearRVPoint> points) {
        pointsNearFrontierReal = points;
    }
    
    public List<NearRVPoint> getGeneratedPoints() {
        return generatedPoints;
    }
    
    public void setGeneratedPoints(List<NearRVPoint> points) {
        generatedPoints = points;
    }

    public List<Point> getDirtyCells(ExplorationImage image, RealAgent agent) {
        List<Point> dirtyCells = new LinkedList<Point>();
        // Erase old skeleton
        /*for(Point p: getSkeleton())
            dirtyCells.add(p);
        
        // Erase old RV points
        for(Point rv: getRVPoints()) 
            for(int i=Math.max(rv.x-4,0); i<=Math.min(rv.x+4,image.getWidth()-1); i++)
                for(int j=Math.max(rv.y-4,0); j<=Math.min(rv.y+4,image.getHeight()-1); j++)
                   dirtyCells.add(new Point(i,j));*/
        
        //Erase text over agents
        for(int i=agent.getX(); i<=agent.getX() + 100; i++)
            for(int j=agent.getY() - Constants.AGENT_RADIUS - 25; j<=agent.getY() - Constants.AGENT_RADIUS; j++)
                if(agent.getOccupancyGrid().locationExists(i,j))
                    agent.getDirtyCells().add(new Point(i,j));
        
        dirtyCells.addAll(curDirtyCells);
        if (sprDisplayData != null) {
            dirtyCells.addAll(sprDisplayData.getDirtyCells(image, agent));
        }
        
        curDirtyCells.clear();
        
        return dirtyCells;
    }

    public void drawCandidatePointInfo(ExplorationImage image) {
        if (sprDisplayData != null) {
            sprDisplayData.drawCandidatePointInfo(image);
        }
        
        image.setG2D();
        Graphics2D g2D = image.getG2D();
        
        for(Point p: generatedPoints) {
            g2D.setPaint(Color.MAGENTA);
            g2D.fillOval(p.x-2, p.y-2, 4, 4);
            
            for (int i = p.x-2; i < p.x+2; i++)
                for (int j = p.y-2; j < p.y+2; j++)
                    curDirtyCells.add(new Point(i,j));
        }
        
        int counter = 0;
        while (!pointsNearFrontierReal.isEmpty()) {
            NearRVPoint p = pointsNearFrontierReal.poll();
            Color c = Color.cyan;
            if (counter == 0) c = Color.RED;
            if (counter == 1) c = Color.GREEN;
            if (counter == 2) c = Color.BLUE;
            g2D.setPaint(c);
            g2D.fillOval(p.x-2, p.y-2, 4, 4);
            
            for (int i = p.x-2; i < p.x+2; i++)
                for (int j = p.y-2; j < p.y+2; j++)
                    curDirtyCells.add(new Point(i,j));
            
            if (counter < 3 && p.commLinkClosestToBase != null) {
                NearRVPoint p1 = p.commLinkClosestToBase.getRemotePoint();
                g2D.fillRect(p1.x-2, p1.y-2, 4, 4);
                
                for (int i = p1.x-2; i < p1.x+2; i++)
                    for (int j = p1.y-2; j < p1.y+2; j++)
                        curDirtyCells.add(new Point(i,j));
                
                image.drawLine(p, p1, c);
                
                curDirtyCells.addAll(image.pointsAlongSegment(p, p1));
                
                NearRVPoint p2 = p1.parentPoint;
                g2D.drawOval(p2.x-2, p2.y-2, 4, 4);
                
                for (int i = p2.x-2; i < p2.x+2; i++)
                    for (int j = p2.y-2; j < p2.y+2; j++)
                        curDirtyCells.add(new Point(i,j));
                
                image.drawLine(p1, p2, c);
                
                curDirtyCells.addAll(image.pointsAlongSegment(p1, p2));
                
                g2D.setPaint(Color.BLACK);
                g2D.drawString(Double.toString(Math.round(p.utility)), p.x - 5, p.y - 5);
                
                for (int i = p.x-5; i < p.x+50; i++)
                    for (int j = p.y-5; j < p.y+25; j++)
                        curDirtyCells.add(new Point(i,j));
                counter++;
            }
            
        }
        //g2D.setPaint(Color.RED);
        //g2D.drawOval(frontierCenter.x-5, frontierCenter.y-5, 10, 10);
    }

    public void drawRendezvousLocation(ExplorationImage image, RealAgent agent) {
        if (sprDisplayData != null) {
            sprDisplayData.drawRendezvousLocation(image, agent);
        }
        
        int x,y;
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        // Draw Child RV
        try{
            x = (int)rvd.getChildRendezvous().getParentLocation().getX();
            y = (int)rvd.getChildRendezvous().getParentLocation().getY();
            image.drawPoint(x, y, Constants.MapColor.childRV());
            for(int i=Math.max(0,x-4); i<=Math.min(x+4,image.getWidth()-1); i++)
                for(int j=Math.max(0,y-4); j<=Math.min(y+4,image.getHeight()-1); j++)
                    agent.getDirtyCells().add(new Point(i,j));
            image.drawText("c:" + rvd.getChildRendezvous().getTimeMeeting() + ":" + rvd.getChildRendezvous().getTimeWait(), 
                    agent.getLocation().x, agent.getLocation().y - 10, Constants.MapColor.text());
            
        }
        catch(java.lang.NullPointerException e) {
        }
        
        // Draw Parent RV
        try{
            x = (int)rvd.getParentRendezvous().getChildLocation().getX();
            y = (int)rvd.getParentRendezvous().getChildLocation().getY();
            image.drawPoint(x, y, Constants.MapColor.parentRV());
            for(int i=Math.max(0,x-4); i<=Math.min(x+4,image.getWidth()-1); i++)
                for(int j=Math.max(0,y-4); j<=Math.min(y+4,image.getHeight()-1); j++)
                    agent.getDirtyCells().add(new Point(i,j));
            image.drawText("p:" + rvd.getParentRendezvous().getTimeMeeting() + ":" + rvd.getParentRendezvous().getTimeWait(), 
                    agent.getLocation().x, agent.getLocation().y - 20, Constants.MapColor.text());
        }
        catch(java.lang.NullPointerException e) {
        }
    }
    
}
