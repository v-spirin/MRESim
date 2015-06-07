/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
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

    public List<Point> getDirtyCells(ExplorationImage image, RealAgent agent) {
        List<Point> dirtyCells = new LinkedList<Point>();
        // Erase old skeleton
        for(Point p: getSkeleton())
            dirtyCells.add(p);
        
        // Erase old RV points
        for(Point rv: getRVPoints()) 
            for(int i=Math.max(rv.x-4,0); i<=Math.min(rv.x+4,image.getWidth()-1); i++)
                for(int j=Math.max(rv.y-4,0); j<=Math.min(rv.y+4,image.getHeight()-1); j++)
                   dirtyCells.add(new Point(i,j));
        
        //Erase text over agents
        for(int i=agent.getX(); i<=agent.getX() + 100; i++)
            for(int j=agent.getY() - Constants.AGENT_RADIUS - 25; j<=agent.getY() - Constants.AGENT_RADIUS; j++)
                if(agent.getOccupancyGrid().locationExists(i,j))
                    agent.getDirtyCells().add(new Point(i,j));
        
        return dirtyCells;
    }

    public void drawCandidatePointInfo(ExplorationImage image) {
        try{
            for(Point p: getSkeleton())
                image.setPixel(p.x, p.y, Constants.MapColor.skeleton());
            
            for(Point p: getRVPoints())
                image.drawPoint(p.x, p.y, Constants.MapColor.rvPoints());
        }
        catch(java.lang.NullPointerException e) {
        }
    }

    public void drawRendezvousLocation(ExplorationImage image, RealAgent agent) {
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
