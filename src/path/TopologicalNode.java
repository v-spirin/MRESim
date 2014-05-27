/*
 *     Copyright 2010, 2014 Julian de Hoog (julian@dehoog.ca), Victor Spirin (victor.spirin@cs.ox.ac.uk)
 *
 *     This file is part of MRESim 2.2, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our paper:
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
package path;

import java.awt.Point;
import java.util.LinkedList;

/**
 *
 * @author Victor
 */
public class TopologicalNode {
    private int ID;
    private Point position;
    private LinkedList<TopologicalNode> neighbours;
    private LinkedList<Path> neighbour_paths;
    
    public TopologicalNode(int ID, Point position)
    {
        this.ID = ID;
        this.position = position;
        neighbours = new LinkedList<TopologicalNode>();
        neighbour_paths = new LinkedList<Path>();
    }
    
    public int getID()
    {
        return ID;
    }
    
    public void setID(int ID)
    {
        this.ID = ID;
    }
    
    public Point getPosition()
    {
        return position;
    }
    
    public void addNeighbour(TopologicalNode neighbour, Path path)
    {
        if (!neighbours.contains(neighbour))
        {
            neighbours.add(neighbour);
            neighbour_paths.add(path);
        }
    }
    
    public LinkedList<TopologicalNode> getListOfNeighbours()
    {
        return neighbours;
    }
    public Path getPathToNeighbour(TopologicalNode neighbour)
    {
        try {
            int index = neighbours.indexOf(neighbour);
            return neighbour_paths.get(index);
        } catch (IndexOutOfBoundsException ex)
        {
            return null;
        }
    }
}
