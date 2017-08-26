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

import config.Constants;
import java.awt.Point;
import java.util.Objects;

/**
 * Rendezvous describes a rendezvous location, and includes information about where each agent
 * should head to.
 *
 * @author Victor
 */
public class Rendezvous {

    private Point childLocation; // where the child of the two agents meeting up should go
    private Point parentLocation; //where the parent of the two agents meeting up should go
    private int timeMeeting; //when they agree to meet up
    //private int minTimeMeeting; //time from which relay will be at the meeting point
    private int timeWait; //how long they agree to wait for partner at RV point
    public Rendezvous parentsRVLocation; //this is the RV location for our parent to meet with its parent (we calculate it).

    public Rendezvous(Point location) {
        assert (location != null);
        childLocation = location;
        parentLocation = location;
        timeMeeting = Constants.MAX_TIME; //meeting time not agreed
        timeWait = Constants.MAX_TIME; //wait indefinitely
        //minTimeMeeting = Constants.MAX_TIME;
    }

    public Rendezvous copy() {
        Rendezvous locCopy = new Rendezvous(childLocation);
        locCopy.setChildLocation(new Point(childLocation));
        locCopy.setParentLocation(new Point(parentLocation));
        locCopy.setTimeMeeting(timeMeeting);
        locCopy.setTimeWait(timeWait);
        //locCopy.setMinTimeMeeting(minTimeMeeting);
        if (parentsRVLocation != null) {
            locCopy.parentsRVLocation = parentsRVLocation.copy();
        }
        return locCopy;
    }

    @Override
    public boolean equals(Object that) {
        if (this == that) {
            return true;
        }
        if (!(that instanceof Rendezvous)) {
            return false;
        }
        Rendezvous other = (Rendezvous) that;
        return this.getChildLocation().equals(other.getChildLocation())
                && this.getParentLocation().equals(other.getParentLocation());
        //this.getTimeMeeting() == other.getTimeMeeting() &&
        //this.getTimeWait() == other.getTimeWait() &&
        //this.getMinTimeMeeting() == other.getMinTimeMeeting();

    }

    @Override
    public int hashCode() {
        int hash = 3;
        hash = 89 * hash + Objects.hashCode(this.childLocation);
        hash = 89 * hash + Objects.hashCode(this.parentLocation);
        hash = 89 * hash + this.timeMeeting;
        hash = 89 * hash + this.timeWait;
        hash = 89 * hash + Objects.hashCode(this.parentsRVLocation);
        return hash;
    }

    @Override
    public String toString() {
        return "parentLoc: (" + (int) parentLocation.getX() + "," + (int) parentLocation.getX() + "), childLoc: (" + (int) childLocation.getX() + "," + (int) childLocation.getX()
                + "), timeMeeting: " + timeMeeting + ", timeWait: " + timeWait;
    }

    public void setChildLocation(Point childLocation) {
        this.childLocation = childLocation;
    }

    public void setParentLocation(Point parentLocation) {
        this.parentLocation = parentLocation;
    }

    public void setTimeMeeting(int timeMeeting) {
        this.timeMeeting = timeMeeting;
    }

    public void setTimeWait(int timeWait) {
        this.timeWait = timeWait;
    }

    public Point getChildLocation() {
        return childLocation;
    }

    public Point getParentLocation() {
        return parentLocation;
    }

    public int getTimeMeeting() {
        return timeMeeting;
    }

    public int getTimeWait() {
        return timeWait;
    }
}
