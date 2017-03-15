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

import communication.CommLink;
import java.awt.Point;
import java.util.LinkedList;

/**
 *
 * @author julh
 */
    public class NearRVPoint extends Point implements Comparable<NearRVPoint> {
        public double distanceToFrontier;
        public double distanceToParent = java.lang.Double.MAX_VALUE;
        public double utility;
        public NearRVPoint parentPoint; //Point to which we need to head to enter comm range of our parent
        public CommLink commLinkClosestToBase; //Communication link from here to a point that is closest to the comm range of base
        
        public LinkedList<CommLink> commLinks = new LinkedList<CommLink>(); //connected points
        
        public NearRVPoint (int newX, int newY) {
            this.x = newX;
            this.y = newY;
            this.distanceToFrontier = 0;
            this.utility = 0;
        }
        
        public NearRVPoint (int newX, int newY, double utility) {
            this.x = newX;
            this.y = newY;
            this.utility = utility;
        }
        
        private double calcUtility() {
            return 10000 / this.distanceToFrontier;
        }
        
        public static double getFullRVUtility(double distToFrontier, double distToBase, int numObstacles) {
            double weight1 = 2.0;
            double weight2 = 1.0;
            double nonLineOfSightRiskFactor = 0.9;
            
            if (numObstacles == 0) nonLineOfSightRiskFactor = 1.0;
            
            return 10000 / (weight1 * distToFrontier + weight2 * distToBase) * nonLineOfSightRiskFactor;
        }
        
        public void setDistanceToFrontier(double d) {
            this.distanceToFrontier = d;
            this.utility = calcUtility();
        }
        
        @Override
        public int compareTo(NearRVPoint other) {
            if(other.utility > this.utility)
                return 1;
            else
                return -1;
        }
        
        public boolean equals(Point that)
        {
            return
                    this.x == that.x &&
                    this.y == that.y;
        }
    }



