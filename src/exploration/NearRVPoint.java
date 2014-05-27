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
package exploration;

import config.Constants;
import java.awt.Point;

/**
 *
 * @author julh
 */
    public class NearRVPoint extends Point implements Comparable<NearRVPoint> {
        public double distanceToFrontier;
        public int degree;
        public double utility;
        
        public NearRVPoint (int newX, int newY, double d) {
            this.x = newX;
            this.y = newY;
            this.distanceToFrontier = d;
            this.degree = 1;
            this.utility = 1 / d * 10000;
        }

        public NearRVPoint (int newX, int newY, double d, int dg) {
            this.x = newX;
            this.y = newY;
            this.distanceToFrontier = d;
            this.degree = dg;
            this.utility = Math.pow(dg, 2) / d * 10000;
            //System.out.println(Constants.INDENT + "Point at " + x + "," + y + " has utility " + (int)utility);
        }

        
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



