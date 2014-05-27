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
package agents;

/**
 *
 * @author Victor
 */
public class UtilityCalculator {
    public static double calculateComStationUtility(double informationGain, int timeOfAcquiring, int current_time)
    {
        return calculateComStationUtility(informationGain, timeOfAcquiring, current_time, 1000000);
    }
    
    public static double calculateComStationUtility(double informationGain, int timeOfAcquiring, int current_time, int N)
    {
        //int change_limit = 500;
        //current_time += timeOfAcquiring;
        int time_limit = N;//current_time * 100; //after this limit, utility gets negative
        //if (current_time > change_limit) time_limit = current_time * 750;
        //double scaling_factor = 1 - (double)timeOfAcquiring / time_limit; // utility diminishes linearly over time
        double scaling_factor = -Math.pow((double)(timeOfAcquiring)/1000, 6);
        //if (scaling_factor < 0) scaling_factor = 0;
       return (informationGain * scaling_factor);
        //return (int) (informationGain * (-Math.exp((double)timeOfAcquiring / (double)time_limit)));
    }
}
