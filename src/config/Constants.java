/*
 *     Copyright 2010, 2015 Julian de Hoog (julian@dehoog.ca), Victor Spirin (victor.spirin@cs.ox.ac.uk)
 *
 *     This file is part of MRESim 2.2, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our papers:
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
package config;

import java.awt.Color;
import java.io.File;
import java.nio.file.Paths;

/**
 *
 * @author julh
 */
public class Constants {
    
    // Max number of rows in environment
    public static final int MAX_ROWS = 600;
    
    // Max number of columns in enviroment
    public static final int MAX_COLS = 800;
    
    // Random seed for random walk (set to a constant for testing, otherwise to System.currentTimeMillis())
    public static final int RANDOM_SEED = (int)System.currentTimeMillis();
    
    // Max time to search for a path, in ms
    public static final int MAX_PATH_SEARCH_TIME = 500;
    
    // Size of relay in image
    public static final int AGENT_RADIUS = 4; // default 4;
    
    // Distance that the grid is partitioned into for A* path planning
    public static final int STEP_SIZE = 1; // default 3;
    
    // Target ratio of info known at base to total info known at agents
    //public static final double TARGET_INFO_RATIO = 1; 
    
    // default agent speed
    public static final int DEFAULT_SPEED = 10;
    
    // Safe distance for each relay's Safe Range, percentage of Free Space Range
    public static final int SAFE_RANGE = 10;
    
    // How thick should the agent mark the obstacles/walls in the OccGrid
    public static final int WALL_THICKNESS = 3; //was 3
    
    // How often should we calculate area known by all the agents? (Takes around 200ms)
    public static final int RECALC_JOINT_AREA = 10;
    
    // Initial delay of timer
    public static final int INIT_DELAY = 500;
    
    // Replan every ... steps
    public static final int REPLAN_INTERVAL = 15;
    
    // Min time between RV replan
    public static final int RV_REPLAN_INTERVAL = REPLAN_INTERVAL;
    
    // Difference in milliseconds between increments in simulation rate slider bar
    public static final int TIME_INCREMENT = 111;
    
    // Maximum proximity an relay may have to a wall, in pixels, when planning paths
    // (May cause bugs if less than step size)
    public static final int WALL_DISTANCE = 3; // default 3;
    
    // How many neighbours are added to the queue for AStar search when planning paths
    public static final int PATH_SEARCH_RESOLUTION = 12;
    
    // Maximum number of nodes to examine for path planner
    public static final int MAX_PATH_PLANNER_ITERATIONS = Integer.MAX_VALUE;
    
    // Number of frontiers (closest ones and biggest) to be evaluated when choosing a frontier
    public static final int MAX_NUM_FRONTIERS = 6;  //was 6
    
    // Minimum size a frontier must have to be considered as part of exploration
    public static final int MIN_FRONTIER_SIZE = 3; // default 20; was 3
    
    // How long should we assume that the teammate is exploring the frontier that he last told us he was to explore?
    public static final int REMEMBER_TEAMMATE_FRONTIER_PERIOD = 500;
    
    // Minimum wait time until attempting to communicate with the Base Station again. This is to ensure when in range
    // with the base station we do not try to transfer big maps every step, resulting in slowdown.
    public static final int MIN_COMM_WITH_CS_PERIOD = 10;
    
    // Probability of going out of service at any given time
    public static final double PROB_OUT_OF_SERVICE = 0.002;
    
    // Maximum possible time (for divisions by zero speed)
    public static final int MAX_TIME = 10000;
    
    // Percent of a territory that must be explored
    public static final double TERRITORY_PERCENT_EXPLORED_GOAL = 0.95;
    
    // Probability of new debris at each time step
    public static final double NEW_DEBRIS_LIKELIHOOD = 0.5;
    
    // Maximum size of new debris
    public static final int NEW_DEBRIS_MAX_SIZE = 50;
    
    // How often agents should recalculate how much they know, how much they are relaying etc.
    public static final int UPDATE_AGENT_KNOWLEDGE_INTERVAL = 1;
    
    // Unexplored topological space ID
    public static final int UNEXPLORED_NODE_ID = Integer.MAX_VALUE;
    
    // Time an agent needs to be in a state, before he starts communicating with the parent for RoleBasedExploration
    public static final int MIN_TIME_IN_EXPLORE_STATE = 15;
    
    public static final int BASE_STATION_TEAMMATE_ID = 1;
    public static final int BASE_STATION_AGENT_ID = 0;
    
    // How often should we check if it's time to RV?
    public static final int CHECK_INTERVAL_TIME_TO_RV = 2;
    
    // How often should we recalculate path to parent
    public static final int PATH_RECALC_PARENT_INTERVAL = 8;
    public static final int PATH_RECALC_CHILD_INTERVAL = 8;
    
    // How long should we wait at RV, before we make alternative arrangements
    public static final int WAIT_AT_RV_BEFORE_REPLAN = 60;
    
    // Minimal time an explorer should explore a frontier before delivering the information back
    public static final int FRONTIER_MIN_EXPLORE_TIME = 75;
    
    //In role-based exploration try not to go to frontiers if we will have to turn back to RV before we can even reach the frontier.
    public static final boolean AVOID_FRONTIERS_WE_CANNOT_REACH_IN_TIME = true;
    
    // How often should we check if we need to rebuild topological path?
    public static final int REBUILD_TOPOLOGICAL_MAP_INTERVAL = REPLAN_INTERVAL;
    
    // How often MUST we rebuild topological map?
    public static final int MUST_REBUILD_TOPOLOGICAL_MAP_INTERVAL = REPLAN_INTERVAL*10;
    
    // How many cells in the occupancy grid need to change for us to rebuild topological map
    public static final int MAP_CHANGED_THRESHOLD = 100;
    
    // How many steps should we initialize for
    public static final int INIT_CYCLES = 3;
    
    // How much better should RV through a Wall be, compared to RV from the same spot not through a wall, to be accepted
    public static final double MIN_RV_THROUGH_WALL_ACCEPT_RATIO = 0.8;
    
    // Maximum time we're allowed to search for distance by skeleton, in ms
    public static final long MAX_TIME_DISTANCE_BY_SKELETON = 100;
    
    public static final boolean OUTPUT_PATH_ERROR = false;
    public static final String DEFAULT_PATH_LOG_DIRECTORY = Paths.get(System.getProperty("user.dir"), "patherror").toString() + File.separator;
    public static final String DEFAULT_LOG_DIRECTORY = Paths.get(System.getProperty("user.dir"), "logs").toString() + File.separator;
    public static final String DEFAULT_IMAGE_LOG_DIRECTORY = Paths.get(System.getProperty("user.dir"), "logs", "images").toString() + File.separator;

    // Colors to be used on map
    public static class MapColor {
        public static final Color background() {return Color.LIGHT_GRAY;}
        public static final Color explored() {return Color.WHITE;}
        public static final Color explored_base() {return Color.YELLOW;}
        public static final Color safe() {return Color.GREEN;}
        public static final Color unexplored() {return Color.LIGHT_GRAY;}
        public static final Color relay() {return Color.RED;}
        public static final Color explorer() {return Color.BLUE;} //{return new Color(238,118,33);}
        public static final Color comStation() {return Color.BLACK;}
        public static final Color sensed() {return Color.CYAN;}
        public static final Color comm() {return Color.BLUE;}
        public static final Color obstacle() {return Color.RED;}
        public static final Color barrier() {return Color.BLACK;}
        public static final Color slope() {return Color.YELLOW;}
        public static final Color hill() {return new Color(255,150,0);}
        public static final Color wall() {return Color.BLACK;}
        public static final Color text() {return Color.BLACK;}
        public static final Color link() {return Color.GREEN;}
        public static final Color frontier() {return Color.MAGENTA;}
        public static final Color test() {return Color.GREEN;}
        public static final Color skeleton() {return Color.BLACK;}
        public static final Color keyPoints() {return Color.LIGHT_GRAY;}
        public static final Color rvPoints() {return Color.RED;}
        public static final Color childRV() {return Color.ORANGE;}
        public static final Color parentRV() {return Color.GREEN;}
        public static final Color hierarchy() {return Color.MAGENTA;}
    }

    // Indent in console information
    public static final String INDENT = "    - ";
}
