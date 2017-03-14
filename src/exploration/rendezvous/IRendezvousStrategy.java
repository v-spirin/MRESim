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
import java.awt.Point;
import path.Path;

/**
 *
 * @author Victor
 */
public interface IRendezvousStrategy {

    /**
     * Generally this method should not be called directly, and may even need to be private.
     * This method must set the following agent parameters:
     *   - Rendezvous
     *     -- Parent rendezvous point
     *     -- Child rendezvous point
     *     -- Time at which rendezvous happens
     *     -- Wait time (time at which rendezvous is called off or backup rendezvous is used)
     *     -- Backup rendezvous location
     *     -- Time at which backup rendezvous happens
     *     -- Time at which backup rendezvous is called off
     *   - Parent teammate rendezvous location (parentRenedzvous.parentsRVLocation)
     * i.e., where our relay will communicate with base station
     * @param agent - child, calculates rendezvous for itself and the parent
     */
    void calculateRendezvousExplorerWithRelay();
    
    //This method not currently implemented anywhere, as tree depth greater than 2 seems to be unnecessary
    void calculateRendezvousRelayWithRelay();
    
    //This method is called at the time step when explorer stops exploring and goes back to RV
    void processExplorerStartsHeadingToRV();
    
    //In the below two methods we can recompute our part of the rendezvous location, e.g. to attempt to intercept 
    //the other agent we are meeting, or to find a better location within comms range of where the other agent will
    //be waiting for us.
    //This method is called when we are replanning path to parent in ReturnToParent state
    void processReturnToParentReplan();
    //This method called just before we check if we are due to return to RV. We can change the meeting point for us here.
    void processExplorerCheckDueReturnToRV();
    //This method is called when we are replanning path to child in GoToChild state
    Path processGoToChildReplan();
    
    //This method is called when we are waiting for parent to arrive at RV. We move to the point returned by the method.
    Point processWaitForParent();
    //This method is called when we are waiting for child to arrive at RV. We move to the point returned by the method.
    Point processWaitForChild();
    //This method is called if the child didn't turn up to meeting and we didn't agree on a backup (or we are already
    //at a backup. Usually we either just wait forever, or turn into an explorer.
    Point processWaitForChildTimeoutNoBackup();
    
    //This method is called when we just got into parent range and are in GiveParentInfo state.
    //Usually we would calculate next RV location here and communicate it to parent in the following timestep.
    void processJustGotIntoParentRange();
    
    void processAfterGiveParentInfoExplorer();
    void processAfterGiveParentInfoRelay();
    void processAfterGetInfoFromChild();
    
    IRendezvousDisplayData getRendezvousDisplayData();
    
    RealAgent getAgent();
    void setAgent(RealAgent ag);
}
