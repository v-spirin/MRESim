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
package communication;

import agents.RealAgent;
import agents.TeammateAgent;
import exploration.rendezvous.Rendezvous;
import exploration.rendezvous.RendezvousAgentData;

/**
 *
 * @author Victor
 */
public class RendezvousDataMessage implements IDataMessage {

    public final Rendezvous parentRendezvous;
    public final Rendezvous childRendezvous;
    public final Rendezvous parentBackupRendezvous;
    public final Rendezvous childBackupRendezvous;

    public RendezvousDataMessage(RendezvousAgentData data) {
        if (data.getParentRendezvous() != null) {
            parentRendezvous = data.getParentRendezvous().copy();
        } else {
            parentRendezvous = null;
        }
        if (data.getChildRendezvous() != null) {
            childRendezvous = data.getChildRendezvous().copy();
        } else {
            childRendezvous = null;
        }
        if (data.getParentBackupRendezvous() != null) {
            parentBackupRendezvous = data.getParentBackupRendezvous().copy();
        } else {
            parentBackupRendezvous = null;
        }
        if (data.getChildBackupRendezvous() != null) {
            childBackupRendezvous = data.getChildBackupRendezvous().copy();
        } else {
            childBackupRendezvous = null;
        }

    }

    @Override
    public void receiveMessage(RealAgent agent, TeammateAgent teammate) {
        RendezvousAgentData rvd = agent.getRendezvousAgentData();

        teammate.getRendezvousAgentData().setChildRendezvous(childRendezvous);
        teammate.getRendezvousAgentData().setParentRendezvous(parentRendezvous);

        //if the message is from our child
        /*if (teammate.getID() == agent.getChild() && teammate.getID() != Constants.BASE_STATION_TEAMMATE_ID) {
            rvd.setChildRendezvous(parentRendezvous);
            rvd.setChildBackupRendezvous(parentBackupRendezvous);
            if (parentRendezvous.parentsRVLocation != null) {
                rvd.setParentRendezvous(parentRendezvous.parentsRVLocation);
            }
        }*/
    }
}
