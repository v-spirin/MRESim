/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package communication;

import agents.RealAgent;
import agents.TeammateAgent;
import config.Constants;
import exploration.rendezvous.Rendezvous;
import exploration.rendezvous.RendezvousAgentData;

/**
 *
 * @author Victor
 */
public class RendezvousDataMessage implements IDataMessage{
    public final Rendezvous parentRendezvous;
    public final Rendezvous childRendezvous;
    public final Rendezvous parentBackupRendezvous;
    public final Rendezvous childBackupRendezvous;
    
    public RendezvousDataMessage(RendezvousAgentData data) {
        if (data.getParentRendezvous() != null)
            parentRendezvous = data.getParentRendezvous().copy();
        else
            parentRendezvous = null;
        if (data.getChildRendezvous() != null)
            childRendezvous = data.getChildRendezvous().copy();
        else
            childRendezvous = null;
        if (data.getParentBackupRendezvous() != null)
            parentBackupRendezvous = data.getParentBackupRendezvous().copy();
        else
            parentBackupRendezvous = null;
        if (data.getChildBackupRendezvous() != null)
            childBackupRendezvous = data.getChildBackupRendezvous().copy();
        else
            childBackupRendezvous = null;
    }

    public void receiveMessage(RealAgent agent, TeammateAgent teammate) {
        teammate.getRendezvousAgentData().setChildRendezvous(childRendezvous);
        teammate.getRendezvousAgentData().setParentRendezvous(parentRendezvous);
        
        //if the message is from our child
        if(teammate.getID() == agent.getChild() && teammate.getID() != Constants.BASE_STATION_TEAMMATE_ID) {
            agent.getRendezvousAgentData().setChildRendezvous(parentRendezvous);
            agent.getRendezvousAgentData().setChildBackupRendezvous(parentBackupRendezvous);
            if (parentRendezvous.parentsRVLocation != null)
                agent.getRendezvousAgentData().setParentRendezvous(parentRendezvous.parentsRVLocation);
        }
    }
}
