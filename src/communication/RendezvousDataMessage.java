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
        RendezvousAgentData rvd = agent.getRendezvousAgentData();
        
        teammate.getRendezvousAgentData().setChildRendezvous(childRendezvous);
        teammate.getRendezvousAgentData().setParentRendezvous(parentRendezvous);
        
        System.out.println(agent + " comms setting teammate " + teammate.getName() + 
                " childRV to " + childRendezvous + ", parentRV to " + parentRendezvous);
        
        //if the message is from our child
        if(teammate.getID() == agent.getChild() && teammate.getID() != Constants.BASE_STATION_TEAMMATE_ID) {
            rvd.setChildRendezvous(parentRendezvous);
            rvd.setChildBackupRendezvous(parentBackupRendezvous);
            System.out.println(agent + " comms setting own childRV to " + parentRendezvous + ", childBackupRV to " + parentBackupRendezvous);
            if (parentRendezvous.parentsRVLocation != null) {
                System.out.println(agent + " comms setting own parentRV loc to " + parentRendezvous.parentsRVLocation);
                rvd.setParentRendezvous(parentRendezvous.parentsRVLocation);
            }
        }
    }
}
