/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package communication;

import agents.RealAgent;
import agents.TeammateAgent;

/**
 *
 * @author Victor
 */
public interface IDataMessage {
    //Message received from teammate to agent
    void receiveMessage(RealAgent agent, TeammateAgent teammate);
}
