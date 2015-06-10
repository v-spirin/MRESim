/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package exploration.rendezvous;

import agents.RealAgent;
import java.awt.Point;

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
    void processGoToChildReplan();
    
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
