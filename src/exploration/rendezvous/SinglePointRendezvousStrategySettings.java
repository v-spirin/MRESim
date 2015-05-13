/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package exploration.rendezvous;

/**
 *
 * @author Victor
 */
public class SinglePointRendezvousStrategySettings {
    //If false, new RV point is simply the point where explorer turns back to head to RV.
    //Otherwise, we try to pick a better spot (near junctions, in corridors, etc.
    public boolean useImprovedRendezvous;
    //If one of the agents doesn't make it to rendezvous, should we try to head to a secondary location?
    public boolean allowReplanning;
    //When calculating time to RV, should we assume that relay has to reach base station location, 
    //or within range radius of communication range of base station?
    public boolean useSimpleCircleCommModelForBaseRange;
    //Should we explicitly make sure explorer has some time to spend exploring the frontier?
    public boolean giveExplorerMinTimeNearFrontier;
}
