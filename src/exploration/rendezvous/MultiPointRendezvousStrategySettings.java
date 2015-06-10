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
public class MultiPointRendezvousStrategySettings {
    //if enabled, agents will try to move to area that they estimate will have better signal strength with the
    //other agent rendezvous position, while they are waiting for that agent
    public boolean moveToBetterCommsWhileWaiting;
    public double SamplePointDensity;
    public boolean replanOurMeetingPoint;
}
