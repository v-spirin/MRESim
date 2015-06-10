/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package exploration.rendezvous;

import agents.RealAgent;
import config.SimulatorConfig;

/**
 *
 * @author Victor
 */
public class RendezvousStrategyFactory {
    public static IRendezvousStrategy createRendezvousStrategy(SimulatorConfig simConfig, RealAgent agent) {
        if (!simConfig.RVThroughWallsEnabled()) {
            SinglePointRendezvousStrategySettings rvSettings = new SinglePointRendezvousStrategySettings();
            rvSettings.allowReplanning = simConfig.replanningAllowed();
            rvSettings.useImprovedRendezvous = simConfig.useImprovedRendezvous();
            IRendezvousStrategy rendezvousStrategy = new SinglePointRendezvousStrategy(agent, rvSettings);
            return rendezvousStrategy;
        } else {
            MultiPointRendezvousStrategySettings rvSettings = new MultiPointRendezvousStrategySettings();
            rvSettings.moveToBetterCommsWhileWaiting = true;
            rvSettings.SamplePointDensity = 400; //roughly every 20 sq. units
            rvSettings.replanOurMeetingPoint = false;
            IRendezvousStrategy rendezvousStrategy = new MultiPointRendezvousStrategy(agent, rvSettings);
            return rendezvousStrategy;
        }
    }
}
