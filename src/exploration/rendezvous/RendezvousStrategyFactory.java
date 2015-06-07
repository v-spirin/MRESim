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
        SinglePointRendezvousStrategySettings rvSettings = new SinglePointRendezvousStrategySettings();
        rvSettings.allowReplanning = simConfig.replanningAllowed();
        rvSettings.useImprovedRendezvous = simConfig.useImprovedRendezvous();
        IRendezvousStrategy rendezvousStrategy = new SinglePointRendezvousStrategy(agent, rvSettings);
        return rendezvousStrategy;
    }
}
