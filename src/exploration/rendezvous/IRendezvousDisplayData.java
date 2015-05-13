/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package exploration.rendezvous;

import agents.RealAgent;
import gui.ExplorationImage;
import java.awt.Point;
import java.util.List;

/**
 *
 * @author Victor
 */
public interface IRendezvousDisplayData {
    List<Point> getDirtyCells(ExplorationImage image, RealAgent agent);
    void drawCandidatePointInfo(ExplorationImage image);
    void drawRendezvousLocation(ExplorationImage image, RealAgent agent);
}
