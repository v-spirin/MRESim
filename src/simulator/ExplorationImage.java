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
package simulator;

import agents.RealAgent;
import agents.TeammateAgent;
import config.Constants;
import config.RobotConfig;
import config.RobotTeamConfig;
import config.SimulatorConfig;
import environment.Environment;
import environment.Frontier;
import environment.OccupancyGrid;
import environment.TopologicalMap;
import gui.MainGUI;
import gui.ShowSettings.ShowSettings;
import gui.ShowSettings.ShowSettingsAgent;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Rectangle2D;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.DataBuffer;
import java.awt.image.DataBufferByte;
import java.awt.image.IndexColorModel;
import java.awt.image.Raster;
import java.awt.image.SampleModel;
import java.awt.image.SinglePixelPackedSampleModel;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;
import java.util.PriorityQueue;
import javax.imageio.ImageIO;
import path.Path;

/**
 *
 * @author julh
 */
public class ExplorationImage {

    private int width;
    private int height;
    ColorModel colorModel;
    Graphics2D g2D;

    private BufferedImage image;

    public ExplorationImage(Environment env) {
        width = env.getColumns();
        height = env.getRows();
        colorModel = generate16ColorModel();
        resetImage();
    }

// <editor-fold defaultstate="collapsed" desc="Get and Set">
    public void resetSize(int newWidth, int newHeight) {
        width = newWidth;
        height = newHeight;

        resetImage();
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public Color getPixelColor(int row, int column) {
        if ((row < 0) || (column < 0)) {
            return Color.WHITE;
        }
        return new Color(image.getRGB(row, column));
    }

    public void setPixel(int row, int column, int color) {
        if ((row < 0) || (column < 0)) {
            return;
        }
        try {
            image.setRGB(row, column, color);
        } catch (ArrayIndexOutOfBoundsException e) {
            System.err.println(this.toString() + "Error: pixel out of image bounds (" + row + ", " + column + ")");
        }
    }

    public void setPixel(int row, int column, Color color) {
        setPixel(row, column, color.getRGB());
    }

    public void setPixel(int row, int column, Environment.Status status) {
        switch (status) {
            case unexplored:
                setPixel(row, column, Constants.MapColor.unexplored());
                break;
            case explored:
                setPixel(row, column, Constants.MapColor.explored());
                break;
            case obstacle:
                setPixel(row, column, Constants.MapColor.obstacle());
                break;
            default:
                break;
        }
    }

    public BufferedImage getImage() {
        return image;
    }

    public Graphics2D getG2D() {
        return g2D;
    }

    public void setG2D() {
        g2D = image.createGraphics();
    }

    public void setImage(String path) {
        try {
            image = ImageIO.read(new File(path));
        } catch (IOException e) {
        }
    }

    public final void resetImage() {
        byte[] pixels = generatePixels(width, height, new Rectangle2D.Float(0, 0, width, height));
        DataBuffer dbuf = new DataBufferByte(pixels, width * height, 0);
        int bitMasks[] = new int[]{(byte) 0xf};
        SampleModel sampleModel = new SinglePixelPackedSampleModel(
                DataBuffer.TYPE_BYTE, width, height, bitMasks);
        WritableRaster raster = Raster.createWritableRaster(sampleModel, dbuf, null);
        image = new BufferedImage(colorModel, raster, false, null);

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                setPixel(i, j, Constants.MapColor.unexplored());
            }
        }
    }
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Dirt">
    private void addDirtyAgentCells(RealAgent agent) {
        // Erase old relay triangle
        for (int i = agent.getX() - 2 * Constants.AGENT_RADIUS; i <= agent.getX() + 2 * Constants.AGENT_RADIUS; i++) {
            for (int j = agent.getY() - 2 * Constants.AGENT_RADIUS; j <= agent.getY() + 2 * Constants.AGENT_RADIUS; j++) {
                if (agent.getOccupancyGrid().locationExists(i, j)) {
                    agent.getDirtyCells().add(new Point(i, j));
                }
            }
        }

        // Erase old relay name
        for (int i = agent.getX() + Constants.AGENT_RADIUS; i <= agent.getX() + Constants.AGENT_RADIUS + 46; i++) {
            for (int j = agent.getY() - Constants.AGENT_RADIUS - 12; j <= agent.getY() - Constants.AGENT_RADIUS + 12; j++) {
                if (agent.getOccupancyGrid().locationExists(i, j)) {
                    agent.getDirtyCells().add(new Point(i, j));
                }
            }
        }

        agent.getDirtyCells().addAll(
                agent.getRendezvousStrategy().getRendezvousDisplayData().getDirtyCells(this, agent));
    }

    private void addDirtyCommRangeCells(RealAgent agent) {
        for (Point cell : circlePoints(agent.getPrevX(), agent.getPrevY(), agent.getCommRange())) {
            if (agent.getOccupancyGrid().locationExists(cell.x, cell.y)) {
                agent.getDirtyCells().add(cell);
                //setPixel(cell.x, cell.y,Constants.MapColor.unexplored());
            }
        }
    }

    private LinkedList<Point> findAllDirt(RealAgent[] agents) {
        LinkedList<Point> allDirt = new LinkedList<Point>();
        for (RealAgent agent : agents) {
            allDirt = mergeLists(allDirt, agent.getDirtyCells());
        }

        return allDirt;
    }

    private void resetDirt(RealAgent[] agent) {
        for (RealAgent currAgent : agent) {
            currAgent.setDirtyCells(new LinkedList<Point>());
        }
    }

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Update Dirty and Full">
    public void dirtyUpdate(ShowSettings settings, ShowSettingsAgent[] agentSettings,
            Environment env, RealAgent[] agent, Polygon[] agentRange) {
        Update(settings, agentSettings, env, agent, agentRange, true);
    }

    public void fullUpdate(ShowSettings settings, ShowSettingsAgent[] agentSettings,
            Environment env, RealAgent[] agents, Polygon[] agentRange) {
        Update(settings, agentSettings, env, agents, agentRange, false);
    }

    public void fullUpdate(ShowSettings settings, ShowSettingsAgent agentSettings, RealAgent agent) {
        ShowSettingsAgent[] agentSettingsArray = new ShowSettingsAgent[1];
        agentSettingsArray[0] = agentSettings;

        RealAgent[] agents = new RealAgent[1];
        agents[0] = agent;

        settings.showEnv = false;
        agentSettingsArray[0].showCommRange = false;

        Update(settings, agentSettingsArray, null, agents, null, false);
    }

    //draws agent grid, start point and endpoint
    public void fullUpdatePath(OccupancyGrid agentGrid, Point startpoint, Point endpoint, ShowSettingsAgent agentSettings) {
        setG2D();

        //<editor-fold defaultstate="collapsed" desc="Draw agent grid according to agentSettings">
        for (int i = 0; i < agentGrid.width; i++) {
            for (int j = 0; j < agentGrid.height; j++) {
                setPixel(i, j, Constants.MapColor.background());
                updatePixelAgent(agentSettings, agentGrid, i, j);
            }
        }
        //</editor-fold>

        //drawLine(startpoint, endpoint, Color.yellow);
        setPixel(startpoint.x, startpoint.y, Color.GREEN);
        setPixel(endpoint.x, endpoint.y, Color.RED);
    }
    //</editor-fold>
    //draw RV generation process
/*public void fullUpdateRVPoints(OccupancyGrid agentGrid, PriorityQueue<NearRVPoint> rvPoints,
            LinkedList<NearRVPoint> generatedPoints, Point frontierCenter,
            ShowSettingsAgent agentSettings) {
        setG2D();

        //<editor-fold defaultstate="collapsed" desc="Draw agent grid according to agentSettings">
        for(int i=0; i<agentGrid.width; i++)
            for(int j=0; j<agentGrid.height; j++)
            {
                setPixel(i, j, Constants.MapColor.background());
                updatePixelAgent(agentSettings, agentGrid, i, j);
            }
        //</editor-fold>

        for(Point p: generatedPoints) {
            g2D.setPaint(Color.MAGENTA);
            g2D.fillOval(p.x-2, p.y-2, 4, 4);
        }

        int counter = 0;
        while (!rvPoints.isEmpty()) {
            NearRVPoint p = rvPoints.poll();
            Color c = Color.cyan;
            if (counter == 0) c = Color.RED;
            if (counter == 1) c = Color.GREEN;
            if (counter == 2) c = Color.BLUE;
            g2D.setPaint(c);
            g2D.fillOval(p.x-2, p.y-2, 4, 4);
            if (counter < 3) {
                NearRVPoint p1 = p.commLinkClosestToBase.getRemotePoint();
                g2D.fillRect(p1.x-2, p1.y-2, 4, 4);
                drawLine(p, p1, c);
                NearRVPoint p2 = p1.parentPoint;
                g2D.drawOval(p2.x-2, p2.y-2, 4, 4);
                drawLine(p1, p2, c);
                g2D.setPaint(Color.BLACK);
                g2D.drawString(Double.toString(Math.round(p.utility)), p.x - 5, p.y - 5);
            }
            counter++;
        }
        g2D.setPaint(Color.RED);
        g2D.drawOval(frontierCenter.x-5, frontierCenter.y-5, 10, 10);
    }*/

    //draws agent grid, topological map, start point and endpoint
    public void fullUpdatePath(OccupancyGrid agentGrid, TopologicalMap tMap, Point startpoint, Point endpoint, ShowSettingsAgent agentSettings) {
        setG2D();

        //<editor-fold defaultstate="collapsed" desc="Draw agent grid according to agentSettings">
        for (int i = 0; i < agentGrid.width; i++) {
            for (int j = 0; j < agentGrid.height; j++) {
                setPixel(i, j, Constants.MapColor.background());
                updatePixelAgent(agentSettings, agentGrid, i, j);
            }
        }
        //</editor-fold>

        drawKeyAreas(tMap);

        drawLine(startpoint, endpoint, Color.yellow);
        setPixel(startpoint.x, startpoint.y, Color.GREEN);
        setPixel(endpoint.x, endpoint.y, Color.RED);
    }

    private void Update(ShowSettings settings, ShowSettingsAgent[] agentSettings,
            Environment env, RealAgent[] agents, Polygon[] agentRange, boolean dirtOnly) {
        setG2D();

        //if(settings.showEnv)
        //    drawEnvironment(env.getFullStatus());
        if (!dirtOnly) {
            if (settings.showEnv) {
                drawEnvironment(env.getFullStatus());
            }

            for (int i = 0; i < width; i++) {
                for (int j = 0; j < height; j++) {
                    updatePixel(agentSettings, agents, i, j);
                }
            }
        } else {
            LinkedList<Point> allDirt = findAllDirt(agents);

            if (settings.showEnv) {
                drawEnvironment(env.getFullStatus(), allDirt);
            }

            allDirt.parallelStream().forEach((currCell) -> {
                updatePixel(agentSettings, agents, currCell.x, currCell.y);
            });

            setPixel(1, 1, Color.MAGENTA);
            setPixel(1, 2, Color.MAGENTA);
            setPixel(2, 1, Color.MAGENTA);
            setPixel(2, 2, Color.MAGENTA);
        }

        resetDirt(agents);

        if (settings.showEnv) //UUUGGGLLLYYYYY Fix for crazy Java-Color Crap
        {
            drawEnvironmentAgain(env.getFullStatus());
        }

        if (settings.showHierarchy) {
            drawHierarchy(agents);
        }

        if (settings.showConnections) {
            drawConnections(agents);
        }

        for (int i = 0; i < agents.length; i++) {

            //Draw skeleton
            if (agentSettings[i].showRVCandidatePointInfo) {
                drawRVSelectionInfo(agents[i]);
            }
            //(agents[i].getTopologicalMap());
            //Update agents
            if (agentSettings[i].showAgent) {
                updateAgent(agents[i]);
            }

            //Update comm ranges
            if (agentSettings[i].showCommRange) {
                updateCommRange(env, agents[i], agentRange[i]);
            }

            //Draw frontiers
            if (agentSettings[i].showFrontiers) {
                drawFrontierOutlines(agents[i]);
            }

            //Draw paths
            if (agentSettings[i].showPath) {
                drawPath(agents[i].getPath(), agents[i].pathTaken);
                agents[i].updatePathDirt();
            }
            //Draw skeleton
            //if(agentSettings[i].showSkeleton)
            //drawSkeleton(agents[i]);
            //Draw border skeleton
            if (agentSettings[i].showBorderSkel) {
                drawBorderSkeleton(agents[i]);
            }
            //Draw border skeleton
            if (agentSettings[i].showRVWalls) {
                drawRVWalls(agents[i]);
            }
        }

        // separate loop to have rendezvous points on top
        for (int i = 0; i < agents.length; i++) {
            //Draw rendezvous
            if (agentSettings[i].showRendezvous) {
                drawRendezvous(agents[i]);
            }
        }
    }

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Save screenshot">
    public void saveScreenshot(String dirName, int timeElapsed) {
        try {
            ImageIO.write(image, "png", new File(dirName + "explorationImage " + timeElapsed + ".png"));
        } catch (IOException e) {
            System.err.println(this.toString() + "Screenshot saving -- Error writing to file!" + e);
        }
    }

    public void saveScreenshot(String dirName) {
        SimpleDateFormat fmt = new SimpleDateFormat("yyyyMMdd_HHmmssS8");
        String filename = fmt.format(new Date());
        try {
            ImageIO.write(image, "png", new File(dirName + "\\" + filename + ".png"));
            if (Constants.DEBUG_OUTPUT) {
                System.out.println("Filename is: " + filename + ".png");
            }
        } catch (IOException e) {
            System.err.println(this.toString() + "Screenshot saving -- Error writing to file!" + e);
        }
    }

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Update functions">
    //<editor-fold defaultstate="collapsed" desc="updatePixel">
    private void updatePixel(ShowSettingsAgent[] agentsSettings, RealAgent[] agents, int xCoord, int yCoord) {
        if ((xCoord < 0) || (yCoord < 0)) {
            return;
        }
        //setPixel(xCoord, yCoord, Constants.MapColor.background());

        for (int i = agents.length - 1; i >= 0; i--) {
            agentsSettings[i].showBaseSpace = (agents[i].getRole() == RobotConfig.roletype.BaseStation) && agentsSettings[i].showFreeSpace;
            updatePixelAgent(agentsSettings[i], agents[i].getOccupancyGrid(), xCoord, yCoord);
        }
    }

    private void updatePixelAgent(ShowSettingsAgent agentSettings, OccupancyGrid agentGrid, int xCoord, int yCoord) {

        if ((xCoord < 0) || (yCoord < 0)) {
            return;
        }
        if (agentSettings.showFreeSpace && agentGrid.freeSpaceAt(xCoord, yCoord)) {
            setPixel(xCoord, yCoord, Constants.MapColor.explored());
        }

        if (agentSettings.showBaseSpace && agentGrid.freeSpaceAt(xCoord, yCoord)) {
            setPixel(xCoord, yCoord, Constants.MapColor.explored_base());
        }

        //Update safe space
        if (agentSettings.showSafeSpace
                && agentGrid.safeSpaceAt(xCoord, yCoord)) {
            setPixel(xCoord, yCoord, Constants.MapColor.safe());
        }

        // Update obstacles
        if (agentSettings.showFreeSpace
                && agentGrid.obstacleAt(xCoord, yCoord)) {
            setPixel(xCoord, yCoord, Constants.MapColor.obstacle());
        }
    }
    //</editor-fold>

    public void redrawEnvAndAgents(MainGUI mainGUI, RobotTeamConfig rtc, SimulatorConfig simConfig) {
        if (mainGUI.showEnv()) {
            drawEnvironment(simConfig.getEnvironment().getFullStatus());
        }
        try {
            RobotConfig curr = new RobotConfig();
            for (int i = 0; i < rtc.getNumRobots(); i++) {
                if (mainGUI.getRobotPanel(i).showAgent()) {
                    curr = rtc.getRobotTeam().get(i + 1);
                    updateAgent(curr.getName(), curr.getStartX(), curr.getStartY(), curr.getStartHeading(), curr.getRole());
                }
                if (mainGUI.showHierarchy()) {
                    RobotConfig parent = rtc.getRobotTeam().get(curr.getParent());
                    for (Point p : pointsAlongThickSegment(new Point(curr.getStartX(), curr.getStartY()),
                            new Point(parent.getStartX(), parent.getStartY()))) {
                        setPixel(p.x, p.y, Constants.MapColor.hierarchy());
                    }
                }
            }
        } catch (NullPointerException npe) {
            System.err.println("Error: could not draw agents");
        }

    }

    public void updateBackground() {
        Rectangle2D.Float bg = new Rectangle2D.Float(0, 0, width, height);
        g2D.setPaint(Constants.MapColor.background());
        g2D.fill(bg);
        g2D.draw(bg);
    }

    public void updateAgent(RealAgent agent) {
        updateAgent(agent.getName(), agent.getX(), agent.getY(), agent.getHeading(), agent.getRole());
        // Add changed cells to relay's dirt
        addDirtyAgentCells(agent);
    }

    public void updateAgent(String name, int xLoc, int yLoc, double head, RobotConfig.roletype role) {
        if ((xLoc < 0) || (yLoc < 0)) {
            return;
        }
        Rectangle.Double tempRect;
        Polygon tempPoly;

        setG2D();

        switch (role) {
            case BaseStation:
                // Draw ComStation
                g2D.setPaint(Constants.MapColor.comStation());
                tempRect = new Rectangle2D.Double(xLoc - Constants.AGENT_RADIUS,
                        yLoc - Constants.AGENT_RADIUS,
                        Constants.AGENT_RADIUS * 2,
                        Constants.AGENT_RADIUS * 2);
                g2D.fill(tempRect);
                g2D.draw(tempRect);
                break;
            case RelayStation:
                // Draw ComStation
                g2D.setPaint(Constants.MapColor.comStation());
                tempRect = new Rectangle2D.Double(xLoc - Constants.AGENT_RADIUS,
                        yLoc - Constants.AGENT_RADIUS,
                        Constants.AGENT_RADIUS * 2,
                        Constants.AGENT_RADIUS * 2);
                g2D.fill(tempRect);
                g2D.draw(tempRect);
                break;
            default:
                if (role.equals(RobotConfig.roletype.Relay)) {
                    g2D.setPaint(Constants.MapColor.relay());
                } else {
                    g2D.setPaint(Constants.MapColor.explorer());
                }
                tempPoly = new Polygon();
                tempPoly.addPoint((int) (xLoc + 2 * Constants.AGENT_RADIUS * Math.cos((float) head)),
                        (int) (yLoc + 2 * Constants.AGENT_RADIUS * Math.sin((float) head)));
                tempPoly.addPoint((int) (xLoc + 2 * Constants.AGENT_RADIUS * Math.cos((float) head + (5 * Math.PI / 6))),
                        (int) (yLoc + 2 * Constants.AGENT_RADIUS * Math.sin((float) head + (5 * Math.PI / 6))));
                tempPoly.addPoint((int) (xLoc + 2 * Constants.AGENT_RADIUS * Math.cos((float) head - (5 * Math.PI / 6))),
                        (int) (yLoc + 2 * Constants.AGENT_RADIUS * Math.sin((float) head - (5 * Math.PI / 6))));
                g2D.fillPolygon(tempPoly);
                g2D.draw(tempPoly);
                break;
        }

        // Draw name
        g2D.setPaint(Constants.MapColor.text());
        g2D.drawString(name, xLoc + Constants.AGENT_RADIUS, yLoc - Constants.AGENT_RADIUS);

    }

    public void updateCommRange(Environment env, RealAgent agent, Polygon range) {
        //drawCircle(relay, Constants.MapColor.comm());

        //drawRange(env, agent, range, Constants.MapColor.comm());
        for (TeammateAgent teammate : agent.getAllTeammates().values()) {
            if (teammate.isInRange()) {
                for (Point p : pointsAlongSegment(agent.getLocation(), teammate.getLocation())) {
                    for (int i = p.x - 1; i <= p.x + 1; i++) {
                        for (int j = p.y - 1; j <= p.y + 1; j++) {
                            setPixel(i, j, Constants.MapColor.link());
                            agent.getDirtyCells().add(new Point(i, j));
                        }
                    }
                }
            }

        }

        /*Old: only circles
        if(relay.isCommunicating())
            drawCircle(relay, Constants.MapColor.link());
        else
            drawCircle(relay, Constants.MapColor.comm());*/
        // new:  full rnage
        if (agent.isCommunicating()) {
            drawRange(env, agent, range, Constants.MapColor.link());
        } else {
            drawRange(env, agent, range, Constants.MapColor.comm());
        }
    }

    public void drawFrontierOutlines(RealAgent agent) {
        PriorityQueue<Frontier> frontiers = agent.getFrontiers();
        if (frontiers == null) {
            return;
        }
        for (Frontier f : frontiers) {
            for (Point p : f.getPolygonOutline()) {
                setPixel(p.x, p.y, Constants.MapColor.frontier());
                agent.getDirtyCells().add(new Point(p.x, p.y));
            }
            for (int q = f.getCentre().x - 2; q < f.getCentre().x + 2; q++) {
                for (int j = f.getCentre().y - 2; j < f.getCentre().y + 2; j++) {
                    if ((q >= 0) && (j >= 0)) {
                        setPixel(q, j, Color.BLACK);
                        agent.getDirtyCells().add(new Point(q, j));
                    }
                }
            }
            for (int q = f.getCentre().x - 1; q < f.getCentre().x + 1; q++) {
                for (int j = f.getCentre().y - 1; j < f.getCentre().y + 1; j++) {
                    if ((q >= 0) && (j >= 0)) {
                        setPixel(q, j, Color.YELLOW);
                    }
                }
            }
        }
    }

    public void drawPath(Path futurePath, LinkedList<Point> pastPath) {
        try {
            // part 1: future path
            g2D.setPaint(Color.RED);
            java.util.List pts = futurePath.getPoints();
            for (int i = 0; i < pts.size() - 1; i++) {
                g2D.drawLine(((Point) pts.get(i)).x, ((Point) pts.get(i)).y, ((Point) pts.get(i + 1)).x, ((Point) pts.get(i + 1)).y);
            }

            //part 2:  past path
            /*g2D.setPaint(Color.BLUE);
            for(int i=0; i<pastPath.size()-1; i++)
                g2D.drawLine(((Point)pastPath.get(i)).x, ((Point)pastPath.get(i)).y, ((Point)pastPath.get(i+1)).x, ((Point)pastPath.get(i+1)).y);
             */
        } catch (java.lang.NullPointerException e) {
        }
    }

    public void drawRVSelectionInfo(RealAgent agent) {
        agent.getRendezvousStrategy().getRendezvousDisplayData().drawCandidatePointInfo(this);
    }

    public void drawKeyAreas(TopologicalMap tMap) {
        try {
            int[][] areaGrid = tMap.getAreaGrid();
            tMap.generateBorderPoints();
            LinkedList<Point> borderPoints = tMap.getBorderPoints();

            for (int i = 0; i < areaGrid.length; i++) {
                for (int j = 0; j < areaGrid[0].length; j++) {
                    if (areaGrid[i][j] > 0) {
                        int remainder = areaGrid[i][j] % 4;
                        Color c = Color.WHITE;
                        if (remainder == 0) {
                            c = Color.YELLOW;
                        }
                        if (remainder == 1) {
                            c = Color.CYAN;
                        }
                        if (remainder == 2) {
                            c = Color.GREEN;
                        }
                        if (remainder == 3) {
                            c = Color.MAGENTA;
                        }
                        if (remainder == 4) {
                            c = Color.MAGENTA;
                        }
                        if (remainder == 5) {
                            c = Color.RED;
                        }
                        if (remainder == 6) {
                            c = Color.CYAN;
                        }
                        if (areaGrid[i][j] == Constants.UNEXPLORED_NODE_ID) {
                            if (tMap.getGrid().freeSpaceAt(i, j)) {
                                c = Color.WHITE;
                            } else {
                                c = Color.GRAY;
                            }
                        }
                        if (tMap.getGrid().isFinalTopologicalMapCell(i, j)) {
                            c = Color.BLUE;
                        }
                        setPixel(i, j, c);
                    } //else setPixel(i, j, Color.RED);
                }
            }
            borderPoints.stream().forEach((p) -> {
                setPixel(p.x, p.y, Color.BLACK);
            });
        } catch (java.lang.NullPointerException e) {
        }
    }

    public void drawRendezvous(RealAgent agent) {
        agent.getRendezvousStrategy().getRendezvousDisplayData().drawRendezvousLocation(this, agent);
    }

    public void drawEnvironment(Environment.Status[][] status, LinkedList<Point> allDirt) {
        allDirt.parallelStream().forEach((p) -> {
            switch (status[p.x][p.y]) {
                case barrier:
                    setPixel(p.x, p.y, Constants.MapColor.wall());
                    break;
                case obstacle:
                    setPixel(p.x, p.y, Constants.MapColor.obstacle());
                    break;
                case slope:
                    setPixel(p.x, p.y, Constants.MapColor.slope());
                    break;
                case hill:
                    setPixel(p.x, p.y, Constants.MapColor.hill());
                    break;
                case explored:
                case unexplored:
                default:
                    setPixel(p.x, p.y, Constants.MapColor.background());
            }
        });

        /*for(Point p : allDirt){
                        switch(status[p.x][p.y]) {
                            case barrier: setPixel(p.x,p.y,Constants.MapColor.wall()); break;
                            case obstacle: setPixel(p.x,p.y,Constants.MapColor.obstacle()); break;
                            case slope: setPixel(p.x,p.y,Constants.MapColor.slope()); break;
                            case hill: setPixel(p.x,p.y,Constants.MapColor.hill()); break;
                            case explored:
                            case unexplored:
                            default:
                                setPixel(p.x,p.y, Constants.MapColor.background());
                        }
        }*/
    }

    public void drawEnvironment(Environment.Status[][] status) {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                switch (status[i][j]) {
                    case barrier:
                        setPixel(i, j, Constants.MapColor.wall());
                        break;
                    case obstacle:
                        setPixel(i, j, Constants.MapColor.obstacle());
                        break;
                    case slope:
                        setPixel(i, j, Constants.MapColor.slope());
                        break;
                    case hill:
                        setPixel(i, j, Constants.MapColor.hill());
                        break;
                    case explored:
                    case unexplored:
                    default:
                        setPixel(i, j, Constants.MapColor.background());
                }
            }
        }

    }

    public void drawEnvironmentAgain(Environment.Status[][] status) {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                switch (status[i][j]) {
                    case obstacle:
                        setPixel(i, j, Constants.MapColor.obstacle());
                        break;
                    case slope:
                        setPixel(i, j, Constants.MapColor.slope());
                        break;
                    case hill:
                        setPixel(i, j, Constants.MapColor.hill());
                        break;
                    default:
                }
            }
        }

    }

    public void drawHierarchy(RealAgent[] agent) {
        for (int i = 1; i < agent.length; i++) {
            for (RealAgent agent1 : agent) {
                if (agent1.getID() == agent[i].getParent()) {
                    for (Point p : pointsAlongThickSegment(agent[i].getLocation(), agent1.getLocation())) {
                        if (agent[i].getParentTeammate().isInRange()) {
                            setPixel(p.x, p.y, Constants.MapColor.link());
                        } else {
                            setPixel(p.x, p.y, Constants.MapColor.hierarchy());
                        }
                        agent[i].getDirtyCells().add(new Point(p.x, p.y));
                    }
                    break;
                }
            }
        }
    }

    public void drawBorderSkeleton(RealAgent agent) {
        try {
            if (agent.getTopologicalMap().getSkeletonPointsBorder() != null) {
                LinkedList<Point> newDirt = new LinkedList<Point>();
                for (Point p : agent.getTopologicalMap().getSkeletonPointsBorder()) {
                    setPixel(p.x, p.y, Constants.MapColor.skeleton());
                    newDirt.add(new Point(p.x, p.y));
                }

                for (Point p : agent.getTopologicalMap().getKeyPointsBorder()) {
                    drawPoint(p.x, p.y, Constants.MapColor.rvPoints());
                    mergeLists(newDirt, circlePoints(p.x, p.y, 4));
                    mergeLists(newDirt, circlePoints(p.x, p.y, 3));
                    mergeLists(newDirt, circlePoints(p.x, p.y, 2));
                    newDirt.add(p);
                }
                agent.addDirtyCells(newDirt);
            }
        } catch (java.lang.NullPointerException e) {
        }
    }

    public void drawRVWalls(RealAgent agent) {
        try {
            LinkedList<Point> keyPoints1 = agent.getTopologicalMap().getKeyPointsBorder();
            LinkedList<Point> keyPoints2 = agent.getTopologicalMap().getSecondKeyPointsBorder();
            LinkedList<Point> newDirt = new LinkedList<Point>();
            if ((keyPoints1 != null) && (keyPoints2 != null)) {
                //int counter = 0;
                for (int i = 0; i < keyPoints1.size(); i++) {
                    if (!keyPoints1.get(i).equals(keyPoints2.get(i))) {
                        //counter++;
                        for (Point p : pointsAlongSegment(keyPoints1.get(i), keyPoints2.get(i))) {
                            setPixel(p.x, p.y, Constants.MapColor.link());
                            newDirt.add(p);
                        }
                    }
                }
                agent.addDirtyCells(newDirt);
                //System.out.println("Total points drawn: " + counter);
            }
        } catch (java.lang.NullPointerException e) {
        }
    }

    public void drawConnections(RealAgent[] agent) {
        /*
            for(int i=0; i<agent.length; i++) {

            // TEMP SSRR presentation update comm link
            if(agent[i].getTimeLastCentralCommand() < 100) {
                for(Point p : pointsAlongSegment(agent[i].getLocation(), agent[0].getLocation())){

                    for(int q=p.x-1; q<=p.x+1; q++)
                        for(int w=p.y-1; w<=p.y+1; w++){
                            setPixel(q, w, Color.RED);
                            agent[i].getDirtyCells().add(new Point(q,w));
                        }
                }
            }
            }

         */
    }

// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Helper functions">
    public void drawRange(Environment env, RealAgent agent, Polygon range, Color color) {
        for (Point p : polygonPoints(range)) {
            if (env.locationExists(p.x, p.y)) {
                setPixel(p.x, p.y, color);
                agent.getDirtyCells().add(new Point(p.x, p.y));
            }
        }
    }

    public void drawCircle(RealAgent agent, Color color) {
        for (Point p : circlePoints(agent.getX(), agent.getY(), agent.getCommRange())) {
            if (agent.getOccupancyGrid().locationExists(p.x, p.y)) {
                setPixel(p.x, p.y, color);
                agent.getDirtyCells().add(new Point(p.x, p.y));
            }
        }
    }

    public void drawLine(Point start, Point end, Color color) {
        for (Point p : pointsAlongSegment(start, end)) {
            setPixel(p.x, p.y, color);
        }
    }

    public void drawText(String text, int x, int y, Color c) {
        setG2D();
        g2D.setPaint(c);
        //g2D.drawString(text, x, y);
    }

    public void drawPoint(int x, int y, Color c) {
        Shape innerCircle, outerCircle;

        outerCircle = new Ellipse2D.Float(x - 4, y - 4, 8, 8);
        g2D.setPaint(Color.BLACK);
        g2D.fill(outerCircle);
        g2D.draw(outerCircle);
        innerCircle = new Ellipse2D.Float(x - 3, y - 3, 6, 6);
        g2D.setPaint(c);
        g2D.fill(innerCircle);
        g2D.draw(innerCircle);
    }

    private LinkedList<Point> circlePoints(int centreX, int centreY, int radius) {
        LinkedList<Point> circleCells = new LinkedList();
        int currX, currY;

        for (double i = 0; i < 360; i += 0.5) {
            currX = (int) (centreX + radius * Math.cos(Math.PI / 180 * i));
            currY = (int) (centreY + radius * Math.sin(Math.PI / 180 * i));

            if (!circleCells.contains(new Point(currX, currY))) {
                circleCells.add(new Point(currX, currY));
            }
        }

        return circleCells;
    }

    private LinkedList<Point> polygonPoints(Polygon p) {
        Point p1, p2;
        LinkedList<Point> allPoints = new LinkedList<Point>();

        if (p == null) {
            return allPoints;
        }

        for (int i = 0; i < (p.npoints - 1); i++) {
            p1 = new Point(p.xpoints[i], p.ypoints[i]);
            p2 = new Point(p.xpoints[i + 1], p.ypoints[i + 1]);
            allPoints = mergeLists(allPoints, pointsAlongSegment(p1, p2));
        }

        return allPoints;
    }

    public LinkedList<Point> pointsAlongSegment(Point p1, Point p2) {
        LinkedList<Point> pts = new LinkedList<Point>();
        int x, y;
        double angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);

        for (int i = 0; i <= p1.distance(p2); i++) {
            x = p1.x + (int) (i * Math.cos(angle));
            y = p1.y + (int) (i * Math.sin(angle));
            if (!pts.contains(new Point(x, y))) {
                pts.add(new Point(x, y));
            }
        }

        if (!pts.contains(p2)) {
            pts.add(p2);
        }

        return pts;
    }

    private LinkedList<Point> pointsAlongThickSegment(Point p1, Point p2) {
        LinkedList<Point> pts = new LinkedList<Point>();
        int x, y;
        double angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);

        for (int i = 0; i <= p1.distance(p2); i++) {
            x = p1.x + (int) (i * Math.cos(angle));
            y = p1.y + (int) (i * Math.sin(angle));
            if (!pts.contains(new Point(x, y))) {
                pts.add(new Point(x, y));
            }
            if (x - 1 >= 0 && !pts.contains(new Point(x - 1, y))) {
                pts.add(new Point(x - 1, y));
            }
            if (y - 1 >= 0 && !pts.contains(new Point(x, y - 1))) {
                pts.add(new Point(x, y - 1));
            }
        }

        if (!pts.contains(p2)) {
            pts.add(p2);
        }

        return pts;
    }

    //Adds all points in list2 to list1 (duplicates allowed), returns merged list.
    public LinkedList<Point> mergeLists(LinkedList<Point> list1, LinkedList<Point> list2) {
        if (list1 == list2) {
            return list1;
        }
        list2.stream().forEach((p) -> {
            list1.add(p);
        });
        return list1;
    }

    private byte[] generatePixels(int w, int h, Rectangle2D.Float loc) {
        float xmin = loc.x;
        float ymin = loc.y;
        float xmax = loc.x + loc.width;
        float ymax = loc.y + loc.height;

        byte[] pixels = new byte[w * h];
        int pIx = 0;
        float[] p = new float[w];
        float q = ymin;
        float dp = (xmax - xmin) / w;
        float dq = (ymax - ymin) / h;

        p[0] = xmin;
        for (int i = 1; i < w; i++) {
            p[i] = p[i - 1] + dp;
        }

        for (int r = 0; r < h; r++) {
            for (int c = 0; c < w; c++) {
                int color = 1;
                float x = 0.0f;
                float y = 0.0f;
                float xsqr;
                float ysqr;
                do {
                    xsqr = x * x;
                    ysqr = y * y;
                    y = 2 * x * y + q;
                    x = xsqr - ysqr + p[c];
                    color++;
                } while (color < 512 && xsqr + ysqr < 4);
                pixels[pIx++] = (byte) (color % 16);
            }
            q += dq;
        }
        return pixels;
    }

    private static ColorModel generate16ColorModel() {
        // Generate 16-color model
        byte[] r = new byte[16];
        byte[] g = new byte[16];
        byte[] b = new byte[16];

        r[0] = 0;
        g[0] = 0;
        b[0] = 0;
        r[1] = 0;
        g[1] = 0;
        b[1] = (byte) 192;
        r[2] = 0;
        g[2] = 0;
        b[2] = (byte) 255;
        r[3] = 0;
        g[3] = (byte) 192;
        b[3] = 0;
        r[4] = 0;
        g[4] = (byte) 255;
        b[4] = 0;
        r[5] = 0;
        g[5] = (byte) 192;
        b[5] = (byte) 192;
        r[6] = 0;
        g[6] = (byte) 255;
        b[6] = (byte) 255;
        r[7] = (byte) 192;
        g[7] = 0;
        b[7] = 0;
        r[8] = (byte) 255;
        g[8] = 0;
        b[8] = 0;
        r[9] = (byte) 192;
        g[9] = 0;
        b[9] = (byte) 192;
        r[10] = (byte) 255;
        g[10] = 0;
        b[10] = (byte) 255;
        r[11] = (byte) 192;
        g[11] = (byte) 192;
        b[11] = 0;
        r[12] = (byte) 255;
        g[12] = (byte) 255;
        b[12] = 0;
        r[13] = (byte) 80;
        g[13] = (byte) 80;
        b[13] = (byte) 80;
        r[14] = (byte) 192;
        g[14] = (byte) 192;
        b[14] = (byte) 192;
        r[15] = (byte) 255;
        g[15] = (byte) 255;
        b[15] = (byte) 255;

        return new IndexColorModel(4, 16, r, g, b);
    }

    @Override
    public String toString() {
        return ("[ExplorationImage] ");
    }
// </editor-fold>

}
