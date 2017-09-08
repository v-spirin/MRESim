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
package gui;

import agents.Agent;
import agents.ComStation;
import agents.RealAgent;
import config.RobotConfig;
import config.RobotTeamConfig;
import config.SimulatorConfig;
import gui.ShowSettings.ShowSettings;
import gui.ShowSettings.ShowSettingsAgent;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.Toolkit;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.io.File;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.ImageIcon;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import simulator.ExplorationImage;
import simulator.SimulationFramework;

/**
 *
 * @author Julian de Hoog
 */
public class MainGUI extends javax.swing.JFrame {

    protected RobotTeamConfig robotTeamConfig;

    protected SimulatorConfig simConfig;

    public static enum runMode {
        running, paused, stopped
    }
    protected runMode RUNMODE;
    protected ExplorationImage explorationImage;
    protected SimulationFramework simulation;

    protected ShowSettings showSettings;
    protected ShowSettingsAgent[] showSettingsAgents;

    protected JFrame graphFrame;
    protected JPanel graphPanel;
    public Map<Integer, Double> jointKnowledge = new HashMap();

    /**
     * Creates new form MainGUI
     */
    public MainGUI() {
        RUNMODE = runMode.stopped;
        initComponents();
        this.addWindowListener(windowListener);
        // center on screen
        Dimension dim = Toolkit.getDefaultToolkit().getScreenSize();
        int x = (dim.width - getSize().width) / 2;
        int y = (dim.height - getSize().height) / 2;
        setLocation(x, y);

        robotTeamConfig = new RobotTeamConfig();
        simConfig = new SimulatorConfig();
        updateFromRobotTeamConfig();
        updateFromEnvConfig();
        //simulation = new SimulationFramework(this, robotTeamConfig, simConfig, explorationImage);

        initGraphFrame();
    }

    private void initGraphFrame() {
        graphFrame = new JFrame("Agent knowledge graph");
        graphPanel = new JPanel();
        graphPanel.setBackground(Color.white);
        graphFrame.setDefaultCloseOperation(JFrame.HIDE_ON_CLOSE);
        graphFrame.setSize(640, 480);
        graphFrame.getContentPane().setLayout(new GridLayout(1, 1));
        graphFrame.getContentPane().add(graphPanel);
        graphPanel.setSize(graphFrame.getSize());
        graphFrame.setVisible(false);
    }

// <editor-fold defaultstate="collapsed" desc="Get and Set">
    public javax.swing.JPanel getPanelConfiguration() {
        return null;
    }

    public javax.swing.JPanel getPanelRobotInfo() {
        return panelRobotInfo;
    }

    public javax.swing.JPanel getPanelExploration() {
        return panelExploration;
    }

    public javax.swing.JScrollPane getScrollpaneImage() {
        return scrollPaneImage;
    }

    public javax.swing.JLabel getLabelImageHolder() {
        return labelImageHolder;
    }

    public SimulationFramework getExploration() {
        return simulation;
    }

    public String getExplorationType() {
        return (null);
    }

    public boolean showEnv() {
        return toggleEnv.isSelected();
    }

    public void setShowHierarchy(boolean show) {
        toggleHierarchy.setSelected(show);
    }

    public boolean showHierarchy() {
        return toggleHierarchy.isSelected();
    }

    public void setShowAreas(boolean show) {
        toggleAreas.setSelected(show);

    }

    public boolean showAreas() {
        return toggleAreas.isSelected();
    }

    public RobotPanel getRobotPanel(int i) {
        //System.out.println(i);
        return (RobotPanel) panelRobotInfo.getComponent(i);
    }

    public ShowSettings getShowSettings() {
        return showSettings;
    }

    public ShowSettingsAgent[] getShowSettingsAgents() {
        return showSettingsAgents;
    }

    public void setRobotTeamConfig(RobotTeamConfig robotTeamConfig) {
        this.robotTeamConfig = robotTeamConfig;
        updateFromRobotTeamConfig();
        updateFromEnvConfig();
    }
    // </editor-fold>

// GUI changes
    public void updateFromRobotTeamConfig() {
        try {
            RobotConfig currRobot;
            RobotPanel currRobotPanel;

            panelRobotInfo.removeAll();
            panelRobotInfo.setLayout(new BoxLayout(panelRobotInfo, BoxLayout.Y_AXIS));
            for (int i = 0; i < robotTeamConfig.getNumRobots(); i++) {
                currRobot = (RobotConfig) (robotTeamConfig.getRobotTeam().get(i + 1));
                currRobotPanel = new RobotPanel(this, currRobot);
                currRobotPanel.getLabelRole().setText(currRobot.getRole().toString());
                panelRobotInfo.add(currRobotPanel);
            }
            panelRobotInfo.add(Box.createVerticalGlue());

            panelRobotInfo.repaint();

            panelRobotInfo.revalidate();
            //panelRobotInfo.getTopLevelAncestor().validate();
            explorationImage = new ExplorationImage(simConfig.getEnvironment());
            explorationImage.redrawEnvAndAgents(this, robotTeamConfig, simConfig);
            labelImageHolder.setIcon(new ImageIcon(explorationImage.getImage()));
            validate();
            updateShowSettings();
            updateShowSettingsAgents();

        } catch (NullPointerException e) {
            System.err.println("Updating from robotTeamConfig: null pointer exception.");
        }
    }

    public void updateRobotConfig() {
        robotTeamConfig.getRobotTeam().forEach((i, robotConfig) -> {
            robotConfig.setLoggingState(getRobotPanel(i - 1).loggingState());
        });//.get(i).setLoggingState(getRobotPanel(i).saveOccupancyGrid());
    }

    private void updateFromEnvConfig() {
        try {
            explorationImage = new ExplorationImage(simConfig.getEnvironment());
            explorationImage.redrawEnvAndAgents(this, robotTeamConfig, simConfig);
            labelImageHolder.setIcon(new ImageIcon(explorationImage.getImage()));
            validate();
        } catch (NullPointerException e) {
            System.err.println("Updating from envConfig: null pointer exception.");
        }
    }

    public void updateFromData(RealAgent agent[], int timeElapsed, double pctAreaKnown, int avgCycleTime) {

        RobotPanel currRobotPanel;
        DecimalFormat oneDigit = new DecimalFormat("#,###,##0.00");

        for (int i = 0; i < agent.length; i++) {
            currRobotPanel = (RobotPanel) panelRobotInfo.getComponent(i);
            currRobotPanel.getLabelRole().setText(agent[i].getRole().toString());
            String state;
            if (agent[i].getState() != Agent.AgentState.AKTIVE) {
                state = agent[i].getState().toString();
            } else {
                state = agent[i].getExploreState().toString();
            }
            currRobotPanel.getLabelState().setText(state);
            currRobotPanel.getLabelPower().setText(Integer.toString(agent[i].getBatteryPower()));
            currRobotPanel.getLabelDynamic().setText(agent[i].getDynamicInfoText());

            ArrayList<ComStation> comStations = agent[i].getComStations();
            String[] coms = new String[comStations.size()];
            for (int u = 0; u < comStations.size(); u++) {
                coms[u] = comStations.get(u).getName();
            }
            currRobotPanel.getRelayList().setListData(coms);

        }

        labelCycleUpdate.setText(Integer.toString(timeElapsed));
        //labelExploredUpdate.setText(Integer.toString(pctAreaKnown));
        labelExploredUpdate.setText(oneDigit.format(pctAreaKnown));
        labelAvgCycleUpdate.setText(avgCycleTime + "");
        //if (timeElapsed % Constants.UPDATE_GRAPH_INTERVAL == 0) updateKnowledgeGraph(agent);
    }

    public void updateShowSettings() {
        showSettings = new ShowSettings();
        showSettings.showEnv = showEnv();
        showSettings.showAreas = showAreas();
        showSettings.showHierarchy = showHierarchy();
    }

    public void updateShowSettingsAgents() {
        int numRobots = robotTeamConfig.getNumRobots();
        showSettingsAgents = new ShowSettingsAgent[numRobots];
        for (int i = 0; i < numRobots; i++) {
            showSettingsAgents[i] = new ShowSettingsAgent();
            if (getRobotPanel(i) != null) {
                showSettingsAgents[i].showAgent = getRobotPanel(i).showAgent();
                showSettingsAgents[i].showCommRange = getRobotPanel(i).showCommRange();
                showSettingsAgents[i].showFreeSpace = getRobotPanel(i).showFreeSpace();
                showSettingsAgents[i].showFrontiers = getRobotPanel(i).showFrontiers();
                showSettingsAgents[i].showPath = getRobotPanel(i).showPath();
                showSettingsAgents[i].showRendezvous = getRobotPanel(i).showRendezvous();
                showSettingsAgents[i].showSafeSpace = getRobotPanel(i).showSafeSpace();
                showSettingsAgents[i].showRVCandidatePointInfo = getRobotPanel(i).showSkeleton();
                showSettingsAgents[i].showBorderSkel = getRobotPanel(i).showBorderSkel();
                showSettingsAgents[i].showRVWalls = getRobotPanel(i).showRVWalls();
            }
        }
    }

    private JFreeChart createKnowledgeChart(RealAgent agent[]) {

        XYSeriesCollection xyDataset = new XYSeriesCollection();
        JFreeChart chart = ChartFactory.createXYLineChart(
                "Agent knowledge", // Title
                "Time", // X-Axis label
                "% map known", // Y-Axis label
                xyDataset, // Dataset
                PlotOrientation.VERTICAL,
                true, // Show legend
                true, // Tooltips
                false
        );
        if (agent == null || simulation == null) {
            return chart;
        }
        for (int i = 0; i < agent.length; i++) {
            //update agent data
            //agent[i].updateTrueAreaKnown(simConfig.getEnvironment());
            agent[i].knowledgeData.put(simulation.getTimeElapsed(), (double) agent[i].getStats().getAreaKnown() / (double) simulation.getTotalArea());

            XYSeries series = new XYSeries("Agent " + i);
            Iterator it = agent[i].knowledgeData.entrySet().iterator();
            while (it.hasNext()) {
                Map.Entry<Integer, Double> pairs = (Map.Entry) it.next();

                series.add(pairs.getKey(), pairs.getValue());
            }
            xyDataset.addSeries(series);
        }
        jointKnowledge.put(simulation.getTimeElapsed(), (double) simulation.getTrueJointAreaKnown() / (double) simulation.getTotalArea());
        XYSeries series = new XYSeries("Joint agent knowledge");
        Iterator it = jointKnowledge.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Integer, Double> pairs = (Map.Entry) it.next();

            series.add(pairs.getKey(), pairs.getValue());
        }
        xyDataset.addSeries(series);
        chart.setBackgroundPaint(Color.white);
        chart.getPlot().setBackgroundPaint(Color.black);
        chart.getPlot().setForegroundAlpha(1f);
        chart.getXYPlot().getRenderer().setSeriesPaint(0, Color.RED);
        chart.getXYPlot().getRenderer().setSeriesStroke(0, new BasicStroke(5));
        chart.getXYPlot().getRenderer().setSeriesPaint(1, Color.BLUE);
        chart.getXYPlot().getRenderer().setSeriesPaint(2, Color.YELLOW);
        chart.getXYPlot().getRenderer().setSeriesPaint(3, Color.GREEN);
        chart.getXYPlot().getRenderer().setSeriesPaint(4, Color.cyan);
        chart.getXYPlot().getRenderer().setSeriesPaint(agent.length, Color.MAGENTA);
        chart.getXYPlot().getRenderer().setSeriesStroke(agent.length, new BasicStroke(3));
        return chart;
    }

    public void updateKnowledgeGraph(RealAgent agent[]) {
        graphPanel.removeAll();
        JFreeChart chart = createKnowledgeChart(agent);
        JPanel chartPanel = new ChartPanel(chart);
        graphPanel.setLayout(new GridLayout(1, 1));
        graphPanel.add(chartPanel);
        graphPanel.getParent().validate();
        chartPanel.updateUI();
    }

    public void runComplete(RealAgent[] agent, int timeElapsed, double pctAreaKnownTeam, int avgCycleTime) {
        buttonStart.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonPlay.png")));
        RUNMODE = runMode.stopped;
        simulation.kill();
        simulation = null;
        JOptionPane.showMessageDialog(new JFrame(),
                "The run has finished.  There are no new position for the robots to move to.",
                "Run complete",
                JOptionPane.PLAIN_MESSAGE);
    }

    private boolean robotStartsOK() {
        RobotConfig currRobot;
        for (int i = 1; i <= robotTeamConfig.getNumRobots(); i++) {
            currRobot = (RobotConfig) robotTeamConfig.getRobotTeam().get(i);
            if (currRobot.getStartX() < 0
                    || currRobot.getStartX() > simConfig.getEnvironment().getColumns()
                    || currRobot.getStartY() < 0
                    || currRobot.getStartY() > simConfig.getEnvironment().getRows()) {
                return false;
            }
        }
        return true;
    }

    WindowListener windowListener = new WindowAdapter() {
        @Override
        public void windowClosing(WindowEvent w) {
            // save last used configs
            robotTeamConfig.saveConfig();
            simConfig.saveSimulatorConfig();
            simConfig.saveWallConfig();
            System.exit(0);
        }
    };

    /**
     * @param args the command line arguments
     */
    public static void main(String args[]) {
        java.awt.EventQueue.invokeLater(() -> {
            new MainGUI().setVisible(true);
        });
    }

    // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
    private void initComponents() {

        panelExploration = new javax.swing.JPanel();
        scrollPaneImage = new javax.swing.JScrollPane();
        labelImageHolder = new javax.swing.JLabel();
        buttonStart = new javax.swing.JButton();
        toggleHierarchy = new javax.swing.JToggleButton();
        toggleEnv = new javax.swing.JToggleButton();
        buttonStep = new javax.swing.JButton();
        buttonStop = new javax.swing.JButton();
        labelCycleUpdate = new javax.swing.JLabel();
        labelCycle = new javax.swing.JLabel();
        labelExploredUpdate = new javax.swing.JLabel();
        labelExplored = new javax.swing.JLabel();
        labelSpeed = new javax.swing.JLabel();
        sliderSpeed = new javax.swing.JSlider();
        scrollPaneRobots = new javax.swing.JScrollPane();
        panelRobotInfo = new javax.swing.JPanel();
        labelAvgCycle = new javax.swing.JLabel();
        labelAvgCycleUpdate = new javax.swing.JLabel();
        toggleAreas = new javax.swing.JToggleButton();
        MainMenu1 = new javax.swing.JMenuBar();
        menuExploration1 = new javax.swing.JMenu();
        menuCommunication1 = new javax.swing.JMenu();
        menuEnvironment1 = new javax.swing.JMenu();
        menuRobots1 = new javax.swing.JMenu();
        menuLogs1 = new javax.swing.JMenu();

        setTitle("Multi-robot Exploration Simulator (MRESim) v2.5");
        setName("OuterFrame"); // NOI18N
        setResizable(false);

        panelExploration.setMaximumSize(new java.awt.Dimension(800, 600));
        panelExploration.setMinimumSize(new java.awt.Dimension(800, 600));
        panelExploration.setRequestFocusEnabled(false);

        labelImageHolder.setBackground(new java.awt.Color(255, 255, 102));
        labelImageHolder.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        labelImageHolder.setMaximumSize(new java.awt.Dimension(800, 600));
        labelImageHolder.setMinimumSize(new java.awt.Dimension(800, 600));
        labelImageHolder.setPreferredSize(new java.awt.Dimension(800, 600));
        scrollPaneImage.setViewportView(labelImageHolder);

        javax.swing.GroupLayout panelExplorationLayout = new javax.swing.GroupLayout(panelExploration);
        panelExploration.setLayout(panelExplorationLayout);
        panelExplorationLayout.setHorizontalGroup(
            panelExplorationLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(scrollPaneImage, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
        );
        panelExplorationLayout.setVerticalGroup(
            panelExplorationLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addComponent(scrollPaneImage, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
        );

        buttonStart.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        buttonStart.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonPlay.png"))); // NOI18N
        buttonStart.setToolTipText("Start simulation");
        buttonStart.setBorderPainted(false);
        buttonStart.setContentAreaFilled(false);
        buttonStart.setIconTextGap(0);
        buttonStart.setMaximumSize(new java.awt.Dimension(36, 36));
        buttonStart.setMinimumSize(new java.awt.Dimension(36, 36));
        buttonStart.setPreferredSize(new java.awt.Dimension(36, 36));
        buttonStart.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonStartActionPerformed(evt);
            }
        });

        toggleHierarchy.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonHierarchy.png"))); // NOI18N
        toggleHierarchy.setToolTipText("Toggle hierarchy");
        toggleHierarchy.setBorderPainted(false);
        toggleHierarchy.setContentAreaFilled(false);
        toggleHierarchy.setMaximumSize(new java.awt.Dimension(36, 36));
        toggleHierarchy.setMinimumSize(new java.awt.Dimension(36, 36));
        toggleHierarchy.setPreferredSize(new java.awt.Dimension(36, 36));
        toggleHierarchy.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleHierarchyActionPerformed(evt);
            }
        });

        toggleEnv.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonEnvironment.png"))); // NOI18N
        toggleEnv.setSelected(true);
        toggleEnv.setToolTipText("Toggle environment");
        toggleEnv.setBorderPainted(false);
        toggleEnv.setContentAreaFilled(false);
        toggleEnv.setMaximumSize(new java.awt.Dimension(36, 36));
        toggleEnv.setMinimumSize(new java.awt.Dimension(36, 36));
        toggleEnv.setPreferredSize(new java.awt.Dimension(36, 36));
        toggleEnv.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleEnvActionPerformed(evt);
            }
        });

        buttonStep.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        buttonStep.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonStep.png"))); // NOI18N
        buttonStep.setToolTipText("Take one step");
        buttonStep.setBorderPainted(false);
        buttonStep.setContentAreaFilled(false);
        buttonStep.setMaximumSize(new java.awt.Dimension(36, 36));
        buttonStep.setMinimumSize(new java.awt.Dimension(36, 36));
        buttonStep.setPreferredSize(new java.awt.Dimension(36, 36));
        buttonStep.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonStepActionPerformed(evt);
            }
        });

        buttonStop.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        buttonStop.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonStop.png"))); // NOI18N
        buttonStop.setToolTipText("Reset simulation");
        buttonStop.setBorderPainted(false);
        buttonStop.setContentAreaFilled(false);
        buttonStop.setMaximumSize(new java.awt.Dimension(36, 36));
        buttonStop.setMinimumSize(new java.awt.Dimension(36, 36));
        buttonStop.setPreferredSize(new java.awt.Dimension(36, 36));
        buttonStop.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                buttonStopActionPerformed(evt);
            }
        });

        labelCycleUpdate.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        labelCycleUpdate.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        labelCycleUpdate.setText("0");

        labelCycle.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        labelCycle.setHorizontalAlignment(javax.swing.SwingConstants.RIGHT);
        labelCycle.setText("Cycle:");

        labelExploredUpdate.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        labelExploredUpdate.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        labelExploredUpdate.setText("0");

        labelExplored.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        labelExplored.setHorizontalAlignment(javax.swing.SwingConstants.RIGHT);
        labelExplored.setText("% Explored:");

        labelSpeed.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        labelSpeed.setHorizontalAlignment(javax.swing.SwingConstants.RIGHT);
        labelSpeed.setText("Speed:");

        sliderSpeed.setMajorTickSpacing(1);
        sliderSpeed.setMaximum(10);
        sliderSpeed.setSnapToTicks(true);
        sliderSpeed.setToolTipText("Simulation speed");
        sliderSpeed.addChangeListener(new javax.swing.event.ChangeListener() {
            public void stateChanged(javax.swing.event.ChangeEvent evt) {
                sliderSpeedStateChanged(evt);
            }
        });

        scrollPaneRobots.setHorizontalScrollBarPolicy(javax.swing.ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
        scrollPaneRobots.setVerticalScrollBarPolicy(javax.swing.ScrollPaneConstants.VERTICAL_SCROLLBAR_ALWAYS);
        scrollPaneRobots.setMinimumSize(new java.awt.Dimension(21, 22));
        scrollPaneRobots.setPreferredSize(new java.awt.Dimension(273, 600));

        panelRobotInfo.setRequestFocusEnabled(false);

        javax.swing.GroupLayout panelRobotInfoLayout = new javax.swing.GroupLayout(panelRobotInfo);
        panelRobotInfo.setLayout(panelRobotInfoLayout);
        panelRobotInfoLayout.setHorizontalGroup(
            panelRobotInfoLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 396, Short.MAX_VALUE)
        );
        panelRobotInfoLayout.setVerticalGroup(
            panelRobotInfoLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGap(0, 2000, Short.MAX_VALUE)
        );

        scrollPaneRobots.setViewportView(panelRobotInfo);

        labelAvgCycle.setFont(new java.awt.Font("Arial", 0, 10)); // NOI18N
        labelAvgCycle.setHorizontalAlignment(javax.swing.SwingConstants.RIGHT);
        labelAvgCycle.setText("Avg Cycle ms:");

        labelAvgCycleUpdate.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        labelAvgCycleUpdate.setHorizontalAlignment(javax.swing.SwingConstants.CENTER);
        labelAvgCycleUpdate.setText("0");

        toggleAreas.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonData.png"))); // NOI18N
        toggleAreas.setToolTipText("Toggle areas");
        toggleAreas.setBorderPainted(false);
        toggleAreas.setContentAreaFilled(false);
        toggleAreas.setMaximumSize(new java.awt.Dimension(36, 36));
        toggleAreas.setMinimumSize(new java.awt.Dimension(36, 36));
        toggleAreas.setPreferredSize(new java.awt.Dimension(36, 36));
        toggleAreas.addActionListener(new java.awt.event.ActionListener() {
            public void actionPerformed(java.awt.event.ActionEvent evt) {
                toggleAreasActionPerformed(evt);
            }
        });

        MainMenu1.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N

        menuExploration1.setText("Exploration");
        menuExploration1.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        menuExploration1.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                menuExplorationMouseClicked(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                menuExplorationMouseReleased(evt);
            }
        });
        menuExploration1.addMenuDragMouseListener(new javax.swing.event.MenuDragMouseListener() {
            public void menuDragMouseEntered(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseExited(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseDragged(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseReleased(javax.swing.event.MenuDragMouseEvent evt) {
                menuExplorationMenuDragMouseReleased(evt);
            }
        });
        MainMenu1.add(menuExploration1);

        menuCommunication1.setText("Communication");
        menuCommunication1.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        menuCommunication1.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                menuCommunicationMouseClicked(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                menuCommunicationMouseReleased(evt);
            }
        });
        menuCommunication1.addMenuDragMouseListener(new javax.swing.event.MenuDragMouseListener() {
            public void menuDragMouseEntered(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseExited(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseDragged(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseReleased(javax.swing.event.MenuDragMouseEvent evt) {
                menuCommunicationMenuDragMouseReleased(evt);
            }
        });
        MainMenu1.add(menuCommunication1);

        menuEnvironment1.setText("Environment");
        menuEnvironment1.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        menuEnvironment1.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                menuEnvironmentMouseClicked(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                menuEnvironmentMouseReleased(evt);
            }
        });
        menuEnvironment1.addMenuDragMouseListener(new javax.swing.event.MenuDragMouseListener() {
            public void menuDragMouseEntered(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseExited(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseDragged(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseReleased(javax.swing.event.MenuDragMouseEvent evt) {
                menuEnvironmentMenuDragMouseReleased(evt);
            }
        });
        MainMenu1.add(menuEnvironment1);

        menuRobots1.setText("Robots");
        menuRobots1.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        menuRobots1.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                menuRobotsMouseClicked(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                menuRobotsMouseReleased(evt);
            }
        });
        menuRobots1.addMenuDragMouseListener(new javax.swing.event.MenuDragMouseListener() {
            public void menuDragMouseEntered(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseExited(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseDragged(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseReleased(javax.swing.event.MenuDragMouseEvent evt) {
                menuRobotsMenuDragMouseReleased(evt);
            }
        });
        MainMenu1.add(menuRobots1);

        menuLogs1.setText("Logs");
        menuLogs1.setFont(new java.awt.Font("Arial", 0, 11)); // NOI18N
        menuLogs1.addMouseListener(new java.awt.event.MouseAdapter() {
            public void mouseClicked(java.awt.event.MouseEvent evt) {
                menuLogsMouseClicked(evt);
            }
            public void mouseReleased(java.awt.event.MouseEvent evt) {
                menuLogsMouseReleased(evt);
            }
        });
        menuLogs1.addMenuDragMouseListener(new javax.swing.event.MenuDragMouseListener() {
            public void menuDragMouseEntered(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseExited(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseDragged(javax.swing.event.MenuDragMouseEvent evt) {
            }
            public void menuDragMouseReleased(javax.swing.event.MenuDragMouseEvent evt) {
                menuLogsMenuDragMouseReleased(evt);
            }
        });
        MainMenu1.add(menuLogs1);

        setJMenuBar(MainMenu1);

        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(panelExploration, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
                        .addComponent(scrollPaneRobots, javax.swing.GroupLayout.DEFAULT_SIZE, 411, Short.MAX_VALUE))
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(labelSpeed, javax.swing.GroupLayout.PREFERRED_SIZE, 44, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addComponent(sliderSpeed, javax.swing.GroupLayout.PREFERRED_SIZE, 89, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(164, 164, 164)
                        .addComponent(buttonStart, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(buttonStep, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(buttonStop, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(231, 231, 231)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                            .addGroup(layout.createSequentialGroup()
                                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                                    .addComponent(labelCycle)
                                    .addComponent(labelExplored, javax.swing.GroupLayout.PREFERRED_SIZE, 70, javax.swing.GroupLayout.PREFERRED_SIZE))
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING, false)
                                    .addComponent(labelExploredUpdate, javax.swing.GroupLayout.PREFERRED_SIZE, 36, javax.swing.GroupLayout.PREFERRED_SIZE)
                                    .addComponent(labelCycleUpdate, javax.swing.GroupLayout.PREFERRED_SIZE, 36, javax.swing.GroupLayout.PREFERRED_SIZE)))
                            .addGroup(layout.createSequentialGroup()
                                .addComponent(labelAvgCycle, javax.swing.GroupLayout.PREFERRED_SIZE, 70, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                                .addComponent(labelAvgCycleUpdate, javax.swing.GroupLayout.PREFERRED_SIZE, 44, javax.swing.GroupLayout.PREFERRED_SIZE)))
                        .addGap(65, 65, 65)
                        .addComponent(toggleEnv, javax.swing.GroupLayout.PREFERRED_SIZE, 36, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(toggleHierarchy, javax.swing.GroupLayout.PREFERRED_SIZE, 36, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(toggleAreas, javax.swing.GroupLayout.PREFERRED_SIZE, 36, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(0, 0, Short.MAX_VALUE)))
                .addContainerGap())
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
                    .addComponent(panelExploration, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                    .addComponent(scrollPaneRobots, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
                .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                    .addGroup(layout.createSequentialGroup()
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(labelCycleUpdate)
                            .addComponent(labelCycle))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(labelExploredUpdate)
                            .addComponent(labelExplored))
                        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                            .addComponent(labelAvgCycleUpdate)
                            .addComponent(labelAvgCycle)))
                    .addGroup(layout.createSequentialGroup()
                        .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
                            .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                                .addComponent(buttonStop, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addComponent(buttonStep, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                                .addComponent(buttonStart, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
                            .addComponent(labelSpeed)
                            .addComponent(sliderSpeed, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(toggleHierarchy, javax.swing.GroupLayout.PREFERRED_SIZE, 36, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(toggleEnv, javax.swing.GroupLayout.PREFERRED_SIZE, 36, javax.swing.GroupLayout.PREFERRED_SIZE)
                            .addComponent(toggleAreas, javax.swing.GroupLayout.PREFERRED_SIZE, 36, javax.swing.GroupLayout.PREFERRED_SIZE))
                        .addGap(0, 0, Short.MAX_VALUE)))
                .addContainerGap())
        );

        pack();
    }// </editor-fold>//GEN-END:initComponents

    private void buttonStartActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_buttonStartActionPerformed
        // Quick error check to make sure starting positions are within env bounds
        if (!robotStartsOK()) {
            JOptionPane.showMessageDialog(new JFrame(), "Error: robot starting positions are not within environment bounds!", "Startup error", JOptionPane.ERROR_MESSAGE);
            return;
        }

        switch (RUNMODE) {
            case stopped:
                RUNMODE = runMode.running;
                buttonStart.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonPause.png")));
                simulation = new SimulationFramework(this, robotTeamConfig, simConfig, explorationImage);
                simulation.start();
                break;
            case running:
                RUNMODE = runMode.paused;
                buttonStart.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonPlay.png")));
                simulation.pause();
                break;
            case paused:
                RUNMODE = runMode.running;
                buttonStart.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonPause.png")));
                if (simulation == null) {
                    startSimulation();
                } else {
                    simulation.start();
                }
                break;
            default:
                break;
        }
}//GEN-LAST:event_buttonStartActionPerformed

    public void startSimulation() {
        simulation = new SimulationFramework(this, robotTeamConfig, simConfig, explorationImage);
        simulation.start();
    }
    private void buttonStepActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_buttonStepActionPerformed
        switch (RUNMODE) {
            case stopped:
                RUNMODE = runMode.paused;
                buttonStart.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonPlay.png")));
                simulation = new SimulationFramework(this, robotTeamConfig, simConfig, explorationImage);
                simulation.start();
                simulation.takeOneStep();
                break;
            case running:
                RUNMODE = runMode.paused;
                buttonStart.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonPlay.png")));
                simulation.takeOneStep();
                break;
            case paused:
                simulation.start();
                simulation.takeOneStep();
                break;
            default:
                break;
        }
}//GEN-LAST:event_buttonStepActionPerformed

    private void buttonStopActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_buttonStopActionPerformed
        RUNMODE = runMode.paused;
        buttonStart.setIcon(new javax.swing.ImageIcon(getClass().getResource("/resources/buttonPlay.png")));
        if (simulation != null) {
            simulation.pause();

            int answer = JOptionPane.showConfirmDialog(new JFrame(),
                    "Are you sure you want to reset?  This will completely stop the run.",
                    "Confirm Reset",
                    JOptionPane.YES_NO_OPTION);

            if (answer == 0) {
                RUNMODE = runMode.stopped;
                simulation.kill();
                simulation = null;
                this.updateFromEnvConfig();
            }
        }
}//GEN-LAST:event_buttonStopActionPerformed

    private void sliderSpeedStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_sliderSpeedStateChanged
        try {
            int newSimRate = sliderSpeed.getValue();
            simConfig.setSimRate(newSimRate);
            if (simulation != null) {
                simulation.simRateChanged(newSimRate, RUNMODE);
            }
        } catch (NullPointerException e) {
        }
    }//GEN-LAST:event_sliderSpeedStateChanged

    private void menuCommunicationMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_menuCommunicationMouseClicked
        CommunicationDialog commDialog = new CommunicationDialog(this, true, simConfig);
    }//GEN-LAST:event_menuCommunicationMouseClicked

    private void menuEnvironmentMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_menuEnvironmentMouseClicked
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setCurrentDirectory(new File(System.getProperty("user.dir") + "/environments/"));
        int returnVal = fileChooser.showOpenDialog(this);
        if (returnVal == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile();
            simConfig.loadEnvironment(file.getPath());
            updateFromEnvConfig();
        }
    }//GEN-LAST:event_menuEnvironmentMouseClicked

    private void toggleEnvActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleEnvActionPerformed
        if (!toggleEnv.isSelected()) {
            simulation.setForce_full_update(true);
        }
        updateShowSettings();
    }//GEN-LAST:event_toggleEnvActionPerformed

    private void menuLogsMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_menuLogsMouseClicked
        LogDialog logDialog = new LogDialog(this, true, simConfig);
    }//GEN-LAST:event_menuLogsMouseClicked

    private void menuExplorationMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_menuExplorationMouseClicked
        ExplorationDialog explorationDialog = new ExplorationDialog(this, true, simConfig);
    }//GEN-LAST:event_menuExplorationMouseClicked

    private void menuRobotsMouseClicked(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_menuRobotsMouseClicked
        RobotConfigDialog robotConfigDialog = new RobotConfigDialog(this, robotTeamConfig);
    }//GEN-LAST:event_menuRobotsMouseClicked

    private void menuExplorationMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_menuExplorationMouseReleased

    }//GEN-LAST:event_menuExplorationMouseReleased

    private void menuCommunicationMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_menuCommunicationMouseReleased

    }//GEN-LAST:event_menuCommunicationMouseReleased

    private void menuEnvironmentMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_menuEnvironmentMouseReleased

    }//GEN-LAST:event_menuEnvironmentMouseReleased

    private void menuRobotsMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_menuRobotsMouseReleased

    }//GEN-LAST:event_menuRobotsMouseReleased

    private void menuLogsMouseReleased(java.awt.event.MouseEvent evt) {//GEN-FIRST:event_menuLogsMouseReleased

    }//GEN-LAST:event_menuLogsMouseReleased

    private void menuLogsMenuDragMouseReleased(javax.swing.event.MenuDragMouseEvent evt) {//GEN-FIRST:event_menuLogsMenuDragMouseReleased
        LogDialog logDialog = new LogDialog(this, true, simConfig);
    }//GEN-LAST:event_menuLogsMenuDragMouseReleased

    private void menuRobotsMenuDragMouseReleased(javax.swing.event.MenuDragMouseEvent evt) {//GEN-FIRST:event_menuRobotsMenuDragMouseReleased
        RobotConfigDialog robotConfigDialog = new RobotConfigDialog(this, robotTeamConfig);
    }//GEN-LAST:event_menuRobotsMenuDragMouseReleased

    private void menuEnvironmentMenuDragMouseReleased(javax.swing.event.MenuDragMouseEvent evt) {//GEN-FIRST:event_menuEnvironmentMenuDragMouseReleased
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setCurrentDirectory(new File(System.getProperty("user.dir") + "/environments/"));
        int returnVal = fileChooser.showOpenDialog(this);
        if (returnVal == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile();
            simConfig.loadEnvironment(file.getPath());
            updateFromEnvConfig();
        }
    }//GEN-LAST:event_menuEnvironmentMenuDragMouseReleased

    private void menuCommunicationMenuDragMouseReleased(javax.swing.event.MenuDragMouseEvent evt) {//GEN-FIRST:event_menuCommunicationMenuDragMouseReleased
        CommunicationDialog commDialog = new CommunicationDialog(this, true, simConfig);
    }//GEN-LAST:event_menuCommunicationMenuDragMouseReleased

    private void menuExplorationMenuDragMouseReleased(javax.swing.event.MenuDragMouseEvent evt) {//GEN-FIRST:event_menuExplorationMenuDragMouseReleased
        ExplorationDialog explorationDialog = new ExplorationDialog(this, true, simConfig);
    }//GEN-LAST:event_menuExplorationMenuDragMouseReleased

    private void toggleHierarchyActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleHierarchyActionPerformed
        if (!toggleHierarchy.isSelected()) {
            simulation.setForce_full_update(true);
        }
        updateShowSettings();
    }//GEN-LAST:event_toggleHierarchyActionPerformed

    private void toggleAreasActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_toggleAreasActionPerformed
        if (!toggleAreas.isSelected()) {
            simulation.setForce_full_update(true);
        }
        updateShowSettings();// TODO add your handling code here:
    }//GEN-LAST:event_toggleAreasActionPerformed

    // Variables declaration - do not modify//GEN-BEGIN:variables
    private javax.swing.JMenuBar MainMenu1;
    private javax.swing.JButton buttonStart;
    private javax.swing.JButton buttonStep;
    private javax.swing.JButton buttonStop;
    private javax.swing.JLabel labelAvgCycle;
    private javax.swing.JLabel labelAvgCycleUpdate;
    private javax.swing.JLabel labelCycle;
    private javax.swing.JLabel labelCycleUpdate;
    private javax.swing.JLabel labelExplored;
    private javax.swing.JLabel labelExploredUpdate;
    public javax.swing.JLabel labelImageHolder;
    private javax.swing.JLabel labelSpeed;
    private javax.swing.JMenu menuCommunication1;
    private javax.swing.JMenu menuEnvironment1;
    private javax.swing.JMenu menuExploration1;
    private javax.swing.JMenu menuLogs1;
    private javax.swing.JMenu menuRobots1;
    private javax.swing.JPanel panelExploration;
    private javax.swing.JPanel panelRobotInfo;
    private javax.swing.JScrollPane scrollPaneImage;
    private javax.swing.JScrollPane scrollPaneRobots;
    private javax.swing.JSlider sliderSpeed;
    private javax.swing.JToggleButton toggleAreas;
    private javax.swing.JToggleButton toggleEnv;
    private javax.swing.JToggleButton toggleHierarchy;
    // End of variables declaration//GEN-END:variables

    @Override
    public String toString() {
        return ("[MainGUI] ");
    }
}
