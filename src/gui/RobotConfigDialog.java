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

import config.RobotTeamConfig;
import config.RobotConfig;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.io.File;
import javax.swing.DefaultCellEditor;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.table.DefaultTableCellRenderer;
import javax.swing.table.TableCellRenderer;
import javax.swing.table.TableColumn;
import javax.swing.table.TableColumnModel;

public class RobotConfigDialog extends JDialog {

// <editor-fold defaultstate="collapsed" desc="Class Variables and Constructor">
    private RobotTeamConfig robotTeamConfig;
    private RobotConfigTableModel model;
    
    private MainGUI mainGUI;
    
    private JTable table;
    private JScrollPane scrollpane;
    private JButton buttonLoad;
    private JButton buttonSave;
    private JButton buttonAdd;
    private JButton buttonRemove;
    private JButton buttonCancel;
    private JButton buttonOK;
    
    public int status;  // To know whether config is ok or not, i.e. when OK pressed.

    public RobotConfigDialog(MainGUI maingui, RobotTeamConfig rtc) {
        super();
        this.mainGUI = maingui;
        this.robotTeamConfig = rtc;
        this.status = 0;
  
        initModel();
        initTable();
        initGUI();
    } 
// </editor-fold>     

// <editor-fold defaultstate="collapsed" desc="Initialization">
    private void initModel() {
        model = new RobotConfigTableModel();

        for (int i = 0; i < RobotConfig.NUMROBOTCONFIGFIELDS; i++) {
            model.addColumn("");         
        } 
            
        updateGUI();
    }

    private void initTable() {
        TableCellRenderer rankRenderer = new RankRenderer();
        TableCellRenderer nameRenderer = new NameRenderer();
        TableCellRenderer editableNumberRenderer = new EditableNumberRenderer();
        TableCellRenderer uneditableNumberRenderer = new UneditableNumberRenderer();
        TableCellRenderer roleRenderer = new NameRenderer();
        TableColumn column;
        
        table = new JTable(model);
        table.setFont(new java.awt.Font("Arial", 0, 12));
        table.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);
        for (int i = 0; i < RobotConfig.NUMROBOTCONFIGFIELDS; i++) {
            column = table.getColumnModel().getColumn(i);
            switch(i) {
                case 0:
                    column.setHeaderValue("#");
                    column.setPreferredWidth(30);
                    column.setCellRenderer(rankRenderer);
                    break;
                case 1: 
                    column.setHeaderValue("Name");
                    column.setPreferredWidth(120);
                    column.setCellRenderer(nameRenderer);
                    break;                                      
                case 2:
                    column.setHeaderValue("Start (x,y,a)");
                    column.setPreferredWidth(92);
                    column.setCellRenderer(editableNumberRenderer);
                    break; 
                case 3:
                    column.setHeaderValue("Sense range");
                    column.setPreferredWidth(84);
                    column.setCellRenderer(uneditableNumberRenderer);
                   break;                                      
                case 4:
                    column.setHeaderValue("Comm range");
                    column.setPreferredWidth(84);
                    column.setCellRenderer(editableNumberRenderer);
                    break;                                                                           
                case 5:
                    column.setHeaderValue("Battery Life");
                    column.setPreferredWidth(84);
                    column.setCellRenderer(editableNumberRenderer);
                    break;                                      
                case 6:
                    column.setHeaderValue("Role");
                    column.setPreferredWidth(100);
                    column.setCellRenderer(roleRenderer);
                    break;                                      
                case 7:
                    column.setHeaderValue("Parent");
                    column.setPreferredWidth(60);
                    column.setCellRenderer(uneditableNumberRenderer);
                    break;                                      
                case 8:
                    column.setHeaderValue("Child");
                    column.setPreferredWidth(60);
                    column.setCellRenderer(editableNumberRenderer);
                    break;
                case 9:
                    column.setHeaderValue("Ability");
                    column.setPreferredWidth(60);
                    column.setCellRenderer(editableNumberRenderer);
                    break;
                case 10:
                    column.setHeaderValue("ComSt.Limit");
                    column.setPreferredWidth(60);
                    column.setCellRenderer(editableNumberRenderer);
                    break;
                case 11:
                    column.setHeaderValue("Speed");
                    column.setPreferredWidth(60);
                    column.setCellRenderer(editableNumberRenderer);
                    break;
            }
        } 
        
    }
    
    
    class RankRenderer extends DefaultTableCellRenderer
    {
        @Override
        public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column) {
            super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, column);
                setHorizontalAlignment(CENTER);
                setBackground(new Color(204, 204, 204));
            return this;
	}
    }
    
    class EditableNumberRenderer extends DefaultTableCellRenderer
    {
        @Override
        public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column) {
            super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, column);
                setHorizontalAlignment(CENTER);
            return this;
	}
    }
    
    class UneditableNumberRenderer extends DefaultTableCellRenderer
    {
        @Override
        public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column) {
            super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, column);
                setHorizontalAlignment(CENTER);
                if(row == 0)
                    setBackground(new Color(204, 204, 204));
                else
                    setBackground(Color.white);
            return this;
	}
    }

    class NameRenderer extends DefaultTableCellRenderer
    {
        @Override
        public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column) {
            super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, column);
                setHorizontalAlignment(LEFT);
                if(row == 0)
                    setBackground(new Color(204, 204, 204));
                else
                    setBackground(Color.white);
            return this;
	}
    }
// </editor-fold> 
    
// <editor-fold defaultstate="collapsed" desc="GUI">
    private void initGUI() {
        scrollpane = new JScrollPane(table);  
        setTitle("Robot Team Configuration");
        
        this.addWindowListener(new java.awt.event.WindowAdapter() {
            public void actionPerformed(ActionEvent event) {
                buttonCancelClicked();
            
            }
        });

        JComboBox comboRole = new JComboBox(robotTeamConfig.getAllRoles());
        comboRole.setEditable(false);
        DefaultCellEditor editor = new DefaultCellEditor(comboRole);
        TableColumnModel tcm = table.getColumnModel();
        tcm.getColumn(6).setCellEditor(editor);


        buttonLoad = new JButton("Load");
        buttonLoad.setFont(new java.awt.Font("Arial", 0, 11));
        buttonLoad.addActionListener((ActionEvent event) -> {
            buttonLoadClicked();
        });

        buttonSave = new JButton("Save");
        buttonSave.setFont(new java.awt.Font("Arial", 0, 11));
        buttonSave.addActionListener((ActionEvent event) -> {
            buttonSaveClicked();
        });

        buttonAdd = new JButton("Add");
        buttonAdd.setFont(new java.awt.Font("Arial", 0, 11));
        buttonAdd.addActionListener((ActionEvent event) -> {
            buttonAddClicked();
        });

        buttonRemove = new JButton("Remove");
        buttonRemove.setFont(new java.awt.Font("Arial", 0, 11));
        buttonRemove.addActionListener((ActionEvent event) -> {
            buttonRemoveClicked();
        });
        
        buttonOK = new JButton("OK");
        buttonOK.setFont(new java.awt.Font("Arial", 0, 11));
        buttonOK.addActionListener((ActionEvent event) -> {
            buttonOKClicked();
        });
        
        buttonCancel = new JButton("Cancel");
        buttonCancel.setFont(new java.awt.Font("Arial", 0, 11));
        buttonCancel.addActionListener((ActionEvent event) -> {
            buttonCancelClicked();
        });
        
        JPanel inputPanel = new JPanel();
        inputPanel.add(buttonLoad);
        inputPanel.add(buttonSave);
        inputPanel.add(buttonAdd);
        inputPanel.add(buttonRemove);
        inputPanel.add(buttonOK);
        inputPanel.add(buttonCancel);

        // <editor-fold defaultstate="collapsed" desc="Layout">
        javax.swing.GroupLayout layout = new javax.swing.GroupLayout(getContentPane());
        getContentPane().setLayout(layout);
        layout.setHorizontalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, layout.createSequentialGroup()
                .addContainerGap()
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING)
                    .addComponent(scrollpane, javax.swing.GroupLayout.DEFAULT_SIZE, 679, Short.MAX_VALUE)
                    .addGroup(layout.createSequentialGroup()
                        .addComponent(buttonLoad, javax.swing.GroupLayout.PREFERRED_SIZE, 80, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(buttonSave, javax.swing.GroupLayout.PREFERRED_SIZE, 80, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(50, 50, 50)
                        .addComponent(buttonAdd, javax.swing.GroupLayout.PREFERRED_SIZE, 80, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(buttonRemove, javax.swing.GroupLayout.PREFERRED_SIZE, 80, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(50, 50, 50)
                        .addComponent(buttonOK, javax.swing.GroupLayout.PREFERRED_SIZE, 80, javax.swing.GroupLayout.PREFERRED_SIZE)
                        .addGap(18, 18, 18)
                        .addComponent(buttonCancel, javax.swing.GroupLayout.PREFERRED_SIZE, 80, javax.swing.GroupLayout.PREFERRED_SIZE)))
                .addContainerGap())
        );
        layout.setVerticalGroup(
            layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
            .addGroup(layout.createSequentialGroup()
                .addContainerGap()
                .addComponent(scrollpane, javax.swing.GroupLayout.PREFERRED_SIZE, 217, javax.swing.GroupLayout.PREFERRED_SIZE)
                .addGap(18, 18, 18)
                .addGroup(layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
                    .addComponent(buttonLoad)
                    .addComponent(buttonSave)
                    .addComponent(buttonAdd)
                    .addComponent(buttonRemove)
                    .addComponent(buttonOK)
                    .addComponent(buttonCancel))
                .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
        );
        // </editor-fold>
        pack();

        // Let window X being clicked be handled by windowClosing method
        setDefaultCloseOperation(DO_NOTHING_ON_CLOSE);
        setSize(950, 330);
        this.addWindowListener(windowListener);
        
        // center on screen
        Dimension dim = Toolkit.getDefaultToolkit().getScreenSize();
        int x = (dim.width-getSize().width)/2;
        int y = (dim.height-getSize().height)/2;
        setLocation(x, y);

        setVisible(true);
    }
    
    private void updateGUI() {
        //Remove all existing rows
        for (int i=model.getRowCount()-1; i>=0; i--)
            model.removeRow(i);
        
        //Replace with current robotTeam
        RobotConfig currRobot;
        String[] addLine = new String[RobotConfig.NUMROBOTCONFIGFIELDS];
        for (int j=1; j<=robotTeamConfig.getNumRobots(); j++) {
            currRobot = (RobotConfig)(robotTeamConfig.getRobotTeam().get(j));
            addLine[0] = String.valueOf(currRobot.getRobotNumber());
            addLine[1] = currRobot.getName();
            addLine[2] = String.valueOf(currRobot.getStartX()).concat(",").concat(
                         String.valueOf(currRobot.getStartY())).concat(",").concat(
                         String.valueOf(currRobot.getStartHeading()));
            addLine[3] = String.valueOf(currRobot.getSensingRange());
            addLine[4] = String.valueOf(currRobot.getCommRange());
            addLine[5] = String.valueOf(currRobot.getBatteryLife());
            addLine[6] = String.valueOf(currRobot.getRole());
            addLine[7] = String.valueOf(currRobot.getParent());
            addLine[8] = String.valueOf(currRobot.getChild());
            addLine[9] = String.valueOf(currRobot.getAbility());
            addLine[10] = String.valueOf(currRobot.getComStationLimit());
            addLine[11] = String.valueOf(currRobot.getSpeed());
            
            model.addRow(addLine);
        }
    }
// </editor-fold> 
    
// <editor-fold defaultstate="collapsed" desc="Load/save robot configuration">
    private void loadConfig(File filename) {
        robotTeamConfig.loadConfig(filename.getPath());
        //For debug: robotTeamConfig.printDetails();
        updateGUI();
    }
    
    private void saveConfig(File filename) {
        robotTeamConfig.updateFromGUI(model);
        robotTeamConfig.saveConfig(filename.getPath());
    }
 // </editor-fold> 
   
// <editor-fold defaultstate="collapsed" desc="Buttons clicked">
    private void buttonLoadClicked() {
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setCurrentDirectory(new File(System.getProperty("user.dir") + "/config/"));
        int returnVal = fileChooser.showOpenDialog(scrollpane);
        if (returnVal == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile();
            loadConfig(file);
        }
    }
    
    private void buttonSaveClicked() {
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setCurrentDirectory(new File(System.getProperty("user.dir") + "/config/"));
        int returnVal = fileChooser.showSaveDialog(scrollpane);
        if (returnVal == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile();
            saveConfig(file);
        }
    }
    
    private void buttonAddClicked() {
        String[] newRobot = { "", "", "", "", "", "" };
        model.addRow(newRobot);
        updateRanks();   
    }
    
    private void buttonRemoveClicked() {
        if(table.getSelectedRow()>=1)
            model.removeRow(table.getSelectedRow());
        else if(table.getRowCount() > 1)
            model.removeRow(table.getRowCount()-1);
        updateRanks();   
    }
    
    private void buttonOKClicked() {
        String errors = checkForErrors();
        if(errors.equals("none")) {
            robotTeamConfig.updateFromGUI(model);
            mainGUI.updateFromRobotTeamConfig();
            //mainGUI.getPanelConfiguration().updateUI();
            this.dispose();
        }
        else {
            JOptionPane.showMessageDialog(new JFrame(), errors, "Robot config errors", JOptionPane.ERROR_MESSAGE);
        }
    }
    
    private void buttonCancelClicked() {
        this.dispose();
    }

 // </editor-fold> 
    
// <editor-fold defaultstate="collapsed" desc="Helper functions">
    private void updateRanks() {
        for (int i = 0; i < table.getRowCount(); i++) 
            table.setValueAt(String.valueOf(i+1), i, 0);
    }
    
    private String checkForErrors() {
        boolean errorFound = false;
        String errors = new String();
        for(int i=0; i<model.getRowCount(); i++) {
            if(((String)(model.getValueAt(i, 1))).isEmpty()){
                errors = errors + "Incorrect Name for Robot in line " + (i+1) + "\n";
                errorFound = true;
            }
            
            // Still need to add start checks
            
            try {Integer.parseInt((String)(model.getValueAt(i, 3)));}
            catch (NumberFormatException e) {errors = errors + "Incorrect Comm range for Robot in line " + (i+1) + "\n"; errorFound = true;}

            try {Integer.parseInt((String)(model.getValueAt(i, 4)));}
            catch (NumberFormatException e) {errors = errors + "Incorrect Sensing range for Robot in line " + (i+1) + "\n"; errorFound = true;}

            try {Integer.parseInt((String)(model.getValueAt(i, 5)));}
            catch (NumberFormatException e) {errors = errors + "Incorrect Battery life for Robot in line " + (i+1) + "\n"; errorFound = true;}

            // Still need to add role, parent, child checks
        }
        if (errorFound == false)
            errors = "none";
        
        return(errors);
    }
    
    WindowListener windowListener = new WindowAdapter() {
        @Override
        public void windowClosing ( WindowEvent w ) {
            // do the same for window's X being clicked as for Cancel button
            buttonCancelClicked();
        } 
    };
// </editor-fold>
   
}