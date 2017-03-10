/*
 *     Copyright 2010, 2015 Julian de Hoog (julian@dehoog.ca), Victor Spirin (victor.spirin@cs.ox.ac.uk)
 *
 *     This file is part of MRESim 2.2, a simulator for testing the behaviour
 *     of multiple robots exploring unknown environments.
 *
 *     If you use MRESim, I would appreciate an acknowledgement and/or a citation
 *     of our papers:
 *
 *     @inproceedings{deHoog2009,
 *         title = "Role-Based Autonomous Multi-Robot Exploration",
 *         author = "Julian de Hoog, Stephen Cameron and Arnoud Visser",
 *         year = "2009",
 *         booktitle = "International Conference on Advanced Cognitive Technologies and Applications (COGNITIVE)",
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
package config;

import environment.Environment;
import java.awt.Color;
import java.awt.image.BufferedImage;
import java.awt.image.Raster;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JOptionPane;

/**
 *
 * @author juliandehoog
 */
public class EnvLoader {


// <editor-fold defaultstate="collapsed" desc="Save Environment">

    public static boolean saveWallConfig_ImageBased(Environment env, String fileName) {
        try{
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(className() + "Trying to save image based environment ... ");
            }
            BufferedImage image = new BufferedImage(Constants.MAX_COLS, Constants.MAX_ROWS, BufferedImage.TYPE_INT_RGB);

            for(int i=0; i<Constants.MAX_COLS; i++)
                for(int j=0; j<Constants.MAX_ROWS; j++)
                    switch (env.statusAt(i, j)){
                        case unexplored: image.setRGB(i, j, Color.blue.getRGB());
                        break;
                        case explored: image.setRGB(i, j, Color.white.getRGB());
                        break;
                        case slope: image.setRGB(i, j, Color.yellow.getRGB());
                        break;
                        case hill: image.setRGB(i, j, new Color(255,69,0).getRGB());
                        break;
                        case obstacle: image.setRGB(i, j, Color.red.getRGB());
                        break;
                        case barrier: image.setRGB(i, j, Color.black.getRGB());
                        break;
                        
                    }
                    /*if(env.obstacleAt(i, j))
                        image.setRGB(i, j, 0);
                    else
                        image.setRGB(i,j,16777215); // int rgb value for white*/

            ImageIO.write(image, "png", new File(fileName));


            if (Constants.DEBUG_OUTPUT) {
                System.out.println(className() + "Environment saved successfully.");
            }
            return true;
        }
        catch (IOException e) {
            System.err.println(className() + "Error: could not write data to " + fileName);
        }

        return false;
    }

    public static boolean saveWallConfig_TextBased(Environment env, String fileName) {
        try{
            PrintWriter outFile = new PrintWriter(new FileWriter(fileName));

            outFile.println(env.getRows());
            outFile.println(env.getColumns());
            for(int i=0; i<env.getRows(); i++){
                for(int j=0; j<env.getColumns(); j++)
                    outFile.print(env.statusAt(i, j).ordinal());
                   /* if(env.statusAt(j, i) == Environment.Status.obstacle)
                        outFile.print("1");
                    else
                        outFile.print("0");*/
                outFile.println();
            }

            outFile.close();
            return true;
        }
        catch(IOException e){
            System.err.println(className() + "Error writing to file " + fileName);
        }

        return false;
    }

    public static boolean saveWallConfig(Environment env, String fileName) {
        String extension = fileName.substring(fileName.lastIndexOf('.')+1, fileName.length());

        if(extension.equals("txt"))
            return saveWallConfig_TextBased(env, fileName);
        else
            return saveWallConfig_ImageBased(env, fileName);
    }

    public static boolean saveWallConfig(Environment env) {
        return saveWallConfig(env, System.getProperty("user.dir") + "/config/lastEnvironment.png");
    }

// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="Load Environment">

    public static Environment loadWallConfig_TextBased(String fileName) {
        File file = new File(fileName);

        if (!file.exists())
            return null;

        try{
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(className() + "Trying to load text based environment from " + fileName + "... ");
            }
            BufferedReader inFile = new BufferedReader(new FileReader(file));

            int rows = Integer.parseInt(inFile.readLine());
            int columns = Integer.parseInt(inFile.readLine());
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(columns + " columns, " + rows + " rows.");
            }

            if(!checkDimensions(rows, columns))
                return null;

            int offsetX = (Constants.MAX_COLS - columns) / 2;
            int offsetY = (Constants.MAX_ROWS - rows) / 2;

            Environment env = new Environment(rows, columns);
            int currValue = 0;

            for(int i=0; i<rows; i++){
                for(int j=0; j<columns; j++){
                    currValue = (char)inFile.read() - '0';
                    switch (currValue) {
                        case 0: env.setStatus(offsetX+j, offsetY+i, Environment.Status.unexplored); break;
                        case 1: env.setStatus(offsetX+j, offsetY+i, Environment.Status.explored); break;
                        case 2: env.setStatus(offsetX+j, offsetY+i, Environment.Status.slope); break;
                        case 3: env.setStatus(offsetX+j, offsetY+i, Environment.Status.hill); break;
                        case 4: env.setStatus(offsetX+j, offsetY+i, Environment.Status.obstacle); break;
                        case 5: env.setStatus(offsetX+j, offsetY+i, Environment.Status.barrier); break;
                    }
                    /*if(currValue == 1)
                        env.setStatus(offsetX+j, offsetY+i, Environment.Status.obstacle);
                    else
                        env.setStatus(offsetX+j, offsetY+i, Environment.Status.unexplored);*/
                }

                // If file has been saved in unix/mac, only LF at end of line
                if(inFile.read() == 10)
                    continue;
                else   // if file has been saved in windows, additional CR exists
                    inFile.read();
            }

            inFile.close();

            //Temp for debugging:
            //saveWallConfig(fileName + "2");
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(className() + "Environment loaded successfully.");
            }
            return env;
        }
        catch (IOException e) {
            System.err.println(className() + "Error: could not read data from " + fileName);
        }
        catch (NumberFormatException e) {
            System.err.println(className() + "Error: incorrect data format in file " + fileName);
        }

        return null;
    }

    public static Environment loadWallConfig_ImageBased(String fileName) {
        File file = new File(fileName);

        if (!file.exists()) {
            return null;
        }

        try {
            BufferedImage image = ImageIO.read(file);
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(className() + "Trying to load image based environment from " + fileName + "... ");
            }
            //Raster raster = image.getRaster();

            int columns = image.getWidth();
            int rows = image.getHeight();
            if (Constants.DEBUG_OUTPUT) {
                System.out.println(columns + " columns, " + rows + " rows.");
            }

            if (!checkDimensions(rows, columns)) {
                return null;
            }

            int offsetX = (Constants.MAX_COLS - columns) / 2;
            int offsetY = (Constants.MAX_ROWS - rows) / 2;

            Environment env = new Environment(Constants.MAX_ROWS, Constants.MAX_COLS);
            if (image.getColorModel().getNumColorComponents() == 1) { //GreyScale-Hack
                Raster raster = image.getRaster();
                for (int i = 0; i < columns; i++) {
                    for (int j = 0; j < rows; j++) {
                        if (raster.getSample(i, j, 0) == 0) {
                            env.setStatus(offsetX + i, offsetY + j, Environment.Status.barrier);
                        } else {
                            env.setStatus(offsetX + i, offsetY + j, Environment.Status.unexplored);
                        }
                    }
                }
            } else {
                for (int i = 0; i < columns; i++) {
                    for (int j = 0; j < rows; j++) {
                        Color c = new Color(image.getRGB(i, j));
                        if(c.equals(Color.BLACK)){
                                env.setStatus(offsetX + i, offsetY + j, Environment.Status.barrier);
                                //System.out.println("BLACK");
                        } else if(c.equals(Color.WHITE))
                                env.setStatus(offsetX + i, offsetY + j, Environment.Status.unexplored);
                        else if(c.equals(new Color(255,69,0)))
                                env.setStatus(offsetX + i, offsetY + j, Environment.Status.hill);
                        else if(c.equals(Color.YELLOW))
                                env.setStatus(offsetX + i, offsetY + j, Environment.Status.slope);
                        else if(c.equals(Color.RED))
                                env.setStatus(offsetX + i, offsetY + j, Environment.Status.obstacle);
                        else {
                                env.setStatus(offsetX + i, offsetY + j, Environment.Status.unexplored);
                        }
                    }
                }
            }
            /*if(raster.getSample(i, j, 0) == 0)
                        env.setStatus(offsetX+i, offsetY+j, Environment.Status.barrier);
                    else
                        env.setStatus(offsetX+i, offsetY+j, Environment.Status.unexplored);*/

            if (Constants.DEBUG_OUTPUT) {
                System.out.println(className() + "Environment loaded successfully.");
            }
            return env;
        } catch (IOException e) {
            System.err.println(className() + "Error: could not read data from " + fileName);
        } catch (NumberFormatException e) {
            System.err.println(className() + "Error: incorrect data format in file " + fileName);
        }
        return null;
    }

    public static Environment loadWallConfig(String fileName) {
        String extension = fileName.substring(fileName.lastIndexOf('.')+1, fileName.length());

        if(extension.equals("txt"))
            return loadWallConfig_TextBased(fileName);
        else
            return loadWallConfig_ImageBased(fileName);
    }
    
// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="Utility">


    private static boolean checkDimensions(int rows, int cols) {
       if(rows > Constants.MAX_ROWS) {
            JOptionPane.showMessageDialog(new JFrame(), "Input image height is too large!  Maximum height is " + Constants.MAX_ROWS + ".", "Input Image Error", JOptionPane.ERROR_MESSAGE);
            return false;
        }

        if(cols > Constants.MAX_COLS) {
            JOptionPane.showMessageDialog(new JFrame(), "Input image width is too large!  Maximum width is " + Constants.MAX_COLS + ".", "Input Image Error", JOptionPane.ERROR_MESSAGE);
            return false;
        }

        return true;
    }
    
    private static String className() {
        return ("[EnvLoader] ");
    }
// </editor-fold>
    
}
