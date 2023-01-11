// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ux;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.stream.Stream;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class AutoPaths {
 // where to look
 final String PathsDir = Filesystem.getDeployDirectory() + File.separator + "pathplanner";
  
 // what we find
 SendableChooser<String> pathChooser = new SendableChooser<>();
 Map<String, Trajectory> m_map = new LinkedHashMap<>();
 
 SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();
 
 boolean auto_default_cmd_set = false;

 public AutoPaths(ShuffleboardTab tab) {

   // create a default trajectory that does nothing 
   SendableRegistry.setName(pathChooser, "PathChooser");
   pathChooser.setDefaultOption("do nothing", null);

   try {
     // fills chooser from directory
     readPaths();         
   } catch (IOException e) {
     e.printStackTrace();
     //OK to continune, it was some IO error, just eat it.
   }

   // put the chooser on the tab we were given 
   SendableRegistry.setName(autoCommandChooser,"AutoChooser");
   tab.getLayout("AutoPath", BuiltInLayouts.kList).withSize(2, 2).add(pathChooser);
   tab.getLayout("Autonomous Command", BuiltInLayouts.kList).withSize(3, 2).add(autoCommandChooser);

 }

 public SendableChooser<String> getChooser() { return pathChooser;}

 public Command getAutonomousCommand() {
   return autoCommandChooser.getSelected();
 }

 public void addAutoCommand(String name, Command cmd) {
   if (auto_default_cmd_set) {
     autoCommandChooser.addOption(name, cmd);
   }
     else {
       // first command added gets to be the default
       autoCommandChooser.setDefaultOption(name, cmd);
       auto_default_cmd_set = true;
     }
 }

 /**
  * Reads the selected trajectory file and retrns parsed object.
  * Null if "do nothing" is selected.
  * 
  * @return Trajectory selected 
  */
 //public Trajectory get() { return  pathChooser.getSelected();   }

 void readPaths() throws IOException {
   Stream<Path> paths = Files.walk(Paths.get(PathsDir));
   paths.filter(Files::isRegularFile).forEach(this::buildEntry);      
   paths.close();
 }
 
 void buildEntry(Path file) {
   Path fn = file.getName(file.getNameCount()-1);
   String key = fn.toString().split("\\.")[0]; 
   System.out.println("**Key:" + key);
   PathPlannerTrajectory traj = loadTrajectory(key);
   pathChooser.addOption(key, key);
   
   // keep a map so we can access any path we need
   m_map.put(key, traj);
 }

 PathPlannerTrajectory loadTrajectory(String filename) {
   try {
     return PathPlanner.loadPath(filename, 1, 1);
   } catch (Exception ex) {
     DriverStation.reportError("Unable to open trajectory: " + filename, ex.getStackTrace());
   }
   return null;
 }

 public Trajectory get(String trajName) {
   return m_map.get(trajName);
 }

}