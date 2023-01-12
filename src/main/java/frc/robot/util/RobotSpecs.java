// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;
import frc.robot.Constants.ChassisConfig;
import frc.robot.Constants.SubsystemConfig;
import frc.robot.Constants.WheelOffsets;

/** Add your docs here. */



public class RobotSpecs {

    public enum RobotNames {
        SwerveBot("SwerveBot"), 
        CompetitionBot("CompetitionBot"),
        UnknownBot("UnknownBot");
        String name;
    
        private RobotNames(String name) {
            this.name = name;
        }
    
        public String toString() {
            return name;
        }
      }
    
    private RobotNames myRobotName;
    private WheelOffsets myWheelOffsets;
    private ChassisConfig myChassisConfig;
    private SubsystemConfig mySubsystemConfig;

    public RobotSpecs(String serialNo){
        myRobotName = getRobotName(serialNo);

        switch(myRobotName){
            case SwerveBot:
                myWheelOffsets = Constants.DriveTrain.swerveBotOffsets;
                myChassisConfig = Constants.DriveTrain.swerveBotChassisConfig;
                mySubsystemConfig = Constants.swerveBotSubsystemConfig;
                break;
            case CompetitionBot:
                myWheelOffsets = Constants.DriveTrain.compBotOffsets;
                myChassisConfig = Constants.DriveTrain.compBotChassisConfig;
                mySubsystemConfig = Constants.compBotSubsystemConfig;
                break;
            case UnknownBot:
                myWheelOffsets = Constants.DriveTrain.swerveBotOffsets;
                myChassisConfig = Constants.DriveTrain.swerveBotChassisConfig;
                mySubsystemConfig = Constants.swerveBotSubsystemConfig;
                System.out.println("***ERROR, bot serial unknown. Using SwerveBot Config***");
                break;
        }
    }

  public WheelOffsets getWheelOffset(){
      return myWheelOffsets;
  }

  public ChassisConfig getChassisConfig(){
      return myChassisConfig;
  }

  public SubsystemConfig getSubsystemConfig(){
      return mySubsystemConfig;
  }
  
  //takes the roborio serial # and returns the robot name
  public RobotNames getRobotName(String serialNo){
    RobotNames tempRobotName;

    if (serialNo == null) 
        return RobotNames.UnknownBot;

    if (serialNo.compareTo("031b7511")==0)
        tempRobotName = RobotNames.SwerveBot;
    if (serialNo.compareTo("03238151")==0)
        tempRobotName = RobotNames.CompetitionBot;
    else tempRobotName = RobotNames.UnknownBot;

    System.out.println("***RoboRio SERIAL NUM: " + serialNo);
    System.out.println("***Robot identified as: " + tempRobotName);
    return tempRobotName;
  }
}