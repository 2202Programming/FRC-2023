// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.Constants.ChassisConfig;
import frc.robot.Constants.SubsystemConfig;
import frc.robot.Constants.WheelOffsets;

/** Add your docs here. */



public class RobotSpecs {

    public enum RobotNames {
        SwerveBot("SwerveBot"), 
        CompetitionBot("CompetitionBot"),
        ChadBot("ChadBot"),
        UnknownBot("UnknownBot"),
        BotOnBoard("BotOnBoard");
        String name;
    
        private RobotNames(String name) {
            this.name = name;
        }
    
        public String toString() {
            return name;
        }
      }
    
    public RobotNames myRobotName;
    private WheelOffsets myWheelOffsets;
    private ChassisConfig myChassisConfig;
    private SubsystemConfig mySubsystemConfig;


    public RobotSpecs() {
        this(System.getenv("serialnum"));        
    }
    
    public RobotSpecs(String serialNo){
        myRobotName = getRobotName(serialNo);

        //if we are simulated, use the competionBot so we have everything
        if (RobotBase.isSimulation()) {
            myRobotName = RobotNames.CompetitionBot;
        }

        switch(myRobotName){
            case SwerveBot:
                myWheelOffsets = Constants.DriveTrain.swerveBotOffsets;
                myChassisConfig = Constants.DriveTrain.swerveBotChassisConfig;
                mySubsystemConfig = Constants.swerveBotSubsystemConfig;
                System.out.println("***I'm a Swervebot***");
                break;
            case CompetitionBot:
                myWheelOffsets = Constants.DriveTrain.chadBotOffsets;
                myChassisConfig = Constants.DriveTrain.chadBotChassisConfig;
                mySubsystemConfig = Constants.chadBotSubsystemConfig;
                System.out.println("***I'm a ChadBot***");
                break;
            case UnknownBot:
                myWheelOffsets = Constants.DriveTrain.swerveBotOffsets;
                myChassisConfig = Constants.DriveTrain.swerveBotChassisConfig;
                mySubsystemConfig = Constants.swerveBotSubsystemConfig;
                System.out.println("***ERROR, bot serial unknown. Using SwerveBot Config***");
                break;
            case BotOnBoard:
                myWheelOffsets = Constants.DriveTrain.swerveBotOffsets;
                myChassisConfig = Constants.DriveTrain.swerveBotChassisConfig;
                mySubsystemConfig = Constants.swerveBotSubsystemConfig;
                System.out.println("***ERROR,BotOnBoard don't expect too much***");
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
    else if (serialNo.compareTo("03238151")==0)
        tempRobotName = RobotNames.ChadBot;
    else if (serialNo.compareTo("0312db1a")==0)
        tempRobotName = RobotNames.BotOnBoard;
    else if (serialNo.compareTo("293e833")==0) //TODO: Fix serial number
        tempRobotName = RobotNames.CompetitionBot;
    else tempRobotName = RobotNames.UnknownBot;

    System.out.println("***RoboRio SERIAL NUM: " + serialNo);
    System.out.println("***Robot identified as: " + tempRobotName);
    return tempRobotName;
  }
}