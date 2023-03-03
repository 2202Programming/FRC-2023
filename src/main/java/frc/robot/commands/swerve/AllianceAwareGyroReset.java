// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class AllianceAwareGyroReset extends InstantCommand {
  private boolean disableVisionPoseRotation;


  //resets the robot rotation/gyro, assuming robot is facing AWAY from driver, and will use alliance
  //to determine if that means 0 or 180 degrees.
  //Also has option to disable future vision rotation updates (and just rely on gryo going forward)
  public AllianceAwareGyroReset(boolean disableVisionRotation) {
    this.disableVisionPoseRotation = disableVisionRotation;
  }

  public AllianceAwareGyroReset(){
    this(true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance() == Alliance.Blue){
      RobotContainer.RC().drivetrain.resetAnglePose(Rotation2d.fromDegrees(0)); //away from blue driverstation is 0 degrees
    }
    else{
      RobotContainer.RC().drivetrain.resetAnglePose(Rotation2d.fromDegrees(180)); //away from red driverstation is 180 degrees
    }
    if(disableVisionPoseRotation){
      RobotContainer.RC().drivetrain.disableVisionPoseRotation();
    }
  }
}
