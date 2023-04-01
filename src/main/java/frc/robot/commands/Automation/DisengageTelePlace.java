// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmLockForDrivingBS;
import frc.robot.commands.Arm.ArmLockForDrivingFS;
import frc.robot.commands.auto.moveToPoint;

public class DisengageTelePlace extends SequentialCommandGroup {

  /**
   * Constructs a DisengageTelePlace
   * @param constraints path constraints
   * @param distance distance to creep away from scoring station to make room for arm retraction (meters)
   */
   public DisengageTelePlace(PathConstraints pathConstraints, double distance) {
    Pose2d targetPose;
    Pose2d currentPose = RobotContainer.RC().drivetrain.getPose();

    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) { //BLUE ALLIANCE
      targetPose = new Pose2d(currentPose.getX() + distance, currentPose.getY(), currentPose.getRotation()); //20 cm away from scoring station, blue side
    }
    else{
      targetPose = new Pose2d(currentPose.getX() - distance, currentPose.getY(), currentPose.getRotation()); //20 cm away from scoring station, red side
    }

   addCommands(
            new moveToPoint(pathConstraints, targetPose),
            new ArmLockForDrivingFS()
        );
  }
}
