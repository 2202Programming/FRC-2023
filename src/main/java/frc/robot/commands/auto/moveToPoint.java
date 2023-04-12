// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.RobotContainer;
import frc.robot.commands.JoystickRumbleEndless;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.util.PoseMath;

public class moveToPoint extends CommandBase {
  /** Creates a new goToScoringPosition. */

  PathConstraints constraints;
  SwerveDrivetrain sdt;
  PPSwerveControllerCommand pathCommand;
  JoystickRumbleEndless rumbleCmd;
  Pose2d targetPose;


  /**
   * Constructs a moveToPoint
   * @param constraints path constraints
   * @param targetPose target pose2D
   */
  public moveToPoint(PathConstraints constraints, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.constraints = constraints;
    this.targetPose = targetPose;
    sdt = RobotContainer.RC().drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    pathCommand = MoveToPoseAutobuilder(constraints, targetPose);
    sdt.disableVisionPose();
    rumbleCmd = new JoystickRumbleEndless(Id.Operator);
    //rumbleCmd.schedule();
    RobotContainer.RC().lights.setBlinking(BlinkyLights.GREEN);
    pathCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
    sdt.stop();
    sdt.enableVisionPose();
    sdt.disableVisionPoseRotation();
    rumbleCmd.cancel();
    RobotContainer.RC().lights.stopBlinking();
    RobotContainer.RC().lights.setAllianceColors();
    System.out.println("***Final End Point:" + sdt.getPose().getTranslation() + ", rot:" + sdt.getPose().getRotation().getDegrees());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }

  public PPSwerveControllerCommand MoveToPoseAutobuilder(PathConstraints constraints, Pose2d finalPose) {
    //takes contraints, final pose - returns a command to move from current post to final pose

    Rotation2d bearing = PoseMath.getHeading2Target(sdt.getPose(), finalPose); //direction directly from point A to B.
    System.out.println("***BEARING = " + bearing);
    //using bearing as your exit and entry angle
    PathPoint startPoint = new PathPoint(sdt.getPose().getTranslation(), bearing, sdt.getPose().getRotation());
    PathPoint endPoint = new PathPoint(finalPose.getTranslation(), bearing, finalPose.getRotation());
    System.out.println("From Point:" + sdt.getPose().getTranslation() + ", rot:" + sdt.getPose().getRotation().getDegrees());
    System.out.println("Expected End Point:" + finalPose.getTranslation()  + ", rot:" + finalPose.getRotation().getDegrees());
    
    PIDController anglePid = new PIDController(6.0, 0, 0);
    anglePid.setTolerance(Math.PI * 0.02);
    anglePid.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xPid = new PIDController(6.0, 0, 0);
    PIDController yPid = new PIDController(6.0, 0, 0);
    xPid.setTolerance(0.01);
    yPid.setTolerance(0.01);

    //create a path from current position to finalPoint
    PathPlannerTrajectory newPath = PathPlanner.generatePath(constraints, startPoint, endPoint);
    System.out.println("Path time: " + newPath.getTotalTimeSeconds());
    
    PPSwerveControllerCommand pathCommand = new PPSwerveControllerCommand(
      newPath, 
      sdt::getPose, // Pose supplier
      sdt.getKinematics(), // SwerveDriveKinematics
      xPid, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      yPid, // Y controller (usually the same values as X controller)
      anglePid, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      sdt::drive, // Module states consumer
      false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      sdt // Requires this drive subsystem
    );
    return pathCommand;
  }
}
