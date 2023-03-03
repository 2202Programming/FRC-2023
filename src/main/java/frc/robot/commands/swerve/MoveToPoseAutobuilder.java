// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;

public class MoveToPoseAutobuilder extends CommandBase {
  /** Creates a new MoveToPoseAutobuilder. */

  PathConstraints constraints;
  Pose2d finalPoint;
  SwerveDrivetrain sdt;

  public MoveToPoseAutobuilder(PathConstraints constraints, Pose2d finalPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.constraints = constraints;
    this.finalPoint = finalPoint;
    sdt = RobotContainer.RC().drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //create a path from current position to finalPoint
    PathPlannerTrajectory newPath = PathPlanner.generatePath(constraints, 
      new PathPoint(sdt.getPose().getTranslation(), finalPoint.getRotation(), sdt.getPose().getRotation()), 
      new PathPoint(finalPoint.getTranslation(), finalPoint.getRotation(), finalPoint.getRotation()));
    
    //execute command to follow path.
    RobotContainer.RC().autoBuilder.followPath(newPath).execute();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sdt.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
