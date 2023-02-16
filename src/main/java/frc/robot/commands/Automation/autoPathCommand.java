// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;

public class autoPathCommand extends CommandBase {

  SwerveDrivetrain drivetrain;
  String pathname;
  double maxVelocity;
  double maxAcceleration;

  //builds an executes an autopath when given the path name
  public autoPathCommand(String pathname, double maxVelocity, double maxAcceleration) {
    this.drivetrain = RobotContainer.RC().drivetrain;
    this.pathname = pathname;
    this.maxAcceleration = maxAcceleration;
    this.maxVelocity = maxVelocity;
  }

  public autoPathCommand(String pathname) {
    this(pathname, 2, 3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(pathname, new PathConstraints(maxVelocity, maxAcceleration));  //5,3 tested and ok
    RobotContainer.RC().autoBuilder.fullAuto(pathGroup).schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}