// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import java.util.ArrayList;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;

public class autoCommand extends CommandBase {

  SwerveDrivetrain drivetrain;

  public autoCommand() {
    this.drivetrain = RobotContainer.RC().drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPose, // Pose2d supplier
      drivetrain::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
      drivetrain.getKinematics(), // SwerveDriveKinematics
      new PIDConstants(4.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(2.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      drivetrain::drive, // Module states consumer used to output to the drive subsystem
      RobotContainer.RC().eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
    );
    ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("AT Test", new PathConstraints(2, 3));  //5,3 tested and ok

    autoBuilder.fullAuto(pathGroup).schedule();

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
