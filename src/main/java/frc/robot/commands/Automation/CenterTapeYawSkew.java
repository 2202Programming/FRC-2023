// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;

public class CenterTapeYawSkew extends Command {
  /** Creates a new CenterTapeYawSkew. */

  CenterTapeYaw yawCommand = new CenterTapeYaw();
  CenterTapeSkew skewCommand = new CenterTapeSkew(false);
  SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;

  public CenterTapeYawSkew() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = RobotContainer.RC().drivetrain;
    this.kinematics = drivetrain.getKinematics();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yawCommand.initialize();
    skewCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yawCommand.execute();
    skewCommand.execute();
    drivetrain.drive(kinematics
    .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, skewCommand.getYSpeed(), yawCommand.getRot(), 
    drivetrain.getPose().getRotation())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    yawCommand.end(interrupted);
    skewCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yawCommand.isFinished() && skewCommand.isFinished();
  }
}
