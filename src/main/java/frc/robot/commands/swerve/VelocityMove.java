// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;

public class VelocityMove extends CommandBase {
  final SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
  final double x_speed;
  final double y_speed;
  final double duration;
  SwerveModuleState[] states;
  final Timer runTime = new Timer();

  // speeds [m/s]  duration [s]
  public VelocityMove(double x_speed, double y_speed, double duration) {
    this.x_speed = x_speed;
    this.y_speed = y_speed;
    this.duration = duration;
    addRequirements(sdt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    runTime.reset();
    runTime.start();
    // compute our drive vector and let it run
    states = sdt.getKinematics().toSwerveModuleStates(new ChassisSpeeds(x_speed, y_speed, 0.0));
    System.out.println("&&&&&&&&&&&--Init finished");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sdt.drive(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    runTime.stop();
    sdt.stop();
    System.out.println("&&&&&&&&&&&--Ended, interrupted " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return runTime.hasElapsed(duration);
    }
}
