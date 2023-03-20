// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;

public class RotateTo extends CommandBase {
  SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
  PIDController rotatePid;
  double rotate_kP = 4.0;
  double rotate_kI = 0.0;
  double rotate_kD = 0.2;
  double rotatePidOutput = 0.0;

  Rotation2d currentAngle;
  double min_rot_rate = 6.0;   
  double rot;
  
  final double vel_tol = 0.1; // [radians/s]
  final double pos_tol = 0.01;// in radians
  final double max_rot_rate = 1.0;  //[radians/s]

  private Rotation2d targetRotation;
  public RotateTo(Rotation2d targetRotation) {
    addRequirements(sdt);
    rotatePid = new PIDController(rotate_kP, rotate_kI, rotate_kD);
    rotatePid.setTolerance(pos_tol, vel_tol);
    rotatePid.enableContinuousInput(-Math.PI, Math.PI);
    this.targetRotation = targetRotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculate();
  }

  public void end(boolean interrupted) {
    sdt.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isReady();
  }

  public boolean isReady() {
    return rotatePid.atSetpoint();
  }
  void calculate(){
    currentAngle = sdt.getPose().getRotation();
    double targetYawError = currentAngle.getRadians();

    rotatePid.setSetpoint(targetRotation.getRadians()); //target heading in radians
    rotatePidOutput = rotatePid.calculate(targetYawError); // radians per second

   
    rot = MathUtil.clamp(rotatePidOutput, -max_rot_rate, max_rot_rate);
    
    
    sdt.drive(sdt.getKinematics().toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rot, currentAngle)));

  }
}
