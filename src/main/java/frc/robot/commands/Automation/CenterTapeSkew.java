// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveDrivetrain;

//translates robot to get both reflective tape targets aligned to same yaw

public class CenterTapeSkew extends CommandBase {
  final SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;
  final PhotonVision photonvision;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  SwerveModuleState[] output_states;

  // PID for retroreflective-based heading to a target
  PIDController tapePid;
  double tape_kP;
  double tape_kI;
  double tape_kD;
  double tapePidOutput = 0.0;

  Rotation2d currentAngle;
  double min_rot_rate = 6.0;        //about 7.5 deg is min we measured
  double r_min_rot_rate = min_rot_rate;

  final double vel_tol = 10.0;
  final double pos_tol = 2.0;
  final double max_rot_rate = 60.0;  //[deg/s]

  /** Creates a new CenterTapeYaw. */
  public CenterTapeSkew() {
    this.drivetrain = RobotContainer.RC().drivetrain;
    addRequirements(drivetrain);
    this.kinematics = drivetrain.getKinematics();
    this.photonvision = RobotContainer.RC().photonVision;

    tapePid = new PIDController(tape_kP, tape_kI, tape_kD);
    tapePid.setTolerance(pos_tol, vel_tol);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculate();
    drivetrain.drive(output_states);
  }

  void calculate(){
    if(photonvision.getNumberOfTapeTargets()>1) { //can't run if 2 tape targets aren't seen
      double targetSkewError = photonvision.getLargestTapeTarget().getYaw() - photonvision.getSecondLargestTapeTarget().getYaw();
      tapePid.setSetpoint(0.0); //target skew is both targets have same yaw
      tapePidOutput = tapePid.calculate(targetSkewError);

      double min_rot = (Math.abs(targetSkewError) > pos_tol)  ? - Math.signum(targetSkewError) * min_rot_rate : 0.0;
      xSpeed = MathUtil.clamp(tapePidOutput + min_rot, -max_rot_rate, max_rot_rate) / 57.3;   //clamp in [deg/s] convert to [rad/s]

      currentAngle = drivetrain.getPose().getRotation();
      output_states = kinematics
          .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 0, 0, currentAngle));
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isReady();
  }

  public boolean isReady() {
    return tapePid.atSetpoint();
  }

}
