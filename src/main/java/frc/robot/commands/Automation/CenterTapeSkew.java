// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveDrivetrain;

//translates robot to get both reflective tape targets aligned to same yaw

public class CenterTapeSkew extends Command {
  final SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;
  final PhotonVision photonvision;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  SwerveModuleState[] output_states;

  // PID for retroreflective-based heading to a target
  PIDController tapePid;
  double tape_kP = 0.3;
  double tape_kI = 0.0;
  double tape_kD = 0.0;
  double tapePidOutput = 0.0;

  Rotation2d currentAngle;
  double min_velocity = 0.1;        //not yet measured
  double degrees_to_meters_per_sec = 0.5;   //not yet measured

  final double vel_tol = 10.0; //m/s
  final double pos_tol = 0.4; //degrees error
  final double max_velocity = 2.0; //m/s

  LinearFilter filter;
  double filteredError;

  private boolean control_motors;

  /** Creates a new CenterTapeYaw. */
  public CenterTapeSkew(boolean control_motors) {
    this.control_motors = control_motors;
    this.drivetrain = RobotContainer.RC().drivetrain;
    if (control_motors){
      addRequirements(drivetrain);
    }
    this.kinematics = drivetrain.getKinematics();
    this.photonvision = RobotContainer.RC().photonVision;

    tapePid = new PIDController(tape_kP, tape_kI, tape_kD);
    tapePid.setTolerance(pos_tol, vel_tol);

  
    filter = LinearFilter.movingAverage(3);
  }

  public CenterTapeSkew(){
    this(true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(control_motors){
      currentAngle = drivetrain.getPose().getRotation();
      output_states = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, currentAngle));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calculate();
    if (control_motors){
      drivetrain.drive(output_states);
    }
  }

  void calculate(){
    if(photonvision.getNumberOfTapeTargets()>1) { //can't run if 2 tape targets aren't seen
      double targetSkewError = filter.calculate(photonvision.getSecondLargestTapeTarget().getYaw() - photonvision.getLargestTapeTarget().getYaw());
      tapePid.setSetpoint(0.0); //target skew is both targets have same yaw
      tapePidOutput = tapePid.calculate(targetSkewError);

      tapePidOutput *= degrees_to_meters_per_sec; //PID output is in degrees error between two targets, need to convert to m/s for drivetrain to translate.

      double min_yspeed = (Math.abs(targetSkewError) > pos_tol)  ? - Math.signum(targetSkewError) * min_velocity : 0.0;
      ySpeed = MathUtil.clamp(tapePidOutput + min_yspeed, -max_velocity, max_velocity);   //clamp in m/s

      if(control_motors) {
        currentAngle = drivetrain.getPose().getRotation();
        output_states = kinematics
            .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, xSpeed, 0, currentAngle));
      }
      SmartDashboard.putNumber("Skew Pid Output", tapePidOutput);
      SmartDashboard.putNumber("Target Skew", targetSkewError);    
    }
  } 

  public double getYSpeed(){
    return ySpeed;
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
