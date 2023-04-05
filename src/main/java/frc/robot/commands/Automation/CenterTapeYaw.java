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
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//rotates robot to center front reflective tape target

public class CenterTapeYaw extends CommandBase {
  final SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;
  final Limelight_Subsystem ll;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  SwerveModuleState[] output_states;

  // PID for retroreflective-based heading to a target
  PIDController tapePid;
  double tape_kP = 2.0;
  double tape_kI = 0.0;
  double tape_kD = 0.0;
  double tapePidOutput = 0.0;

  Rotation2d currentAngle;
  double min_rot_rate = 6.0;        
 
  final double vel_tol = 2.0;
  final double pos_tol = 2.0;
  final double max_rot_rate = 60.0;  //[deg/s]

  private boolean control_motors;
  int frameCount;
  boolean lastValid = false;
  boolean currentValid = false;
  double goalYaw;

  /** Creates a new CenterTapeYaw. */
  public CenterTapeYaw(boolean control_motors, double goalYaw) {
    this.goalYaw = goalYaw;
    this.control_motors = control_motors;
    this.drivetrain = RobotContainer.RC().drivetrain;
    if (control_motors) {
      addRequirements(drivetrain);
    }
    this.kinematics = drivetrain.getKinematics();
    this.ll = RobotContainer.RC().limelight;

    tapePid = new PIDController(tape_kP, tape_kI, tape_kD);
    tapePid.setTolerance(pos_tol, vel_tol);
  }

  public CenterTapeYaw(){
    this(true, -26.8);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("***Starting Tape Correction, current ll X:"+ll.getX() + ", LL valid=" + ll.valid());
    frameCount = 0;
    tapePid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // frameCount++;
    // if(!ll.valid()) return;
    frameCount++;
    lastValid = currentValid;
    currentValid = ll.valid();
    if(!lastValid && currentValid) System.out.println("***LL became valid at frame# " + frameCount);
    calculate();
    if(control_motors && frameCount > 1){ //wait for LL to get target
      drivetrain.drive(output_states);
    }
  }

  void calculate(){
    double Yaw = ll.getX();
    tapePidOutput = tapePid.calculate(Yaw, goalYaw);//goal yaw is centered - 12.7 deg since ll offset
    double yError = tapePid.getPositionError();
    double min_rot =  Math.signum(tapePidOutput) * min_rot_rate;
    rot = MathUtil.clamp(tapePidOutput + min_rot, -max_rot_rate, max_rot_rate) / 57.3;   //clamp in [deg/s] convert to [rad/s]
    currentAngle = drivetrain.getPose().getRotation();
    output_states = kinematics
          .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rot, currentAngle));

    SmartDashboard.putNumber("tapePidOutput", tapePidOutput);
    SmartDashboard.putNumber("rot", rot * 57.3);
    SmartDashboard.putNumber("Yaw Error", yError);   
    SmartDashboard.putBoolean("LL Valid", ll.valid());
    SmartDashboard.putNumber("Framecount", frameCount); 
  }

  public double getRot(){
    return rot;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    System.out.println("***Ending Tape Correction, interrupted?" + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isReady();
  }

  public boolean isReady() {
    return (tapePid.atSetpoint());
  }

}
