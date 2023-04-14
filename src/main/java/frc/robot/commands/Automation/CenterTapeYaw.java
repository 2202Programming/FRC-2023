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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//rotates robot to center front reflective tape target

public class CenterTapeYaw extends CommandBase {
  final SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;
  final Limelight_Subsystem ll;
  final PhotonVision photonVision;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  SwerveModuleState[] vision_out;
  ChassisSpeeds zero_cs = new ChassisSpeeds(0.0, 0.0, 0.0);
  SwerveModuleState[] no_turn_states;

  public enum PhotoreflectiveMethod {
    Photonvision, Limelight
  }

  private PhotoreflectiveMethod PRmethod;

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
  final double max_rot_rate = 60.0; // [deg/s]

  final double high_tape_Y = 21.0;
  final double mid_tape_Y = 0.0;
  final double high_tape_goal = -16.0;
  final double mid_tape_goal = -24.6;
  final double max_yaw_error = 15.0; // max number of degrees the target can be off and we still think it's legit

  private boolean control_motors;
  int frameCount;
  Timer timer = new Timer();
  final double timeoutSeconds;
  boolean lastValid = false;
  boolean currentValid = false;
  boolean highYaw = false;
  double goalYaw;
  int validCount;
  int sequentialBadFrames;

  /**
   * Creates a new CenterTapeYaw.
   * 
   * @param control_motors if the motors should be controlled
   * @param goalYaw        goal X error
   * @param timeoutSeconds seconds until timeout of run
   * @param PRmedthod      enum for if photonvision or Limelight should be used
   * 
   */
  public CenterTapeYaw(boolean control_motors, double timeoutSeconds, PhotoreflectiveMethod PRmethod) {
    this.control_motors = control_motors;
    this.timeoutSeconds = timeoutSeconds;
    this.drivetrain = RobotContainer.RC().drivetrain;
    this.photonVision = RobotContainer.RC().photonVision;
    this.PRmethod = PRmethod;
    this.kinematics = drivetrain.getKinematics();
    this.ll = RobotContainer.RC().limelight;

    if (control_motors) {
      addRequirements(drivetrain);
    }
    no_turn_states = kinematics.toSwerveModuleStates(zero_cs);
    tapePid = new PIDController(tape_kP, tape_kI, tape_kD);
    tapePid.setTolerance(pos_tol, vel_tol);
  }

  public CenterTapeYaw() {
    this(true, 2.0, PhotoreflectiveMethod.Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    if (PRmethod == PhotoreflectiveMethod.Limelight)
      System.out.println("***Starting LL Tape Correction, current ll X:" + ll.getX() + ", LL valid=" + ll.valid());
    else
      System.out.println("***Starting PV Tape Correction, current PV X:" + photonVision.getLargestTapeTarget().getYaw()
          + ", PV valid=" + photonVision.hasTapeTarget());
    frameCount = 0;
    validCount = 0;
    sequentialBadFrames = 0;
    tapePid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean valid_tape = false;
    SwerveModuleState[] out;
    frameCount++;
    switch (PRmethod) {
      case Photonvision:
        lastValid = currentValid;
        currentValid = photonVision.hasTapeTarget();
        if (!lastValid && currentValid)
          System.out.println("***PV became valid at frame# " + frameCount);
        if (currentValid) {
          calculatePV();
          valid_tape = true;
        }
        break;
      default:
      case Limelight:
        lastValid = currentValid;
        currentValid = ll.valid();
        validCount = (currentValid) ? validCount + 1 : 0;
        if (!lastValid && currentValid)
          System.out.println("***LL became valid at frame# " + frameCount);
        if (lastValid && !currentValid)
          System.out.println("***LL became invalid at frame# " + frameCount);
        if (validCount > 5) {
          calculateLL();
          valid_tape = true;
        }
        if(!currentValid || highYaw) {
          sequentialBadFrames++;
          System.out.println("Sequental bad frame #" + sequentialBadFrames + ", valid="+currentValid+", highYaw="+highYaw);
        } else {
          sequentialBadFrames = 0;
        }
        break;
    }

    // pick our pid output or no motion output based on valid_tape
    out = (valid_tape) ? vision_out : no_turn_states;
    if (control_motors) {
      drivetrain.drive(out);
    }

  }

  void calculateLL() {
    double Yaw = ll.getX();
    double Elevation = ll.getY();

    //Is measured target elevation closer to what we expect for the high tape (back pole) or mid tape?
    //Different X goals depending which target is being visualized
    if(Math.abs(high_tape_Y - Elevation) > Math.abs(mid_tape_Y - Elevation)){ //true means elevation closer to mid tape
      goalYaw = mid_tape_goal;
    }
    else {
      goalYaw = high_tape_goal;
    }
    tapePidOutput = tapePid.calculate(Yaw, goalYaw); //set measurement AND goal in one foul swoop
    double yawError = tapePid.getPositionError();
    double min_rot = Math.signum(tapePidOutput) * min_rot_rate;
    rot = MathUtil.clamp(tapePidOutput + min_rot, -max_rot_rate, max_rot_rate) / 57.3; // clamp in [deg/s] convert to
                                                                                       // [rad/s]
    currentAngle = drivetrain.getPose().getRotation();
    vision_out = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rot, currentAngle));

    //if our yaw error is too big maybe we are seeing the next pole over, so don't move.
    if (Math.abs(yawError) > max_yaw_error){ 
      highYaw = true;
      vision_out = no_turn_states;
    } else {
      highYaw = false;
    }

    SmartDashboard.putNumber("tapePidOutput", tapePidOutput);
    SmartDashboard.putNumber("rot", rot * 57.3);
    SmartDashboard.putNumber("LL Yaw Error", yawError);
    SmartDashboard.putBoolean("LL Valid", ll.valid());
    SmartDashboard.putNumber("Framecount", frameCount);
  }

  void calculatePV() {
    double Yaw = photonVision.getLargestTapeTarget().getYaw();
    tapePidOutput = tapePid.calculate(Yaw, goalYaw);// goal yaw is centered - 12.7 deg since ll offset
    double yError = tapePid.getPositionError();
    double min_rot = Math.signum(tapePidOutput) * min_rot_rate;
    rot = MathUtil.clamp(tapePidOutput + min_rot, -max_rot_rate, max_rot_rate) / 57.3; // clamp in [deg/s] convert to
                                                                                       // [rad/s]
    currentAngle = drivetrain.getPose().getRotation();
    vision_out = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, rot, currentAngle));

    SmartDashboard.putNumber("tapePidOutput", tapePidOutput);
    SmartDashboard.putNumber("rot", rot * 57.3);
    SmartDashboard.putNumber("PV Yaw Error", yError);
    SmartDashboard.putNumber("Framecount", frameCount);
  }

  public double getRot() {
    return rot;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    timer.stop();
    System.out.println("***Ending Tape Correction, interrupted?" + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isReady() || timer.hasElapsed(timeoutSeconds);
  }

  public boolean isReady() {
    return (tapePid.atSetpoint());
  }

}
