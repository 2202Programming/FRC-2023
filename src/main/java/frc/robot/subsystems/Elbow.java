// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//TODO: Gear ratios, outer loop (position),
package frc.robot.subsystems;

import frc.robot.Constants.CAN;
import frc.robot.util.PIDFController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Elbow extends SubsystemBase {
  double velCmd; //calculated
  double posCmd; //calculated
  static int PosSlot = 0;
  static double maxVel = 5.0;   //[deg/s]
  static double maxAccel = 5.0;  //[deg/s^2]
  final double gearRatio = (1.0 / 175.0);

  // at position limits
  static double posLimit = 3.0; //[deg]
  static double velLimit = 5.0; //[deg/s]

  /** Creates a new Elbow. */
  final CANSparkMax ctrl;
  final SparkMaxPIDController pid;
  final RelativeEncoder encoder;

  PIDController positionPID = new PIDController(5.0, 0.150, 0.250); // TODO: Get actual values, just what arm values are rn
  // Use vel loop for HW
  PIDFController hwPID = new PIDFController(0.002141, 0.00005, 0.15, 0.05017); // TODO (It's what arm values are rn, will need to change)

  // measurements
  double currentPos;
  double desiredPos;
  double currentVel;
  double desiredVel;

  public Elbow() {
    ctrl = new CANSparkMax(CAN.LEFT_ELBOW, MotorType.kBrushless);
    ctrl.restoreFactoryDefaults();
    ctrl.setIdleMode(CANSparkMax.IdleMode.kBrake);
    pid = ctrl.getPIDController();
    encoder = ctrl.getEncoder();
    encoder.setPositionConversionFactor(gearRatio * 360.0);   //[deg]
    encoder.setVelocityConversionFactor(gearRatio * 360.0 / 60.0);   //[deg/s]
    // configure the hardware pid
    hwPID.copyTo(pid, 0);
    pid.setSmartMotionMaxVelocity(maxVel, PosSlot);
    pid.setSmartMotionMaxAccel(maxAccel, PosSlot);

    //safety and power limits
    ctrl.setSmartCurrentLimit(10, 10,  10); //stallLimit, int freeLimit, int limitRPM);
    ntcreate();
  }

  // @Override
  public void periodic(double velAdjust) {
   //measure 
   currentPos = encoder.getPosition();   // [deg]
   posCmd = MathUtil.clamp(positionPID.calculate(currentPos) + velAdjust, -maxVel, maxVel);
   velCmd = positionPID.atSetpoint() ? 0.0 : velCmd;
   pid.setReference(velCmd, ControlType.kVelocity);
   currentVel = encoder.getVelocity();
    if(currentVel > maxVel){
      currentVel = maxVel;
    }
    if(-currentVel < -maxVel){
      currentVel = -maxVel;
    }
  }

  public boolean isAtPosition() {
    double posError = Math.abs(currentPos - desiredPos);
    return  ((posError < posLimit) && 
            (Math.abs(currentVel) < velLimit));
  }

  public void setPosition(double degrees) {
    this.desiredPos = degrees;
    // send the desired pos to the hardware controller
    pid.setReference(desiredPos, ControlType.kSmartMotion);
  } 
  public double getPosition() {
    return currentPos;
  }

  public void setVelocity(double vel){
    this.desiredVel = vel;
    pid.setReference(desiredVel, ControlType.kVelocity); //smart or no?
  }

  public double getVelocity(){
    return currentVel;
  }

  public double getMaxVel(){
    return maxVel;
  }
  public void setMaxVel(double vel){
    maxVel = Math.abs(vel);
  }
  public double getMaxAccel(){
    return maxAccel;
  }
  public void setMaxAccel(double a){
    maxAccel = Math.abs(a);
  }

  NetworkTable table = NetworkTableInstance.getDefault().getTable("elbow");
  NetworkTableEntry nt_currentPos;
  NetworkTableEntry nt_desiredPos;
  NetworkTableEntry nt_desiredVel;
  NetworkTableEntry nt_currentVel;
  public void ntcreate(){
    nt_currentPos = table.getEntry("Current Position");
    nt_currentVel = table.getEntry("Current Velocity");
    nt_desiredPos = table.getEntry("Desired Position");
    nt_desiredVel = table.getEntry("Desired Velocity");

    nt_currentPos.setDouble(currentPos);
    nt_currentVel.setDouble(currentVel);
    nt_desiredPos.setDouble(desiredPos);
    nt_desiredVel.setDouble(desiredVel);


  }
  public void ntupdates(){

  }

  
}
