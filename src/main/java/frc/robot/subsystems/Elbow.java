// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//TODO: Gear ratios, outer loop (position),
package frc.robot.subsystems;

import frc.robot.Constants.CAN;
import frc.robot.util.PIDFController;
import frc.robot.util.VelocityControlled;
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
import edu.wpi.first.wpilibj.Timer;

public class Elbow extends SubsystemBase implements VelocityControlled {
  double velCmd; //calculated
  double posCmd; //calculated
  boolean velocityMode;
  static int PosSlot = 0;
  static double maxVel = 5.0;   //[deg/s]
  static double maxAccel = 5.0;  //[deg/s^2]
  final double gearRatio = (360.0/175.0);

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
  double currentVel;
  double desiredVel;

  public Elbow() {
    ctrl = new CANSparkMax(CAN.ELBOW_Motor, MotorType.kBrushless);
    ctrl.clearFaults();
    ctrl.restoreFactoryDefaults();
    ctrl.setIdleMode(CANSparkMax.IdleMode.kCoast);
    pid = ctrl.getPIDController();
    encoder = ctrl.getEncoder();
    encoder.setPositionConversionFactor(gearRatio);   //[deg]
    encoder.setVelocityConversionFactor(gearRatio / 60.0);   //[deg/s]
    encoder.setPosition(0.0);
    // configure the hardware pid
    hwPID.copyTo(pid, 0);
    pid.setSmartMotionMaxVelocity(maxVel, PosSlot);
    pid.setSmartMotionMaxAccel(maxAccel, PosSlot);

    //todo:set tol on pos pid

    //safety and power limits
    ctrl.setSmartCurrentLimit(20, 20,  10); //stallLimit, int freeLimit, int limitRPM);
    ctrl.burnFlash();
    Timer.delay(.2);
    ntcreate();
  }

  // @Override
  public void periodic() {
   //measure 
   currentPos = encoder.getPosition();   // [deg]
   currentVel = encoder.getVelocity();
   //calcs
   desiredVel =  (velocityMode) ? desiredVel : MathUtil.clamp(positionPID.calculate(currentPos), -maxVel, maxVel);
   
   //output
  // desiredVel = 0.0;
 //  pid.setReference(desiredVel, ControlType.kVelocity);
   ntupdates();
  }

  public boolean atSetpoint() {
    return positionPID.atSetpoint();
  }


  public void setSetpoint(double degrees){
      positionPID.setSetpoint(degrees);
      velocityMode = false;
      desiredVel = 0.0;
  }

  // sets encoder to a default positon, doesn't move anything
  public void setPosition(double degrees) {
      encoder.setPosition(degrees);
  } 

  public double getSetpoint() {
    return positionPID.getSetpoint();
  }

  public double getPosition() {
    return currentPos;
  }

  public void setVelocityCmd(double vel){
    this.desiredVel = vel;
    velocityMode = true;
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

  public void hold(){
    //zero the velocity
   // pid.setReference(0.0, ControlType.kVelocity);
    //set positionPID to where we are
    currentPos = encoder.getPosition();
    setSetpoint(currentPos);
    positionPID.reset();
    positionPID.calculate(currentPos);  //give pid one measurement where we are
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

  }
  
  public void ntupdates(){
    nt_currentPos.setDouble(encoder.getPosition());
    nt_currentVel.setDouble(encoder.getVelocity());
    nt_desiredPos.setDouble(positionPID.getSetpoint());
    nt_desiredVel.setDouble(desiredVel);
  }

  
}
