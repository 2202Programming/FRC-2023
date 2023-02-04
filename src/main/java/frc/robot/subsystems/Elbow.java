// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.CAN;
import frc.robot.util.PIDFController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elbow extends SubsystemBase {
  static int PosSlot = 0;
  static double maxVel = 10.0;   //[deg/s]
  static double maxAccel = 5.0;  //[deg/s^2]

  // at position limits
  static double posLimit = 3.0; //[deg]
  static double velLimit = 5.0; //[deg/s]

  /** Creates a new Elbow. */
  final CANSparkMax ctrl;
  final SparkMaxPIDController pid;
  final RelativeEncoder encoder;

  // placeholder for HW PID values
  // Use position loop in the HW
  PIDFController hwPID = new PIDFController(0.0, 0.0, 0.0, 0.0); // TODO

  // measurements
  double currentPos;
  double desiredPos;
  double currentVel;

  public Elbow() {
    ctrl = new CANSparkMax(CAN.ELBOW, MotorType.kBrushless);
    ctrl.restoreFactoryDefaults();
    ctrl.setIdleMode(CANSparkMax.IdleMode.kBrake);
    pid = ctrl.getPIDController();
    encoder = ctrl.getEncoder();
    // TODO: set scaling on endoder to use [deg] and [deg/s]
    encoder.setPositionConversionFactor(0.0);   //TODO fix me
    encoder.setVelocityConversionFactor(0.0);   //TODO fix me
    // configure the hardware pid
    hwPID.copyTo(pid, 0);
    pid.setSmartMotionMaxVelocity(maxVel, PosSlot);
    pid.setSmartMotionMaxAccel(maxAccel, PosSlot);

    //safety and power limits
    ctrl.setSmartCurrentLimit(10, 10,  10); //stallLimit, int freeLimit, int limitRPM);
    
  }

  @Override
  public void periodic() {
   //measure 
   currentPos = encoder.getPosition();   // [deg]
   currentVel = encoder.getVelocity();
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

  
}
