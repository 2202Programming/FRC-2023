// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Constants.CAN;
import frc.robot.Constants.Claw.GamePieceHeld;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants.Claw;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/*
 * Notes from Mr.L  1/27/2023
 * 
 * Some of your variables would make for a good API
 * 
 *      is_open the state can be read off the Pnumatics 
 *      pieceheld -->  enum  HasPiece()  returns NONE, CUBE, CONE
 *     
 *      Is angle is wrist angle??, that may be a PWM Server (see vacuum bot)
 *      it won't need a PID, but will need a pwm port. 
 *      
 */
// 2 servos, 1 left/right each have a 0 position, have one mirror the other, (switch signs & mapping)
 // call 180 angle 0 and work from one way to the other - 0 wrist won't be 0 servo 
 //Eventually will need 2 solenoids 
public class Claw_Substyem extends SubsystemBase {
  private double current_angle;
  private double desired_angle;
  private boolean is_open;
  private GamePieceHeld piece_held; 
  private double servo_position;
  private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Claw.SOLENOID_FORWARD_CHANNEL, Claw.SOLENOID_REVERSE_CHANNEL);
  Servo rightServo = new Servo(0);
  Servo leftServo = new Servo(1);
  static final Value OPEN = Value.kForward;
  static final Value CLOSE = Value.kReverse;
  

  
  /** Creates a new Claw. */
  public Claw_Substyem() {
    //TODO Find out motor then update
    piece_held = GamePieceHeld.Empty;
  }
public double getAngle(){
  solenoid.set(null);
  return desired_angle;
}
public void setAngle(double Desired_angle){
  this.desired_angle = Desired_angle;
}
public double getServoPos(){
  return servo_position;
}
public void setServoPos(double Servo_position){
  this.servo_position = Servo_position;
}

  public double getCurrentAngle(){
    return current_angle;
  }
  public double getDesiredAngle(){
    return desired_angle;
  }
  public void setDesiredAngle(double Desired_angle){
    this.desired_angle = Desired_angle;
    //drive wrist server here

    // if it is a servo, we don't have a measure of the angle
    // unless we do something tricky in the servo.
    // if it is a motor, we need the pid and a sensor for angle, likely a POT.
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void open(){
    solenoid.set(OPEN);
  }
  public void close(){
    solenoid.set(CLOSE);
  }
  public Value isOpen(){
    return solenoid.get();
  }


  /**
   ******** NETWORK TABLES ***********
   */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Claw");

  NetworkTableEntry nt_currentAngle;
  NetworkTableEntry nt_desiredAngle;
  NetworkTableEntry nt_isOpen;
  NetworkTableEntry nt_gamePieceHeld;                         



  public void ntcreate(){
    nt_currentAngle = table.getEntry("Current Angle");
    nt_desiredAngle = table.getEntry("Desired Anngle");
    nt_isOpen = table.getEntry("Is Claw Open");
    nt_gamePieceHeld = table.getEntry("Game Piece Held");
  }
  public void ntupdates(){
    //info
    nt_currentAngle.setDouble(current_angle);
    nt_desiredAngle.setDouble(desired_angle);
    nt_isOpen.setBoolean(is_open);
    nt_gamePieceHeld.setString(piece_held.toString());

  }

}