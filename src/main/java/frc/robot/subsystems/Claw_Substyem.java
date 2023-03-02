// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PCM1;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;

//Eventually will need 2 solenoids 
public class Claw_Substyem extends SubsystemBase {

  public enum GamePieceHeld { 
    Cube, Cone, Empty
  }

  // solnoid constants
  static final Value OPEN = Value.kForward;
  static final Value CLOSE = Value.kReverse;

  // Wrist constants & constraints - all TODO
  final int WRIST_STALL_CURRENT = 20;
  final int WRIST_FREE_CURRENT = 10;
  double wrist_maxAccel = 10.0;  //only used if in smartmode, a future
  double wrist_maxVel = 20.0;
  double wrist_posTol = 3.0;
  double wrist_velTol = 2.0;
  final double wrist_conversionFactor = 360.0/150.0; //GR=150.0
  static final double WristMinDegrees = -90.0; 
  static final double WristMaxDegrees = 90.0; 

  // Rotation Constants
  final int ROTATE_STALL_CURRENT = 20;
  final int ROTATE_FREE_CURRENT = 10;
  double rotate_maxAccel = 10.0; //only used if in smartmode, a future
  double rotate_maxVel = 20.0;
  double rotate_posTol = 3.0;
  double rotate_velTol = 2.0;
  final double rotate_conversionFactor = 360.0/100.0; //GR=100.0

  // Hardware
  final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PCM1.CLAW_FWD, PCM1.CLAW_REV);
  final NeoServo wrist_servo;
  final NeoServo rotate_servo;

  // PIDS for NeoServos
  // TODO (It's what arm values are rn, will need to change)
  PIDController wrist_positionPID = new PIDController(5.0, 0.150, 0.250);
  PIDFController wrist_hwVelPID = new PIDFController(0.00, 0.0000, 0.0, 0.050);

  // TODO (It's what arm values are rn, will need to change)
  PIDController rotate_positionPID = new PIDController(5.0, 0.150, 0.250);
  PIDFController rotate_hwVelPID = new PIDFController(0.002141, 0.00005, 0.15, 0.05017);

  // state vars
  private boolean is_open;
  private GamePieceHeld piece_held;

  public Claw_Substyem() {
    wrist_servo = new NeoServo(CAN.WRIST_Motor, wrist_positionPID, wrist_hwVelPID, false);
    rotate_servo = new NeoServo(CAN.CLAW_ROTATE_MOTOR, rotate_positionPID, rotate_hwVelPID, false);

    wrist_servo
        .setConversionFactor(wrist_conversionFactor)
        .setSmartCurrentLimit(WRIST_STALL_CURRENT, WRIST_FREE_CURRENT)
        .setVelocityHW_PID(wrist_maxVel, wrist_maxAccel)        
        .setTolerance(wrist_posTol, wrist_velTol)
        .setMaxVelocity(wrist_maxVel)
        .setBrakeMode(IdleMode.kCoast)
        .burnFlash();
    
    rotate_servo
        .setConversionFactor(rotate_conversionFactor)
        .setSmartCurrentLimit(ROTATE_STALL_CURRENT, ROTATE_FREE_CURRENT)
        .setVelocityHW_PID(rotate_maxVel, rotate_maxAccel)
        .setTolerance(rotate_posTol, rotate_velTol)
        .setMaxVelocity(rotate_maxVel)
        .burnFlash();
    wrist_servo.setSetpoint(0.0);
    wrist_servo.setPosition(0.0);
    rotate_servo.setSetpoint(0.0);
    rotate_servo.setPosition(0.0);


    piece_held = GamePieceHeld.Cube;
  }

  // accessors for servos for testing
  public NeoServo getWrist() {
    return wrist_servo;
  }

  public NeoServo getRotator() {
    return rotate_servo;
  }

  // getting the angles current position
  public double getWristAngle() {
    return wrist_servo.getPosition();
  }
  
  // getting the angles current position
  public double getRotatetAngle() {
    return rotate_servo.getPosition();
  }
  
  public boolean wristAtSetpoint() {
    return wrist_servo.atSetpoint();
  }

  
  public void setWristAngle(double degrees) {
    wrist_servo.setSetpoint(degrees);
  }
  public boolean rotateAtSetpoint() {
    return rotate_servo.atSetpoint();
  }

 
  @Override
  public void periodic() {
    wrist_servo.periodic();
    rotate_servo.periodic();

    //clawStatus();
    // check any lightgates

  }

  // setting solenoid NOTE:2/7 don't need OpenClaw... there will be a Claw Object
  // so it will read claw.open() or claw.close()
  public void open() {
    solenoid.set(OPEN);
    is_open = true;
  }

  public void close() {
    solenoid.set(CLOSE);
    is_open = false;
  }

  public boolean isOpen() {
    return is_open;
  }

  // getting status of solenoid (open/close)
  // and keep the is_open flag up to date
  // commands can use isOpen()
  private Value clawStatus() {
    var value = solenoid.get();
    is_open = (value == OPEN);
    return value;
  }

  /**
   ******** NETWORK TABLES ***********
   */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Claw");
  NetworkTableEntry nt_servoPos;
  NetworkTableEntry nt_angle;
  NetworkTableEntry nt_isOpen;
  NetworkTableEntry nt_gamePieceHeld;

  public void ntcreate() {
    nt_servoPos = table.getEntry("wrist_cmd");
    nt_angle = table.getEntry("wrist_angle");
    nt_isOpen = table.getEntry("Claw Open");
    nt_gamePieceHeld = table.getEntry("Game Piece Held");
  }

  public void ntupdates() {
    nt_servoPos.setDouble(wrist_servo.getPosition());
    nt_angle.setDouble(getWristAngle());
    nt_isOpen.setBoolean(is_open);
    nt_gamePieceHeld.setString(piece_held.toString());

  }

}