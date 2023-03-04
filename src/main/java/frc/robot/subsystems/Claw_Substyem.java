// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PCM1;
import frc.robot.commands.utility.WatcherCmd;
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
  double wrist_maxAccel = 10.0; // only used if in smartmode, a future
  double wrist_maxVel = 120.0;
  double wrist_posTol = 3.0;
  double wrist_velTol = 2.0;
  final double wrist_conversionFactor = 360.0 / 150.0; // GR=150.0
  static final double WristMinDegrees = -90.0;
  static final double WristMaxDegrees = 90.0;

  // compensate for wrist rotation
  double maxArbFF = 0.02;

  // Rotation Constants
  final int ROTATE_STALL_CURRENT = 20;
  final int ROTATE_FREE_CURRENT = 10;
  double rotate_maxAccel = 10.0; // only used if in smartmode, a future
  double rotate_maxVel = 20.0;
  double rotate_posTol = 3.0;
  double rotate_velTol = 2.0;
  final double rotate_conversionFactor = 360.0 / 100.0; // GR=100.0

  // Hardware
  final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PCM1.CLAW_FWD, PCM1.CLAW_REV);
  final NeoServo wrist_servo;
  final NeoServo rotate_servo;

  // PIDS for NeoServos - first pass on wrist tuning
  PIDController wrist_positionPID = new PIDController(8.0, 0.05, 0.0);
  PIDFController wrist_hwVelPID = new PIDFController(0.00055, 0.000004, 0., 0.00185);

  // TODO (It's what arm values are rn, will need to change)
  PIDController rotate_positionPID = new PIDController(5.0, 0.150, 0.250);
  PIDFController rotate_hwVelPID = new PIDFController(0.002141, 0.00005, 0.15, 0.05017);

  DoubleSupplier elbowAngle;

  // state vars
  private boolean is_open;
  private GamePieceHeld piece_held;

  public Claw_Substyem() {
    wrist_servo = new NeoServo(CAN.WRIST_Motor, wrist_positionPID, wrist_hwVelPID, true);
    rotate_servo = new NeoServo(CAN.CLAW_ROTATE_MOTOR, rotate_positionPID, rotate_hwVelPID, false);

    wrist_servo.setName(getName() + "/wrist")
        .setConversionFactor(wrist_conversionFactor)
        .setSmartCurrentLimit(WRIST_STALL_CURRENT, WRIST_FREE_CURRENT)
        .setVelocityHW_PID(wrist_maxVel, wrist_maxAccel)
        .setTolerance(wrist_posTol, wrist_velTol)
        .setMaxVelocity(wrist_maxVel)
        .setBrakeMode(IdleMode.kCoast)
        .burnFlash();

    rotate_servo.setName(getName() + "/rotator")
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

    // default elbow angle supplier in case we are testing
    elbowAngle = this::zero;

    piece_held = GamePieceHeld.Cube;
  }

  // some of my most complex code
  double zero() {
    return 0.0;
  }

  // accessors for servos for testing
  public NeoServo getWrist() {
    return wrist_servo;
  }

  public NeoServo getRotator() {
    return rotate_servo;
  }

  public void setElbowDoubleSupplier(DoubleSupplier func) {
    elbowAngle = func;
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
    // calculate holding power needed from angle and maxArbFF term
    double arbff = maxArbFF * Math.sin(
        Math.toRadians(elbowAngle.getAsDouble() + wrist_servo.getSetpoint()));
    wrist_servo.setArbFeedforward(arbff);
    wrist_servo.periodic();
    rotate_servo.periodic();

    // clawStatus();
    // check any lightgates
  }

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

  public Command getWatcher() {
    wrist_servo.getWatcher();
    rotate_servo.getWatcher();
    return new ClawWatcher();
  }

  /**
   ******** NETWORK TABLES ***********
   */
  class ClawWatcher extends WatcherCmd {
    NetworkTableEntry nt_isOpen;
    NetworkTableEntry nt_gamePieceHeld;
    NetworkTableEntry nt_maxArbFF;

    @Override
    public String getTableName() {
      return Claw_Substyem.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_isOpen = table.getEntry("Open");
      nt_gamePieceHeld = table.getEntry("Game Piece");
      nt_maxArbFF = table.getEntry("/wrist/maxArbFF");

      // default value for mutables
      nt_maxArbFF.setDouble(maxArbFF);
    }

    public void ntupdate() {
      nt_isOpen.setBoolean(is_open);
      nt_gamePieceHeld.setString(piece_held.toString());
      // get mutable values
      maxArbFF = nt_maxArbFF.getDouble(maxArbFF);
    }
  }

}