// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.PCM1;
import frc.robot.Constants.PowerOnPos;
import frc.robot.RobotContainer;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;
import frc.robot.util.VelocityControlled;

//Eventually will need 2 solenoids 
public class Claw_Substyem extends SubsystemBase {

  /*
   * claw level tracking options
   */
  public enum ClawTrackMode {
    frontSide(90.0),  //dpl 4/14/23 level out for shelf pickup, was 94
    backSide(-109.0),
    faceDown(-10.0),
    faceUp(170.0),
    free(0.0);   //any angle, use setWristAngle()
    double angle;

    ClawTrackMode(double angle) {
      this.angle = angle;
    }

    public double angle() {
      return this.angle;
    }
  };

  // solenoid constants
  static final Value OPEN = Value.kForward;
  static final Value CLOSE = Value.kReverse;

  // Wrist constants & constraints
  final int WRIST_STALL_CURRENT = 20;
  final int WRIST_FREE_CURRENT = 10;
  final double WRIST_MIN_DEG = -130.0;  //WITH THE CLAW CLOSED
  final double WRIST_MAX_DEG = 110.0;
  double wrist_maxAccel = 10.0; // only used if in smartmode, a future
  double wrist_maxVel = 160.0;
  double wrist_posTol = 4.0;
  double wrist_velTol = 2.0;
  final double wrist_conversionFactor = 360.0 / 150.0; // GR=150.0
 
  // compensate for wrist rotation
  double maxArbFF = 0.02;


  // Hardware
  final DoubleSolenoid solenoid = new DoubleSolenoid(CAN.PCM1, PneumaticsModuleType.REVPH, PCM1.CLAW_FWD, PCM1.CLAW_REV);
  final DigitalInput lightgate = new DigitalInput(DigitalIO.ClawLightgate);
  final NeoServo wrist_servo;

  final TalonSRX intake_wheels;
  double wheel_speed = 1.0;

  // PIDS for NeoServos - first pass on wrist tuning
  // Testing showed 200 [deg/sec] was good to go! Still lots of overshoot on vel
  // step response. 25% 3/4/23
  PIDController wrist_positionPID = new PIDController(4.0, 0.005, 0.05);
  PIDFController wrist_hwVelPID = new PIDFController(0.00045, 0.0, 0.01, 0.0017); // original kI 0.0000052
 

  // reads the elbow angle for tracking
  DoubleSupplier elbowAngle;

  // state vars
  private boolean is_open;
  private boolean gate_blocked;
  ClawTrackMode trackElbowMode = ClawTrackMode.backSide; 

  public Claw_Substyem() {
    //wrist_hwVelPID.setIzone(wrist_hwVelPID.getP()); // on same order of magnitude as hw kP
    wrist_servo = new NeoServo(CAN.WRIST_Motor, wrist_positionPID, wrist_hwVelPID, true);
    
    intake_wheels = new TalonSRX(CAN.CLAW_WHEEL_MOTOR);
    intake_wheels.setInverted(false);   //flip this is in/out is reversed

    wrist_servo.setName(getName() + "/wrist")
        .setConversionFactor(wrist_conversionFactor)
        .setSmartCurrentLimit(WRIST_STALL_CURRENT, WRIST_FREE_CURRENT)
        .setVelocityHW_PID(wrist_maxVel, wrist_maxAccel)
        .setTolerance(wrist_posTol, wrist_velTol)
        .setMaxVelocity(wrist_maxVel)
        .setBrakeMode(IdleMode.kCoast)
        .burnFlash();

    wrist_servo.setClamp(WRIST_MIN_DEG , WRIST_MAX_DEG);

    // make sure we are at a good staring point (folded inside)
    wrist_servo.setSetpoint(PowerOnPos.wrist);
    wrist_servo.setPosition(PowerOnPos.wrist);
  
    // Use elbow if we have one, otherwise zero
    elbowAngle = (RobotContainer.RC().elbow != null) ? RobotContainer.RC().elbow::getPosition : this::zero;

    this.setTrackElbowMode(ClawTrackMode.backSide);
    open(); // claw should be open at power up
    wheelsOff(); // wheels should not be spinning at power up
  }

  // some of my most complex code
  double zero() {
    return 0.0;
  }

  // accessors for servos for testing
  public VelocityControlled getWrist() {
    return wrist_servo;
  }

 
  public void setElbowDoubleSupplier(DoubleSupplier func) {
    elbowAngle = func;
  }

  // getting the angles current position
  public double getWristAngle() {
    return wrist_servo.getPosition();
  }
 
 
  public boolean wristAtSetpoint() {
    return wrist_servo.atSetpoint();
  }

  public void setWristAngle(double degrees) {
    wrist_servo.setSetpoint(degrees);
    trackElbowMode = ClawTrackMode.free;
  }
 
 


  @Override
  public void periodic() {
    //measure elbow if we are tracking, otherwise use zero
    double elbow_meas = elbowAngle.getAsDouble();

    // calculate holding power needed from angle and maxArbFF term
    double arbff = maxArbFF * Math.sin(Math.toRadians(elbow_meas + wrist_servo.getSetpoint()));
    wrist_servo.setArbFeedforward(arbff);

    // in tracking modes, do the elbow math to figure out where to go
    if (trackElbowMode != ClawTrackMode.free) {
      wrist_servo.setSetpoint(trackElbowMode.angle() - elbow_meas);
    }
    //NOTE:in Free mode, the wrist setpoint is called directly
    
    //run the servo calcs
    wrist_servo.periodic();
  
    // read gate, yes it needs to be negated.
    gate_blocked = !lightgate.get();
  }


  @Override
  public void simulationPeriodic(){
    wrist_servo.simulationPeriodic();
  }


  /**
   * Sets the track mode to front or back based on the angle of the claw relative to the field.
   * These two moves are what should be used when transitioning, or moving, between unknown states.
   * 
   * @return The claw track mode in its current state, regardless of whether it has been changed.
   */
  public ClawTrackMode  setTransitionClawTrackMode() {
    // elbow angle + wrist angle relative to elbow = field-relative angle
    // field-relative angle is what track mode angles are based on so this is what we want to use
    double fieldRelAngle = getWristAngle() + elbowAngle.getAsDouble();
    //looks at claw anglesto pick which side we should be on coming from free mode.
    ClawTrackMode mode =  fieldRelAngle >= 0.0 ? ClawTrackMode.frontSide : ClawTrackMode.backSide;
    setTrackElbowMode(mode);
    return getTrackElbowMode();
  } 

  public void setTrackElbowMode(ClawTrackMode mode) {
    //tracking mode, zero the manual offset
    trackElbowMode = mode;
  }

  public ClawTrackMode getTrackElbowMode(){
    return trackElbowMode;
  }
  public boolean isTrackingElbow() {
    // we are tracking elbow if not in free mode
    return (trackElbowMode != ClawTrackMode.free);
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

  public boolean isGateBlocked() {
    return gate_blocked;
  }

  public Command getWatcher() {
    wrist_servo.getWatcher();
    return new ClawWatcher();
  }

  //Claw intake Wheel control
  public void setWheelSpeed(double speed) {
    wheel_speed = Math.abs(speed);
  }
  public void wheelsIn(){
    intake_wheels.set(TalonSRXControlMode.PercentOutput, wheel_speed );
  }
  public void wheelsOut() {
    intake_wheels.set(TalonSRXControlMode.PercentOutput, -wheel_speed );
  }
  public void wheelsOff() {
    intake_wheels.set(TalonSRXControlMode.PercentOutput, 0.0 );
  }

  /**
   ******** NETWORK TABLES ***********
   */
  class ClawWatcher extends WatcherCmd {
    NetworkTableEntry nt_isOpen;
    NetworkTableEntry nt_gamePieceHeld;
    NetworkTableEntry nt_maxArbFF;
    NetworkTableEntry nt_trackMode;
    NetworkTableEntry nt_lightgate;
    @Override
    public String getTableName() {
      return Claw_Substyem.this.getName();
    }

    public void ntcreate() {
      NetworkTable table = getTable();
      nt_isOpen = table.getEntry("Open");
      nt_maxArbFF = table.getEntry("/wrist/maxArbFF");
      nt_trackMode = table.getEntry("trackMode");
      nt_lightgate = table.getEntry("lightgate");

      // default value for mutables
      nt_maxArbFF.setDouble(maxArbFF);
    }

    public void ntupdate() {
      nt_isOpen.setBoolean(is_open);
      nt_trackMode.setString(trackElbowMode.toString());
      nt_lightgate.setBoolean(isGateBlocked());

      // get mutable values
      maxArbFF = nt_maxArbFF.getDouble(maxArbFF);
      
    }
  }

}