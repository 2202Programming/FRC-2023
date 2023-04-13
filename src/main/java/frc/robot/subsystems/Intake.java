// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.PCM1;
import frc.robot.Constants.Intake_Constants;

/*
 * On the intake: upper and lower motor and pneumatics (double solenoid)
 * On the car wash: left and right motor
 * Everything is a NEO on a Spark Max
 */

public class Intake extends SubsystemBase {

  private double currentCarwashSpeed;

  //Localized Constants - what valve value does what action
  static final Value DEPLOY  = Value.kReverse;
  static final Value RETRACT = Value.kForward;

  //Instantiations
  final CANSparkMax l_intake_mtr = new CANSparkMax(CAN.INTAKE_LEFT_MTR, CANSparkMax.MotorType.kBrushless);
  final CANSparkMax r_intake_mtr = new CANSparkMax(CAN.INTAKE_RIGHT_MTR, CANSparkMax.MotorType.kBrushless);
  final DoubleSolenoid rt_intake_solenoid = new DoubleSolenoid(CAN.PCM1,
              PneumaticsModuleType.REVPH,
              PCM1.RT_INTAKE_UP_SOLENOID_PCM,
              PCM1.RT_INTAKE_DOWN_SOLENOID_PCM);

final DoubleSolenoid lt_intake_solenoid = new DoubleSolenoid(CAN.PCM1,
              PneumaticsModuleType.REVPH,
              PCM1.LT_INTAKE_UP_SOLENOID_PCM,
              PCM1.LT_INTAKE_DOWN_SOLENOID_PCM);

  final DigitalInput lightgate = new DigitalInput(DigitalIO.IntakeLightGate);


  final CANSparkMax l_carwash_mtr = new CANSparkMax(CAN.CARWASH_LEFT_MTR, CANSparkMax.MotorType.kBrushless);
  final CANSparkMax r_carwash_mtr = new CANSparkMax(CAN.CARWASH_RIGHT_MTR, CANSparkMax.MotorType.kBrushless);
  
  /** Creates a new Intake. */
  public Intake() {
    motor_config(l_intake_mtr, false);
    motor_config(r_intake_mtr, false);
    motor_config(l_carwash_mtr, true);
    motor_config(r_carwash_mtr, false);

    ntconfig();
  }

  void motor_config(CANSparkMax mtr, boolean inverted) {
     mtr.clearFaults();
     mtr.restoreFactoryDefaults();
     mtr.setInverted(inverted);
  }

  @Override
  public void periodic() {
    ntupdates();
  }

  //Turn Intake Motor On by sending a double value
  public void setIntakeSpeed(double intakeMotorStrength) {
    l_intake_mtr.set(intakeMotorStrength);
    r_intake_mtr.set(intakeMotorStrength);
  }

  public double getIntakeSpeed() {
    return l_intake_mtr.get();
  }

  public void intakeOff() {
    setIntakeSpeed(0.0);
  }

  /*
    Mr.L - removing these to force using setIntakeSpeed(spd)  
    Commands can change the speed and deal with directions.

  public void intakeOn(){    //on() with no-args is default
    setIntakeSpeed(Intake_Constants.IntakeMotorStrength);
  }
  
  public void intakeOnReverse() {
    setIntakeSpeed(-Intake_Constants.IntakeMotorStrength);
  }

 

  public void intakeReverse() {
    setIntakeSpeed(-currentIntakeSpeed);
    currentIntakeSpeed = -currentIntakeSpeed;
  }
*/

  //Deploy arm mechanism using a Double Solenoids
  public void deploy() {
    rt_intake_solenoid.set(DEPLOY);
    lt_intake_solenoid.set(DEPLOY);
  }

  //Retract arm mechanism using a Double Solenoids
  public void retract() {
      rt_intake_solenoid.set(RETRACT);
      lt_intake_solenoid.set(RETRACT);
  }
  
  //Returns the state of the Intake Arm
  public boolean isDeployed() {
    return (rt_intake_solenoid.get() == DEPLOY); 
  }

  public void setCarwashSpeed(double carwashMotorStrength) {
    l_carwash_mtr.set(carwashMotorStrength);
    r_carwash_mtr.set(carwashMotorStrength);
    currentCarwashSpeed = carwashMotorStrength;
  }

  public void carwashOn() {
    setCarwashSpeed(Intake_Constants.CarwashMotorStrength);
  }

  public void carwashOnReverse() {
    setCarwashSpeed(-Intake_Constants.CarwashMotorStrength);
  }

  public void carwashOff() {
    setCarwashSpeed(0.0);
  }

  public void carwashReverse() {
    setCarwashSpeed(-currentCarwashSpeed);
    currentCarwashSpeed = -currentCarwashSpeed;
  }

  public boolean lightgateIsBlocked() {
    return !lightgate.get();
  }

  /**
   * NTs
   */

  NetworkTable nt = NetworkTableInstance.getDefault().getTable("Intake");
  NetworkTableEntry nt_intakeSpeed = nt.getEntry("Intake Speed");
  NetworkTableEntry nt_carwashSpeed = nt.getEntry("Carwash Speed");

  public void ntconfig() {
    // nt_intakeSpeed.setDouble(0.0);
    // nt_carwashSpeed.setDouble(0.0);
  }

  public void ntupdates() {
    // if (nt_intakeSpeed.getDouble(0.0) != IntakeMotorStrength) setIntakeSpeed(nt_intakeSpeed.getDouble(0.0));
    // if (nt_carwashSpeed.getDouble(0.0) != CarwashMotorStrength) setIntakeSpeed(nt_carwashSpeed.getDouble(0.0));
  }

}

