// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PCM1;

public class Intake extends SubsystemBase {

  final double IntakeMotorStrength = 0.6; //used for default ON

  //Localized Constants - what valve value does what action
  static final Value DEPLOY  = Value.kReverse;
  static final Value RETRACT = Value.kForward;

  //Instantiations
  final CANSparkMax intake_mtr = new CANSparkMax(CAN.INTAKE_MTR, CANSparkMax.MotorType.kBrushless);
  final DoubleSolenoid intake_solenoid = new DoubleSolenoid(CAN.PCM1,
              PneumaticsModuleType.CTREPCM,
              PCM1.INTAKE_UP_SOLENOID_PCM,
              PCM1.INTAKE_DOWN_SOLENOID_PCM);

  //TODO: dpl  1/27/23 possible lightgate for triggering - check with mechanical team
  
  /** Creates a new Intake. */
  public Intake() {
    intake_mtr.clearFaults();
    intake_mtr.restoreFactoryDefaults();
    intake_mtr.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Turn Intake Motor On by sending a double value
  public void on(double intakeMotorStrength) {
    intake_mtr.set(intakeMotorStrength);
  }

  public void on(){    //on() with no-args is default
    on(IntakeMotorStrength);
  }   

  public void off() {
    intake_mtr.set(0);
  }

  //Deploy arm mechanism using a Double Solenoids
  public void deploy() {
    intake_solenoid.set(DEPLOY);
  }

  //Retract arm mechanism using a Double Solenoids
  public void retract() {
      intake_solenoid.set(RETRACT);
  }
  
  //Returns the state of the Intake Arm
  public boolean isDeployed() {
    return ( intake_solenoid.get() == DEPLOY); 
  }

}
