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

public class Carwash extends SubsystemBase {

  final double CarwashMotorStrength = 0; //used for default ON


  //Instantiations
  final CANSparkMax l_carwash_mtr = new CANSparkMax(CAN.INTAKE_LEFT_MTR, CANSparkMax.MotorType.kBrushless);
  final CANSparkMax r_carwash_mtr = new CANSparkMax(CAN.INTAKE_RIGHT_MTR, CANSparkMax.MotorType.kBrushless);

  //TODO: dpl  1/27/23 possible lightgate for triggering - check with mechanical team
  
  /** Creates a new Carwash. */
  public Carwash() {
    motor_config(l_carwash_mtr, false);
    motor_config(r_carwash_mtr, true);    //TODO: Check if inversion is needed
    
  }

  void motor_config(CANSparkMax mtr, boolean inverted) {
     mtr.clearFaults();
     mtr.restoreFactoryDefaults();
     mtr.setInverted(inverted);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Turn Intake Motor On by sending a double value
  public void on(double intakeMotorStrength) {
    l_intake_mtr.set(intakeMotorStrength);
    r_intake_mtr.set(intakeMotorStrength);
  }

  public void on(){    //on() with no-args is default
    on(CarwashMotorStrength);
  }   

  public void off() {
    on(0);
  }

}

