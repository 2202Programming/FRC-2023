// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;

public class Elbow extends SubsystemBase {
  /** Creates a new Elbow. */
  CANSparkMax elbowMotor;
  private double currentPos;
  private double desiredPos;
  public Elbow() {
    
  }

  @Override
  public void periodic() {
    if (isAtPosition()){
      stopMotor();
    } else {
      goToPosition(); //TODO create goToPosition method
    }
  }

  public boolean isAtPosition(){
    if (desiredPos == currentPos){
      return true;
    } else {
      return false;
    }
  }

  public void setDesiredPos(double degrees){
    this.desiredPos = degrees;
  }

  public void stopMotor(){
    elbowMotor.stopMotor();
  }
}
