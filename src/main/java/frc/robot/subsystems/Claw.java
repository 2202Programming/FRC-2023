// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Constants.CAN;
import frc.robot.Constants.Claw.GamePieceHeld;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Claw extends SubsystemBase {
  private double current_angle;
  private double desired_angle;
  private boolean is_open;
  private PIDController claw_controller = new PIDController(0.0, 0.0, 0.0);
  private GamePieceHeld piece_held; 
  /** Creates a new Claw. */
  public Claw() {
    //TODO Find out motor then update
    piece_held = GamePieceHeld.Empty;
  }

  public double getCurrentAngle(){
    return current_angle;
  }
  public double getDesiredAngle(){
    return desired_angle;
  }
  public void setDesiredAngle(double Desired_angle){
    this.desired_angle = Desired_angle;
  }
  public boolean isOpen(){
    return is_open;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  /**
   ******** NETWORK TABLES ***********
   */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Claw");

  NetworkTableEntry nt_currentAngle;
  NetworkTableEntry nt_desiredAngle;
  NetworkTableEntry nt_isOpen;
  NetworkTableEntry nt_kP;
  NetworkTableEntry nt_kD;
  NetworkTableEntry nt_kI;



  public void ntcreate(){
    nt_currentAngle = table.getEntry("Current Angle");
    nt_desiredAngle = table.getEntry("Desired Anngle");
    nt_isOpen = table.getEntry("Is Claw Open");
    nt_kP = table.getEntry("kP");
    nt_kI = table.getEntry("kI");
    nt_kD = table.getEntry("kD");

    nt_kP.setDouble(0.0);
    nt_kI.setDouble(0.0);
    nt_kD.setDouble(0.0);
  }
  public void ntupdates(){
    //info
    nt_currentAngle.setDouble(current_angle);
    nt_desiredAngle.setDouble(desired_angle);
    nt_isOpen.setBoolean(is_open);
    claw_controller.setP(nt_kP.getDouble(0.0));
    claw_controller.setI(nt_kI.getDouble(0.0));
    claw_controller.setD(nt_kD.getDouble(0.0));

  }

}