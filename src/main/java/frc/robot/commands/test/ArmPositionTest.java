// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.ArmSS;

public class ArmPositionTest extends CommandBase {
  ArmSS arm = RobotContainer.RC().armSS;
  HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
  double MaxLength = 39.0; // [cm]

  boolean setpointAtZero;
  double count = 0;

  /*
   * Simple test to position the arm using the left trigger
   * 
   *  may be used as a sub-system default, never ends
   */
  public ArmPositionTest() {
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setPositions(0.0); // assumes arm is at bottom when starting
    arm.setVelocityLimit(15.0); //[cm/s]
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pos = dc.Driver().getLeftTriggerAxis() * MaxLength; //[0.0 .. 1.0][cm]
    arm.setSetpoints(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
