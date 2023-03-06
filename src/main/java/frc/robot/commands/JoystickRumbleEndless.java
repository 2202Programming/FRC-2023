// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.Constants.DriverControls.Id;

public class JoystickRumbleEndless extends CommandBase {
  /** Rumbless while scheduled. */

  HID_Xbox_Subsystem dc;
  Id id;

  public JoystickRumbleEndless(Id id) {
    dc = RobotContainer.RC().dc;
    this.id = id;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.RC().dc.turnOnRumble(id, RumbleType.kBothRumble);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.RC().dc.turnOffRumble(id, RumbleType.kBothRumble);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
