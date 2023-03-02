// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import edu.wpi.first.wpilibj2.command.Place;

public class beginPlacement extends CommandBase {
  public final HID_Xbox_Subsystem dc; // short for driver controls
  /** Creates a new beginPlacement. */
  public beginPlacement(HID_Xbox_Subsystem dc) {
    this.dc = dc;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: Pull from Nathanael's color sensor subsystem code
    //TODO: Move game piece and orientation code into here
    //Get game piece color
    boolean piece = true; //isCube
    //Get game piece orientation
    boolean orientation = true; //isOk
    //Location
    boolean lateral; //isLeft 
    boolean vertical; //isMiddle
    //TODO: REPLACE WITH ENUMS
    //Get operator buttons
    if (dc.Operator().leftBumper().getAsBoolean()) {
      lateral = false;
      vertical = false;
    };
    if (dc.Operator().rightBumper().getAsBoolean()) {
      lateral = true;
      vertical = false;
    };
    if (dc.Operator().leftTrigger().getAsBoolean()) {
      vertical = true;
      lateral = false;
    };
    if (dc.Operator().rightTrigger().getAsBoolean()) {
      vertical = true;
      lateral = true;
    };
    //Place
    new Place(piece,lateral,vertical,orientation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
