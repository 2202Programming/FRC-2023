// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utility;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NT_Print extends InstantCommand {
  NetworkTable table;

  // Create new NetworkTable entry
  public NT_Print(NetworkTable table, String key, Object value) {
    
    if (value instanceof Integer) {
    } else if (value instanceof String) {

    } else if (value instanceof Boolean) {

    } else {// Double

    }
  }

  // Use existing NetworkTable entry
  public NT_Print(NetworkTableEntry entry, Object value) {
    if (value instanceof Integer) {

    } else if (value instanceof String) {

    } else if (value instanceof Boolean) {

    } else {// Double

    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}
