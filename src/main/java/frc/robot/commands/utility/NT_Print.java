// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utility;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class NT_Print extends InstantCommand {

  // Create new NT_Print command
  // To allow for Network table outputs during command groups (similar to the print command)
  public NT_Print(NetworkTableEntry entry, Object value) {

    entry.setValue(value);

  }
}
