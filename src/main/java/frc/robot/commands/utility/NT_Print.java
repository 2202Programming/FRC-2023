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
    this.table = table;
    NetworkTableEntry nt_value;
    nt_value = table.getEntry(key);
    if (value instanceof Integer) {
      int temp = (int) value;
      nt_value.setInteger(temp);
    } else if (value instanceof String) {
      String temp = (String) value;
      nt_value.setString(temp);
    } else if (value instanceof Boolean) {
      boolean temp = (boolean) value;
      nt_value.setBoolean(temp);
    } else if(value instanceof Double) {
      double temp = (double) value;
      nt_value.setDouble(temp);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}
