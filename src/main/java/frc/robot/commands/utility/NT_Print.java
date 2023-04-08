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

//Create new network table
public class NT_Print extends InstantCommand {
  NetworkTable table;
  NetworkTableEntry nt_value;
  // Create new NetworkTable entry
  public NT_Print(NetworkTable table, String key, Object value) {
    this.table = table;
    nt_value = table.getEntry(key);
    new NT_Print(nt_value, value);
  }

  //Use existing network table
  //Can't use switch case since it is too expensive/java restrictions
  public NT_Print(NetworkTableEntry entry,Object value){
    if (value instanceof Integer) {
      int temp = (int) value;
     entry.setInteger(temp);
    } else if (value instanceof String) {
      String temp = (String) value;
     entry.setString(temp);
    } else if (value instanceof Boolean) {
      boolean temp = (boolean) value;
     entry.setBoolean(temp);
    } else if(value instanceof Double) {
      double temp = (double) value;
     entry.setDouble(temp);
    }
  }
}
