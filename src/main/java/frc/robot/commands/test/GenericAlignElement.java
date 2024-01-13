// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.VelocityControlled;

public class GenericAlignElement extends Command {
  double increment;
  final VelocityControlled device;

  public GenericAlignElement(VelocityControlled device, double increment) {
    this.device = device;
    this.increment =increment;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double cmdPos = device.getSetpoint();
    cmdPos += increment;
    device.setSetpoint(cmdPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
  /*
   * Three buttons are used to run the alignment commands. 
   * Some general button for select A,B,X,Y...
   * Then use the D-Pad or some other button to inc/dec the device's setpoint.
   */
  public static void GenericAlignEelementFactory(VelocityControlled device, double increment, Trigger select, Trigger up, Trigger down) {
    increment = Math.abs(increment);
    Command upCmd = new GenericAlignElement(device, increment);
    Command downCmd = new GenericAlignElement(device, -increment);
    
    // factories do work, ours it to bind the commands to the right triggers
    select.and(up).onTrue(upCmd);
    select.and(down).onTrue(downCmd);

  }


}




