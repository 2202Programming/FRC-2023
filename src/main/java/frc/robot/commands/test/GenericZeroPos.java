// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.VelocityControlled;

public class GenericZeroPos extends CommandBase {

  final VelocityControlled device;
  public GenericZeroPos(VelocityControlled device){
    this.device = device;
    this.runsWhenDisabled();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.device.setPosition(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
