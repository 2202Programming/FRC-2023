// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
/*
 * Scales the sticks for precise tracking. 
 * true  - (20% xy, 10% rot)
 * false - restores values
 */
public class PrecisionMode extends InstantCommand {
  double scale_xy = 0.2;
  double scale_rot = 0.1;
  final boolean enable;
  final HID_Xbox_Subsystem  dc = RobotContainer.RC().dc;

  public PrecisionMode(boolean enable) {
    this.enable = enable;
  }

  public PrecisionMode(double scale_xy, double scale_rot) {
    this.enable = true;
    this.scale_xy = scale_xy;
    this.scale_rot = scale_rot;
  }

  // Apply the scale factor to the stick, just like cutting the max speed 25%
  @Override
  public void initialize() {
    if (enable) {
      dc.setStickScale(scale_xy, scale_rot);
    }
    else  dc.resetStickScale();
  }
}
