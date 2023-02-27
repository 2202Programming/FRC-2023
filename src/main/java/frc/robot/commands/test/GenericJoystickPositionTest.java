// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.utility.Lockout;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.VelocityControlled;

public class GenericJoystickPositionTest extends CommandBase implements Lockout {
  final VelocityControlled device;
  final HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
  final double max_vel;
  final double min_pos;
  final double max_pos;
  // joystick device [-1 to 1.0]
  final DoubleSupplier stick;

  // for stick mapping -1.0 to 1.0
  final double range;
  final double offset;
  double old_max_vel;

  /*
   * Simple test to position the device using the left trigger
   * 
   * Start with the device in the middle of its range,
   * 
   * may be used as a sub-system default, never ends
   */
  public GenericJoystickPositionTest(
      VelocityControlled device,
      DoubleSupplier stick,
      double min_pos, double max_pos, double max_vel) {
    this.device = device;
    this.max_pos = max_pos;
    this.min_pos = min_pos;
    this.max_vel = max_vel;
    this.stick = stick;
    //
    this.offset = (min_pos + max_pos) / 2.0;
    this.range = (max_pos - min_pos) / 2.0; // range is half due to stick range = 2.0

    addRequirements(device);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    old_max_vel = device.getMaxVel();
    device.setPosition(offset);
    device.setMaxVel(max_vel);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double native_pos = MathUtil.clamp(stick.getAsDouble(), -1.0, 1.0);
    double pos = offset + native_pos * range;
    device.setSetpoint(pos);
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
