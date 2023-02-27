// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.utility.Lockout;
import frc.robot.util.VelocityControlled;

public class GenericVelocityTest extends CommandBase implements Lockout{

  Timer stopwatch = new Timer();

  final VelocityControlled device;
  final double vel, runTime, pauseTime;
  double old_max_speed;

  // states
  double cmdVel;
  double time;
  boolean running;

  /**
   * Creates a new deviceVelocityTest.
   * 
   * Runs the device at fixed speed for given period,
   * pauses, then runs in reverse for given period.
   * 
   * Take CAUTION to start in the middle of the devices range, there are no position
   * checks if the hardware doesn't have them.
   * 
   * Runs until button is released.
   */

  public GenericVelocityTest(VelocityControlled device,
    double vel, double runTime, double pauseTime) {
    this.device = device;
    this.vel = vel;
    this.runTime = runTime;
    this.pauseTime = pauseTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    old_max_speed = device.getMaxVel();
    device.setMaxVel(vel);
    running = true;
    cmdVel = vel;
    stopwatch.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (running) {
      // moving the device
      if (stopwatch.hasElapsed(runTime)) {
        running = false;
        cmdVel = cmdVel * -1.0; // flip the direction
        stopwatch.reset();
        device.setVelocityCmd(0.0);
      } else
        device.setVelocityCmd(cmdVel);
      return;
    }

    // pausing
    device.setVelocityCmd(0.0);
    if (stopwatch.hasElapsed(pauseTime)) {
      running = true;
      stopwatch.reset();
      device.setVelocityCmd(cmdVel);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    device.hold();
    device.setMaxVel(old_max_speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // rely on button
  }
}
