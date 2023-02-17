// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSS;

public class ArmVelocityTest extends CommandBase {

  Timer stopwatch = new Timer();

  ArmSS arm;
  double vel, runTime, pauseTime;

  // states
  double cmdVel;
  double time;
  boolean running;

  /**
   * Creates a new ArmVelocityTest.
   * 
   * Runs the are at a fixed speed for given period,
   * pauses, then runs in reverse for given period.
   * 
   *
   * Runs until button is released.
   */

  public ArmVelocityTest(double vel, double runTime, double pauseTime) {
    this.arm = RobotContainer.RC().armSS;
    this.vel = vel;
    this.runTime = runTime;
    this.pauseTime = pauseTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    running = true;
    cmdVel = vel;
    stopwatch.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (running) {
      // moving the arm
      if (stopwatch.hasElapsed(runTime)) {
        running = false;
        cmdVel = cmdVel * -1.0; // flip the direction
        stopwatch.reset();
        arm.setVelocityCmd(0.0);
      } else
        arm.setVelocityCmd(cmdVel);
      return;
    }

    // pausing
    arm.setVelocityCmd(0.0);
    if (stopwatch.hasElapsed(pauseTime)) {
      running = true;
      stopwatch.reset();
      arm.setVelocityCmd(cmdVel);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // rely on button
  }
}
