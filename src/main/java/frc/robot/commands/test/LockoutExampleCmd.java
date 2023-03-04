// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.utility.Lockout;

/**
 * 
 * Usage example for locking out command from multiple button hits.
 * This is good for testing or in some cases competiton where a double press
 * of a button would be bad.
 * 
 * Bind a command to a button, then decorate the command withlockout()
 * 
 * You can copy the following line into RobotContainer to see the example work.
 *   dc.Driver().leftBumper()
 *       .whileTrue(new LockoutExampleCmd().WithLockout(15.0));
 * 
 */

/* To use the withLockout() decorator, you must use 'implements lockout' */
public class LockoutExampleCmd extends CommandBase implements Lockout {
  int count;

  /** Creates a new LockoutExampleCmd. */
  public LockoutExampleCmd() {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Running Lockout example.");
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Done running Lockout example.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
