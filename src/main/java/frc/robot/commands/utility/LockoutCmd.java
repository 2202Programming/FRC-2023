// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utility;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LockoutCmd extends CommandBase {

  Command protected_cmd;
  double lockout_period;
  Timer last_run = new Timer();
  boolean ok = false; // ok to run protected command

  /** Creates a new Lockout. */
  public LockoutCmd(Command protected_cmd, double lockout_period) {
    this.protected_cmd = protected_cmd;
    this.lockout_period = lockout_period;
    last_run.start();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (last_run.hasElapsed(lockout_period)) // elapsed > lockout_period)
    {
      //debug System.out.println("Command " + protected_cmd.getName()+" running.");
      last_run.restart();
      protected_cmd.initialize();
      ok = true;
    } else {
      double wait = Math.floor(lockout_period - last_run.get());
      System.out.println("Command " + protected_cmd.getName()+" LOCKED OUT, wait: " + wait);
      ok = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ok) {
      protected_cmd.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (ok) {
      last_run.reset();
      protected_cmd.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!ok)
      return true;
    return protected_cmd.isFinished();
  }
}
