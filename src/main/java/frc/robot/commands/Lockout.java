// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Lockout extends CommandBase {

  Command protected_cmd;
  double lockout_period;
  Timer last_run = new Timer();
  double last_start = 0.0;
  boolean ok = false;

  /** Creates a new Lockout. */
  public Lockout(Command protected_cmd, double lockout_period) {
    this.protected_cmd = protected_cmd;
    this.lockout_period = lockout_period;
    last_run.reset();

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (last_start == 0.0 || last_run.get() > lockout_period)
    {
      last_run.start();
      protected_cmd.initialize();
      ok = true;
    }
    else {
      System.out.println("Command LOCKED OUT, ending");
      ok = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ok) protected_cmd.execute();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (ok) {
      protected_cmd.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!ok) return true;
    return protected_cmd.isFinished();
  }
}
