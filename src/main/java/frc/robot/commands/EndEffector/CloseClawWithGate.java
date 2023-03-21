// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw_Substyem;

public class CloseClawWithGate extends CommandBase {
  final Claw_Substyem claw = RobotContainer.RC().claw;
  Timer closeTimer = new Timer();
  double waitPeriod = 5.3;
  boolean done;
  
  /** Creates a new CloseClawWithGate. */
  public CloseClawWithGate() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.open();
    closeTimer.stop();
    System.out.printf("Claw auto open Started");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (claw.isGateBlocked()) {
      claw.close();
      closeTimer.start();
      System.out.printf("ClawTimer Started");
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Claw Close by light gate finished");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return closeTimer.hasElapsed(waitPeriod);
  }
}
