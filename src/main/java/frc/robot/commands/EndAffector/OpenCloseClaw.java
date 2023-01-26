// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndAffector;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class OpenCloseClaw extends CommandBase {
  final Claw claw;
  final String name;
  final double clawWidth;
  public OpenCloseClaw(Claw claw, String name, double clawWidth) {
    this.claw = claw;
    this.name = name;
    this.clawWidth = clawWidth;

    // Use addRequirements() here to declare subsystem dependencies.
  }
  //Don't know if this can be done in the same class/file or if it needs separate class/files


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setClawPos(openClaw);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
