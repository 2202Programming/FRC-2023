// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndAffector;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class GamePieceAngle extends CommandBase {
  final Claw claw;
  final double angle;
  final String name;
  /** Creates a new GamePieceAngle. */
  public GamePieceAngle(Claw claw, String name, double angle) {
    this.claw = claw;
    this.name = name;
    this.angle = angle;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setAngle(angle);
    System.out.println("Moving to" + angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //0.0 or whatever is default
    claw.setAngle(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //when claw is at desired angle
    return claw.correctAngle;
  }
}
