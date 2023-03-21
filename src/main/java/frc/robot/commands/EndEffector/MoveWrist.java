// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw_Substyem;

public class MoveWrist extends CommandBase {
  final Claw_Substyem claw;
  final double angle;
  final double maxVel;

  double old_maxVel;
 
  /** Creates a new GamePieceAngle. */
  public MoveWrist(double angle) {
    this(angle, -1.0);
  }
  public MoveWrist(double angle, double maxVel)  {
    this.claw = RobotContainer.RC().claw;
    this.angle = angle;
    this.maxVel = (maxVel < 0.0) ? claw.getWrist().getMaxVel() : maxVel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    old_maxVel = claw.getWrist().getMaxVel();
    claw.setWristAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // nothing to do, but wait for the wrist to move
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.getWrist().setMaxVel(old_maxVel);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw.wristAtSetpoint();
  }
}
