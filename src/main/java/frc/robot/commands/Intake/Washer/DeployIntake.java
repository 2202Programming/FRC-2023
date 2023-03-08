// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.Washer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.RobotContainer;

public class DeployIntake extends CommandBase {
  /** Creates a new activateIntake. */
  final Intake intake;

  public DeployIntake() {
    intake = RobotContainer.RC().intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.deploy();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // nothing needs to happen here
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // do nothing
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
