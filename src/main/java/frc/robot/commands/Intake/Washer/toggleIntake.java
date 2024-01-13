// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.Washer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.RobotContainer;

public class toggleIntake extends Command {
  /** Creates a new toggleIntake. */
  final Intake intake;
  boolean intakePos;
  
  public toggleIntake() {
    intake =  RobotContainer.RC().intake;
    intakePos = intake.isDeployed();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intakePos = true) {
      intake.deploy();
    } if (intakePos = false) {
      intake.retract();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // do nothing
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // do nothing
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakePos != intake.isDeployed();
  }
}
