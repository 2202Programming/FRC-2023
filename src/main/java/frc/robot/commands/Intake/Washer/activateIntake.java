// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.Washer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Carwash;


public class activateIntake extends CommandBase {
  /** Creates a new activateIntake. */
  final Intake intake;
  final Carwash washer;

  public activateIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.off();
    washer.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Idea: when the button is released edit a variable to return false
    //We may also want to end it when 2/3 color sensors are triggered
    return false;
  }
}
