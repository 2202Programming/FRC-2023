// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.Washer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.Intake;

public class outtakeCompetitionToggle extends CommandBase {
  /** Creates a new intakeCompetitionToggle. */
  Intake intake;
  ColorSensors colorSensors = RobotContainer.RC().colorSensors;
  public outtakeCompetitionToggle() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = RobotContainer.RC().intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.deploy();
    intake.intakeOnReverse();
    intake.carwashOnReverse();

    colorSensors.resetOuttakeFrames();
    colorSensors.updateOuttakeFrames(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.retract();
    intake.intakeOff();
    intake.carwashOff();

    colorSensors.updateOuttakeFrames(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (DriverStation.isAutonomousEnabled()) return colorSensors.getObjectOuttake();
    return false;
  }
}
