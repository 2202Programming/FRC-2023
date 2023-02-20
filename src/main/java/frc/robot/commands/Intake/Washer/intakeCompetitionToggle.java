// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.Washer;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.commands.JoystickRumbleEndless;
import frc.robot.subsystems.Intake;

public class intakeCompetitionToggle extends CommandBase {
  /** Creates a new intakeCompetitionToggle. */
  Intake intake;
  JoystickRumbleEndless rumbleCommand;

  public intakeCompetitionToggle() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = RobotContainer.RC().intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.deploy();
    intake.intakeOn();
    intake.carwashOn();
    RobotContainer.RC().lights.setBlinking(new Color8Bit(255, 255, 0));
    rumbleCommand = new JoystickRumbleEndless(Id.Operator);
    rumbleCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.intakeOff();
    intake.carwashOff();
    intake.retract();
    RobotContainer.RC().lights.stopBlinking();
    RobotContainer.RC().blinkyLights();
    rumbleCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
