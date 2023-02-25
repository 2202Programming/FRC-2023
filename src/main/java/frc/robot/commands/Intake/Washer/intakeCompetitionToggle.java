// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.Washer;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.RobotContainer;
import frc.robot.commands.JoystickRumbleEndless;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.Intake;

public class intakeCompetitionToggle extends CommandBase implements BlinkyLightUser {
  /** Creates a new intakeCompetitionToggle. */
  Intake intake;
  JoystickRumbleEndless rumbleCommand;
  Color8Bit myColor = new Color8Bit(255, 255, 0);

  public intakeCompetitionToggle() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = RobotContainer.RC().intake;
    addRequirements(intake);
  }

  @Override
  public Color8Bit colorProvider() {
      return myColor;
  }

  @Override
  public boolean requestBlink(){
    return true;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.deploy();
    intake.intakeOn();
    intake.carwashOn();
    enableLights();
    rumbleCommand = new JoystickRumbleEndless(Id.Operator, RumbleType.kBothRumble);
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
    rumbleCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}