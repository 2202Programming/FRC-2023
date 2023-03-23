// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.Washer.outtakeCompetitionToggle;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.swerve.RotateTo;
import frc.robot.commands.swerve.VelocityMove;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

public class PlaceHybrid extends CommandBase {
  // State vars
  SequentialCommandGroup cmd;
  HorizontalScoringLane horizontalRequest;

  // Constants
  final double DEADZONE2 = 0.025; // [%^2] squared number

  // SSs
  HID_Xbox_Subsystem dc = RobotContainer.RC().dc; 

  
  /** Creates a new PlaceHybrid. */
  public PlaceHybrid(HorizontalScoringLane horizontalRequest) {
    this.horizontalRequest = horizontalRequest;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup();

    // 1. Move to place
    cmd.addCommands(
      new goToScoringPosition(new PathConstraints(2.0, 3.0), horizontalRequest), 
      new RotateTo(new Rotation2d((DriverStation.getAlliance().equals(Alliance.Blue)) ? 180 : 0)),
      new VelocityMove(1.0, 0.0, 0.5) 
      // 0.5 magic number results in an integral of 1/2m because that's how much we're off by 
      );

    // 2. Eject
    cmd.addCommands(new outtakeCompetitionToggle().withTimeout(3.0));

    // Schedule cmd, add stop condition
    cmd.until(() -> {
      boolean xStickStill = (Math.pow(dc.Driver().getLeftX(), 2) + Math.pow(dc.Driver().getLeftY(), 2)) > DEADZONE2;
      boolean yStickStill = (Math.pow(dc.Driver().getRightX(), 2) + Math.pow(dc.Driver().getRightY(), 2)) > DEADZONE2;
      return !(xStickStill && yStickStill);
    }).schedule();
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
    return true;
  }
}
