// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmMoveTo;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.ElbowMoveTo;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Claw_Substyem.ClawTrackMode;

public class PlaceHighAuto extends SequentialCommandGroup {
  /** Creates a new PlaceHighAuto. */
  private Claw_Substyem claw = RobotContainer.RC().claw;
  private Elbow elbow = RobotContainer.RC().elbow;
  private ArmSS arm = RobotContainer.RC().armSS;

  public PlaceHighAuto() {
   addCommands(
        new InstantCommand(() -> {
          claw.close();
        }),
        new InstantCommand(() -> {
          claw.setTrackElbowMode(ClawTrackMode.backSide);
        }),
        new ArmMoveTo(10.0),
        new ElbowMoveTo(70.0),
        new InstantCommand(() -> {
          claw.setTrackElbowMode(ClawTrackMode.frontSide);
        }),
        new ElbowMoveTo(155.0),
        new ArmMoveTo(37.0),
        new ElbowMoveTo(135.0),
        new InstantCommand(() -> {
          claw.close();
        }),
        new MoveCollectiveArm(CollectivePositions.power_on));

    // Use addRequirements() here to declare subsystem dependencies.
      }
}
