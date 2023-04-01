// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.ElbowMoveTo;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Arm.WristMoveTo;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Claw_Substyem.ClawTrackMode;
import frc.robot.subsystems.Elbow;

public class PlaceHighTele extends SequentialCommandGroup {
  /** Creates a new PlaceHighAuto. */
  private Claw_Substyem claw = RobotContainer.RC().claw;
  private Elbow elbow = RobotContainer.RC().elbow;
  private ArmSS arm = RobotContainer.RC().armSS;

  public PlaceHighTele() {
   addCommands(
        // new ParallelCommandGroup(
        //     new ElbowMoveTo(20.0),
        //     new WristMoveTo(0.0),
        //     new InstantCommand(() -> {
        //         claw.setTrackElbowMode(ClawTrackMode.faceDown);
        //     })
        // ),
        // new ParallelCommandGroup(
        //     new ElbowMoveTo(45.0),
        //     new WristMoveTo(-45.0)
        // ),
        // new ParallelCommandGroup(
        //     new ElbowMoveTo(100.0),
        //     new WristMoveTo(-90.0)
        // ),
        new ParallelCommandGroup(
            new ElbowMoveTo(25.0),
            new WristMoveTo(-25.0)
        ),
        new InstantCommand(() -> {
            claw.setTrackElbowMode(ClawTrackMode.faceDown);
        }),
        new ElbowMoveTo(155.0),
        new MoveCollectiveArm(CollectivePositions.placeConeHighFS),
        new InstantCommand(() -> {
          claw.open();
        }));

    // Use addRequirements() here to declare subsystem dependencies.
      }
}
