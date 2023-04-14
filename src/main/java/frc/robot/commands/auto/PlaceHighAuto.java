// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmLockForDrivingFS;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.ElbowMoveTo;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Arm.WristMoveTo;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Claw_Substyem.ClawTrackMode;

public class PlaceHighAuto extends SequentialCommandGroup {
  /** Creates a new PlaceHighAuto.
   * 
   *  This is gonna take like 10 seconds lol
   */
  private Claw_Substyem claw = RobotContainer.RC().claw;

  public PlaceHighAuto() {
   addCommands(
        new InstantCommand(() -> {
            claw.setTrackElbowMode(ClawTrackMode.backSide); //face claw up to not hit post as we rotate to place
        }),
        new ElbowMoveTo(25.0),  // safe to flip
        new InstantCommand(() -> {
            claw.setTrackElbowMode(ClawTrackMode.faceUp); //face claw up to not hit post as we rotate to place
        }),
        new ElbowMoveTo(115.0), // get above it. 115 is theoretical max we can move
        new MoveCollectiveArm(CollectivePositions.placeHighFS),
        new InstantCommand(() -> { claw.open();}),
        new WaitCommand(0.5),
        new ArmLockForDrivingFS()
        );
      }
}