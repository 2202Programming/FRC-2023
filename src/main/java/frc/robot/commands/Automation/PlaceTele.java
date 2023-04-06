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
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Claw_Substyem.ClawTrackMode;

public class PlaceTele extends SequentialCommandGroup {
  /** Creates a new PlaceHighAuto.
   * 
   *  This command assumes the starting position is ArmLockForDrivingFS.
   *  If there is any doubt as to whether ArmLockForDrivingFS will be the starting position,
   *  before this command is called, make sure to call ArmLockForDrivingFS.
   */
  private Claw_Substyem claw = RobotContainer.RC().claw;

  public PlaceTele(CollectivePositions finalPosition) {
   addCommands(
        new ParallelCommandGroup( //start to unwind claw/cone from inside robot without going too far out to hit station
            new ElbowMoveTo(25.0),
            new WristMoveTo(-25.0)
        ),
        new InstantCommand(() -> {
            claw.setTrackElbowMode(ClawTrackMode.faceDown); //face claw down to remain tucked in as we rotate to place
        }),
        new ElbowMoveTo(145.0),  //get above the post, so when we rotate wrist & extend are we miss posts
        new MoveCollectiveArm(finalPosition));
        //new ElbowMoveTo(130.0)); TODO remove this
      }
}