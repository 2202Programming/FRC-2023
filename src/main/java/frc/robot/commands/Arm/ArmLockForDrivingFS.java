// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.MoveCollectiveArm.Positions;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Claw_Substyem.ClawTrackMode;
import frc.robot.subsystems.Elbow;

public class ArmLockForDrivingFS extends CommandBase {

  // hardware commanded
  ArmSS arm = RobotContainer.RC().armSS;
  Elbow elbow = RobotContainer.RC().elbow;
  Claw_Substyem claw = RobotContainer.RC().claw;

  Command errorCmd = new PrintCommand("Not sure how to move arm to frontside.");
  Command on_backside_safe;
  Command on_backside_mv2flip;
  Command on_frontside;
  
  Command cmd;  //this is what will run

  Positions arm_current;
  double ELBOW_RETRACT = 140.0; // [deg/s] use slower speed
  double ARM_RETRACT = 20.0;
  double FLIP_TIME = 1.0; // [s] for waiting when flip in safe area
  Positions safe = CollectivePositions.safeToFlip.pos_info;

  /**
   * Moves arm to a frontside position and locks it
   * down for driving with a piece. 
   * 
   */
  public ArmLockForDrivingFS() {

    Positions move_slow = new Positions(CollectivePositions.travelFS);
    move_slow.armMaxVel = ARM_RETRACT;
    move_slow.elbowMaxVel = ELBOW_RETRACT;
    
    // on front-side, stay there and fold in.
    on_frontside = new SequentialCommandGroup(
        // front tracking already enabled        
        new MoveCollectiveArm(move_slow),
        new MoveCollectiveArm(CollectivePositions.travelLockFS)    );         
    on_frontside.setName("safe fs move");


    on_backside_safe = new SequentialCommandGroup(
        // backside tracking already enabled
        new InstantCommand(() -> { claw.setTrackElbowMode(ClawTrackMode.frontSide); }),
        new WaitUntilCommand(claw::wristAtSetpoint).withTimeout(FLIP_TIME),
        new MoveCollectiveArm(move_slow),
        new MoveCollectiveArm(CollectivePositions.travelLockFS)    );
    on_backside_safe.setName("backside safe to flip to fs");

    // on front-side but outside flip, move elbow out flip, move back
    on_backside_mv2flip = new SequentialCommandGroup(
        //need to move to a flip point with elbow
        new MoveElbow(safe.elbowPos, safe.elbowMaxVel),
        new InstantCommand(() -> { claw.setTrackElbowMode(ClawTrackMode.frontSide); }),
        new WaitUntilCommand(claw::wristAtSetpoint).withTimeout(FLIP_TIME),
        new MoveCollectiveArm(move_slow),
        new MoveCollectiveArm(CollectivePositions.travelLockFS)    );        
    on_backside_mv2flip.setName("backside move out to flip_first");            
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cmd = errorCmd;
    arm_current = getStart();

    // this move the claw to the nearest elbow track mode
    // it should always be safe because tracking is clear of the robot frame.
    var mode = claw.setNearestClawTrackMode();

    switch (mode) {
      case frontSide :
        cmd = on_frontside;
        break;

      case backSide:
        cmd = (safe2flip()) ? on_backside_safe : on_backside_mv2flip;
        break;

      case free:
      default:
        // This is an error we should be on one side or other
        cmd = errorCmd;
        break;

    }
    cmd.schedule();
    // cmd.unless(null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("*********ArmLockForDrivingFS ended, interrupted " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !cmd.isScheduled();
  }

  /*
   * safe2flip()
   * Looks at the start position
   */
  boolean safe2flip() {   
    if ((arm_current.elbowPos >= safe.elbowPos) &&
        (arm_current.armPos >= safe.armPos)) {
      return true;
    }
    // could test another point
    return false;
  }

  // read our current arm positions
  Positions getStart() {
    return new Positions(
        arm.getPosition(),
        elbow.getPosition(),
        claw.getWristAngle(),
        claw.getTrackElbowMode(),
        arm.getMaxVel(),
        elbow.getMaxVel());
  }

}
