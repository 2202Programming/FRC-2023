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

public class ArmLockForDriving extends CommandBase {

  // hardware commanded
  ArmSS arm = RobotContainer.RC().armSS;
  Elbow elbow = RobotContainer.RC().elbow;
  Claw_Substyem claw = RobotContainer.RC().claw;

  Command errorCmd = new PrintCommand("Not sure how to move arm to backside.");
  Command on_backside;
  Command on_frontside;
  Command on_frontside_mv2flip;
  Command cmd;  //this is what will run

  Positions arm_current;
  double ELBOW_RETRACT = 60.0; // [deg/s] use slower speed
  double ARM_RETRACT = 16.0;
  double FLIP_TIME = 2.0; // [s] for waiting when flip in safe area
  Positions safe = CollectivePositions.safeToFlip.pos_info;

  /**
   * Moves arm to a backside position and locks it
   * down for driving. Looks starting location and picks
   * a sequence to move it.
   * 
   * 
   */
  public ArmLockForDriving() {

    Positions move_slow_power_on = new Positions(CollectivePositions.power_on);
    move_slow_power_on.armMaxVel = ARM_RETRACT;
    move_slow_power_on.elbowMaxVel = ELBOW_RETRACT;

    on_backside = new SequentialCommandGroup(
        new PrintCommand("*********Starting on_backside"),
        // backside tracking already enabled
        new MoveCollectiveArm(move_slow_power_on),
        new MoveCollectiveArm(CollectivePositions.travelLockNoPieceBS)
        //new PrintCommand("*********Ending on_backside")
        
        );
    on_backside.setName("already_on_bs");

    // on front-side need to flip safely and fold up
    on_frontside = new SequentialCommandGroup(
        new PrintCommand("*********Starting on_frontside"),
        // front tracking already enabled
        new InstantCommand(() -> { claw.setTrackElbowMode(ClawTrackMode.backSide); }),
        new WaitUntilCommand(claw::wristAtSetpoint).withTimeout(FLIP_TIME),
        new MoveCollectiveArm(move_slow_power_on),
        new MoveCollectiveArm(CollectivePositions.travelLockNoPieceBS),
        new PrintCommand("*********Ending on_frontside")
        );                
    on_frontside.setName("safe_flip_before_move");

    // on front-side but outside flip, move elbow out flip, move back
    on_frontside_mv2flip = new SequentialCommandGroup(
        new PrintCommand("*********Starting move to flip"),
        //need to move to a flip point with elbow
        new MoveElbow(safe.elbowPos, safe.elbowMaxVel),
        new InstantCommand(() -> { claw.setTrackElbowMode(ClawTrackMode.backSide); }),
        new WaitUntilCommand(claw::wristAtSetpoint).withTimeout(FLIP_TIME),
        new MoveCollectiveArm(move_slow_power_on),
        new MoveCollectiveArm(CollectivePositions.travelLockNoPieceBS),
        new PrintCommand("*********Ending move to flip")
        );          
        
    on_frontside_mv2flip.setName("move_out_to_flip_first");        
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("******init ArmLockForDriving cmd *****");
    cmd = errorCmd;
    arm_current = getStart();

    // this move the claw to the nearest elbow track mode
    // it should always be safe because tracking is clear of the robot frame.
    var mode = claw.setNearestClawTrackMode();
    System.out.println("==========Here's the mode: " + mode.toString());
    switch (mode) {
      case backSide:
        cmd = on_backside;
        break;

      case frontSide:
        cmd = (safe2flip()) ? on_frontside : on_frontside_mv2flip;
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
    System.out.println("*********ArmLockForDriving ended, interrupted " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    System.out.println("*********ArmLockForDriving cmd.sched" + cmd.isScheduled() + "  fin="+cmd.isFinished());    
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
