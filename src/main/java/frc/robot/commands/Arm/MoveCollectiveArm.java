// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Elbow;

public class MoveCollectiveArm extends CommandBase {
  //hardware
  ArmSS arm = RobotContainer.RC().armSS;
  Elbow elbow = RobotContainer.RC().elbow;
  Claw_Substyem claw = RobotContainer.RC().claw;
  
  /*
   * claw level tracking options
   * 
   */
  enum clawGoal{frontSide, backSide};   //tbd delivery
  
  /*
   * Safe to flip zone for elbow angle
   */
  final double minFlip = 70.0;
  final double maxFlip = 120.0;

/**
 * State of colective, either start or target.
 */
  class Positions{
    double armPos;
    double elbowPos;
    double clawOffsetTrackPos;
    double wristPos;
    clawGoal side;

    Positions(double arm, double elbow, double clawOffsetTrack, double wrist, clawGoal side){
      armPos = arm;
      elbowPos = elbow;
      clawOffsetTrackPos = clawOffsetTrack;
      wristPos = wrist;
      this.side = side;
    }
  }

  // filled in at init based on where we are
  Positions start;
  Positions target;

  /*
   * CollectiveMode names the target
   */
  public enum CollectiveMode {
    power_on,
    travel, mid, high, travelMid
  };

  // {arm, elbow, clawOffsetTrack}, wrist tracks elbow for now [cm, deg, deg]
  
  Positions power_on = new Positions (0.0, 10.0, -90.0, 0.0, clawGoal.frontSide); 
  Positions travel  = new Positions (0.0, 10.0, -90.0, 0.0, clawGoal.frontSide); // wrist/track need to be done
  Positions mid = new Positions (20.0, 90.0, 90.0, 0.0, clawGoal.frontSide); // wrist/track need to be done
  Positions high = new Positions (35.0, 120.0, 90.0,0.0, clawGoal.frontSide); // wrist/track need to be done
  Positions travelMid = new Positions(10.0, 50.0, 90.0,0.0, clawGoal.frontSide); // wrist/track need to be done

  Positions positions[] = {travel, mid, high, travelMid};

 
  /** Creates a new MoveCollectiveArm. */
  public MoveCollectiveArm(CollectiveMode where) {
    target = positions[where.ordinal()];
    addRequirements(arm, elbow, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //capture where we are
    start = getStart();
    
    //move towards our target
    arm.setSetpoint(target.armPos);
    elbow.setSetpoint(target.elbowPos);
    
    //wrist will be done in exec to make sure we are safe
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Safe to transition sides?
    if (target.side == start.side) return;
    
    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  /*
   * getStart() captures where we are now, ignoring velocities 
   */
  Positions getStart() {
    return new Positions(
      arm.getPosition(), 
      elbow.getPosition(),
      claw.getElbowOffset(),
      claw.getWristAngle(),
      claw.getElbowOffset() > 0.0 ? clawGoal.frontSide : clawGoal.backSide);
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return (arm.atSetpoint() && elbow.atSetpoint() );
  }

  void setFrontSide(){
    claw.setElbowOffset(90.0);
  }
  void setBackSide(){
    claw.setElbowOffset(-90.0);
  }
}
