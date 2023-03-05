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
  ArmSS arm = RobotContainer.RC().armSS;
  Elbow elbow = RobotContainer.RC().elbow;
  Claw_Substyem claw = RobotContainer.RC().claw;
  enum clawGoal{frontSide, backSide};
  clawGoal initialClawSide;
  final double minFlip = 70.0;
  final double maxFlip = 120.0;

  double elbow_maxVel = 60.0;

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

  // {arm, elbow, clawOffsetTrack}, wrist tracks elbow for now [cm, deg, deg]

   final Positions travel  = new Positions (0.0, 10.0, -90.0, 0.0, clawGoal.frontSide); // wrist/track need to be done
   final Positions mid = new Positions (20.0, 90.0, 90.0, 0.0, clawGoal.frontSide); // wrist/track need to be done
   final Positions high = new Positions (35.0, 120.0, 90.0,0.0, clawGoal.frontSide); // wrist/track need to be done
   final Positions travelMid = new Positions(10.0, 50.0, 90.0,0.0, clawGoal.frontSide); // wrist/track need to be done

   final Positions positions[] = {travel, mid, high, travelMid};

  final Positions target;
 public enum CollectiveMode {
    travel, mid, high, travelMid
  };

  /** Creates a new MoveCollectiveArm. */
  public MoveCollectiveArm(CollectiveMode where) {
    target = positions[where.ordinal()];
    addRequirements(arm, elbow, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setSetpoint(target.armPos);
    elbow.setSetpoint(target.elbowPos);
    elbow.setMaxVel(elbow_maxVel);
    claw.setElbowDoubleSupplier(elbow::getPosition);
    claw.setTrackElbow(true); 
    initialClawSide = claw.getElbowOffset() > 0.0 ? clawGoal.frontSide : clawGoal.backSide;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Safe to transition sides?
    if(target.side == initialClawSide) return;
    
    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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
