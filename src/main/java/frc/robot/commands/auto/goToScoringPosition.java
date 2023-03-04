// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.MoveToPoseAutobuilder;

public class goToScoringPosition extends CommandBase {
  /** Creates a new goToScoringPosition. */

  ScoringTrio scoringPosition;
  PathConstraints constraints;

  public enum ScoringTrio {
    Left(0), Center(1), Right(2);

    public final int value;

    ScoringTrio(int value) {
      this.value = value;
    }
  }

  //pick correct scoring pose based on alliance
  public goToScoringPosition(PathConstraints constraints, ScoringTrio scoringPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.scoringPosition = scoringPosition;
    this.constraints = constraints;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int scoringBlock; 
    int scoringTrioAdjusted;

    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) { //BLUE ALLIANCE
      //FOR BLUE: 2 for left (driver's point of view), 1 for center, 0 for right
      if(RobotContainer.RC().dc.Driver().leftBumper().getAsBoolean()) scoringBlock = 2;
      else if(RobotContainer.RC().dc.Driver().rightBumper().getAsBoolean()) scoringBlock = 0;
      else scoringBlock = 1;

      //FOR BLUE: left is largest index of scoring trio
      switch(scoringPosition){
        case Left:
          scoringTrioAdjusted = 2;
          break;
        case Center:
          scoringTrioAdjusted = 1;
          break;
        default:
        case Right:
          scoringTrioAdjusted = 0;
          break;      
      }
      new MoveToPoseAutobuilder(constraints, Constants.FieldPoses.blueScorePoses[scoringBlock][scoringTrioAdjusted]).schedule();
    }
    else { //RED ALLIANCE
      //FOR RED: 0 for left (driver's point of view), 1 for center, 2 for right
      if(RobotContainer.RC().dc.Driver().leftBumper().getAsBoolean()) scoringBlock = 0;
      else if(RobotContainer.RC().dc.Driver().rightBumper().getAsBoolean()) scoringBlock = 2;
      else scoringBlock = 1;

      //FOR RED: left is smallest index of scoring trio
      switch(scoringPosition){
        case Left:
          scoringTrioAdjusted = 0;
          break;
        case Center:
          scoringTrioAdjusted = 1;
          break;
        default:
        case Right:
          scoringTrioAdjusted = 2;
          break;      
      }
      new MoveToPoseAutobuilder(constraints, Constants.FieldPoses.redScorePoses[scoringBlock][scoringTrioAdjusted]).schedule();
    }
    
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
