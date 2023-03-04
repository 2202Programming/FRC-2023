// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.swerve.MoveToPoseAutobuilder;

public class goToScoringPosition extends CommandBase {
  /** Creates a new goToScoringPosition. */

  int scoringPosition;
  PathConstraints constraints;


  //pick correct scoring pose based on alliance
  public goToScoringPosition(PathConstraints constraints, int scoringPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.scoringPosition = scoringPosition;
    this.constraints = constraints;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      new MoveToPoseAutobuilder(constraints, Constants.FieldPoses.blueScorePoses[scoringPosition-1]).schedule();
    }
    else{
      new MoveToPoseAutobuilder(constraints, Constants.FieldPoses.redScorePoses[scoringPosition-1]).schedule();;
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
