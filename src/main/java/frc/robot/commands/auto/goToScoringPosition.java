// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.util.PoseMath;
import frc.robot.Constants.ScoringBlock;

public class goToScoringPosition extends CommandBase {
  /** Creates a new goToScoringPosition. */

  ScoringBlock scoringPosition;
  PathConstraints constraints;
  SwerveDrivetrain sdt;
  PPSwerveControllerCommand pathCommand;

  //pick correct scoring pose based on alliance
  public goToScoringPosition(PathConstraints constraints, ScoringBlock scoringPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.scoringPosition = scoringPosition;
    this.constraints = constraints;
    sdt = RobotContainer.RC().drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int scoringBlock; 
    int scoringAdjusted;

    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) { //BLUE ALLIANCE
      //FOR BLUE: 2 for left (driver's point of view), 1 for center, 0 for right
      if(RobotContainer.RC().dc.Driver().leftBumper().getAsBoolean()) scoringBlock = 2;
      else if(RobotContainer.RC().dc.Driver().rightBumper().getAsBoolean()) scoringBlock = 0;
      else scoringBlock = 1;

      //FOR BLUE: left is largest index of scoring trio
      switch(scoringPosition){
        case Left:
          scoringAdjusted = 2;
          break;
        case Center:
          scoringAdjusted = 1;
          break;
        default:
        case Right:
          scoringAdjusted = 0;
          break;      
      }
      pathCommand = MoveToPoseAutobuilder(constraints, Constants.FieldPoses.blueScorePoses[scoringBlock][scoringAdjusted]);
    }
    else { //RED ALLIANCE
      //FOR RED: 0 for left (driver's point of view), 1 for center, 2 for right
      if(RobotContainer.RC().dc.Driver().leftBumper().getAsBoolean()) scoringBlock = 0;
      else if(RobotContainer.RC().dc.Driver().rightBumper().getAsBoolean()) scoringBlock = 2;
      else scoringBlock = 1;

      //FOR RED: left is smallest index of scoring trio
      switch(scoringPosition){
        case Left:
          scoringAdjusted = 0;
          break;
        case Center:
          scoringAdjusted = 1;
          break;
        default:
        case Right:
          scoringAdjusted = 2;
          break;      
      }
      pathCommand = MoveToPoseAutobuilder(constraints, Constants.FieldPoses.redScorePoses[scoringBlock][scoringAdjusted]);
    }
    pathCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }

  public PPSwerveControllerCommand MoveToPoseAutobuilder(PathConstraints constraints, Pose2d finalPose) {
    //takes contraints, final pose - returns a command to move from current post to final pose

    Rotation2d bearing = PoseMath.getHeading2Target(sdt.getPose(), finalPose); //direction directly from point A to B.
    //using bearing as your exit and entry angle
    PathPoint startPoint = new PathPoint(sdt.getPose().getTranslation(), bearing, sdt.getPose().getRotation());
    PathPoint endPoint = new PathPoint(finalPose.getTranslation(), bearing, finalPose.getRotation());
    System.out.println("From Point:" + sdt.getPose().getTranslation());
    System.out.println("End Point:" + finalPose.getTranslation());

    //create a path from current position to finalPoint
    PathPlannerTrajectory newPath = PathPlanner.generatePath(constraints, startPoint, endPoint);
    System.out.println("Path time: " + newPath.getTotalTimeSeconds());
    
    PPSwerveControllerCommand pathCommand = new PPSwerveControllerCommand(
      newPath, 
      sdt::getPose, // Pose supplier
      sdt.getKinematics(), // SwerveDriveKinematics
      new PIDController(4.0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      new PIDController(4.0, 0, 0), // Y controller (usually the same values as X controller)
      new PIDController(2.0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      sdt::drive, // Module states consumer
      false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      sdt // Requires this drive subsystem
    );
    return pathCommand;
  }
}
