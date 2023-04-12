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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.HorizontalSubstationLane;
import frc.robot.RobotContainer;
import frc.robot.commands.JoystickRumbleEndless;
import frc.robot.commands.Automation.CenterTapeYaw;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.util.PoseMath;

public class goToScoringPosition extends CommandBase {
  /** Creates a new goToScoringPosition. */

  HorizontalScoringLane horizontalScoringLane;
  HorizontalSubstationLane horizontalSubstationLane;
  PathConstraints constraints;
  SwerveDrivetrain sdt;
  PPSwerveControllerCommand pathCommand;
  JoystickRumbleEndless rumbleCmd;
  Pose2d targetPose;
  int loopNum = 0;
  double distanceTolerance = 0.05; //in meters
  boolean pathingFinished = false;
  boolean tapeFinished = false;
  boolean allFinished = false;
  int maxLoops = 3;
  CenterTapeYaw tapeCommand;

  /**
   * Constructs a goToScoringPosition
   *
   * @param constraints path contraint object
   * @param horizontalScoringLane Which of three macro station to go to (left/right/center)
   * @param horizontalSubstationLane which lane of the station (micro scale) to go to (left/right/center)
   */
  public goToScoringPosition(PathConstraints constraints, HorizontalScoringLane horizontalScoringLane, HorizontalSubstationLane horizontalSubstationLane) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.horizontalScoringLane = horizontalScoringLane;
    this.constraints = constraints;
    this.horizontalSubstationLane = horizontalSubstationLane;
    sdt = RobotContainer.RC().drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int scoringBlock; 
    int scoringAdjusted;
    loopNum = 0;
    pathingFinished = false;
    tapeFinished = false;
    allFinished = false;
    tapeCommand = new CenterTapeYaw();

    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) { //BLUE ALLIANCE
      //FOR BLUE: 2 for left (driver's point of view), 1 for center, 0 for right
      if(horizontalSubstationLane.equals(HorizontalSubstationLane.Left)) scoringBlock = 2;
      else if(horizontalSubstationLane.equals(HorizontalSubstationLane.Right)) scoringBlock = 0;
      else scoringBlock = 1;

      //FOR BLUE: left is largest index of scoring trio
      switch(horizontalScoringLane){
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
      targetPose = Constants.FieldPoses.blueScorePoses[scoringBlock][scoringAdjusted];
    }
    else { //RED ALLIANCE
      //FOR RED: 0 for left (driver's point of view), 1 for center, 2 for right
      if(horizontalSubstationLane.equals(HorizontalSubstationLane.Left)) scoringBlock = 0;
      else if(horizontalSubstationLane.equals(HorizontalSubstationLane.Right)) scoringBlock = 2;
      else scoringBlock = 1;

      //FOR RED: left is smallest index of scoring trio
      switch(horizontalScoringLane){
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
      targetPose = Constants.FieldPoses.blueScorePoses[scoringBlock][scoringAdjusted];
    }
    pathCommand = MoveToPoseAutobuilder(constraints, targetPose);
    sdt.disableVisionPose();
    RobotContainer.RC().limelight.enableLED();
    RobotContainer.RC().limelight.setPipeline(1); //no more april tag update during the move, get ready for tape
    rumbleCmd = new JoystickRumbleEndless(Id.Operator);
    //rumbleCmd.schedule();
    RobotContainer.RC().lights.setBlinking(BlinkyLights.GREEN);
    pathCommand.schedule();
    loopNum++;
    System.out.println("***Loop #" + loopNum);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(tapeCommand.isScheduled()) return; //nothing to do, waiting for LL tape to complete

    if(pathCommand.isFinished()){ //once path command finishes, check how close to target it is, and how many loops we've done
      if(isAtTarget() || (loopNum == maxLoops)) { //either at target, or max # of loops
        pathingFinished = true;
        
        if((horizontalSubstationLane == HorizontalSubstationLane.Left) || (horizontalSubstationLane == HorizontalSubstationLane.Right)){ //cone station
          System.out.println("*** Running Tape Yaw... ");
          tapeCommand.schedule();
          tapeFinished = false;
        }
        else { //not a cone station, no need for RR tape turn
          tapeFinished = true;
        }
      }
      else{ //run path command again to close error
        loopNum++;
        System.out.println("***Distance error too big, starting Loop #" + loopNum);
        pathCommand = MoveToPoseAutobuilder(constraints, targetPose); //refresh pathCommand, since we are at a new pose (same target though)
        pathCommand.schedule();
      }

    }
  }

  private boolean isAtTarget(){
    double distance = frc.robot.util.PoseMath.poseDistance(targetPose, sdt.getPose());
    System.out.println("*** Loop#" + loopNum +" done, distance error: "+distance);
    return (distance < distanceTolerance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
    sdt.stop();
    RobotContainer.RC().limelight.disableLED();
    RobotContainer.RC().limelight.setPipeline(0); //back to apriltag pipe
    sdt.enableVisionPose();
    sdt.disableVisionPoseRotation();
    rumbleCmd.cancel();
    RobotContainer.RC().lights.stopBlinking();
    RobotContainer.RC().lights.setAllianceColors();
    System.out.println("***Final End Point:" + sdt.getPose().getTranslation() + ", rot:" + sdt.getPose().getRotation().getDegrees());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pathingFinished && tapeFinished) return true; //both are done
    else if (pathingFinished && !tapeFinished) {//pathing done, tape running
      if(tapeCommand.isFinished()) return true; //pathing done, tape is now done
    }
    return false;
  }

  public PPSwerveControllerCommand MoveToPoseAutobuilder(PathConstraints constraints, Pose2d finalPose) {
    //takes contraints, final pose - returns a command to move from current post to final pose

    Rotation2d bearing = PoseMath.getHeading2Target(sdt.getPose(), finalPose); //direction directly from point A to B.
    System.out.println("***BEARING = " + bearing);
    //using bearing as your exit and entry angle
    PathPoint startPoint = new PathPoint(sdt.getPose().getTranslation(), bearing, sdt.getPose().getRotation());
    PathPoint endPoint = new PathPoint(finalPose.getTranslation(), bearing, finalPose.getRotation());
    System.out.println("From Point:" + sdt.getPose().getTranslation() + ", rot:" + sdt.getPose().getRotation().getDegrees());
    System.out.println("Expected End Point:" + finalPose.getTranslation()  + ", rot:" + finalPose.getRotation().getDegrees());
    
    PIDController anglePid = new PIDController(6.0, 0, 0);
    anglePid.setTolerance(Math.PI * 0.02);
    anglePid.enableContinuousInput(-Math.PI, Math.PI);

    PIDController xPid = new PIDController(6.0, 0, 0);
    PIDController yPid = new PIDController(6.0, 0, 0);
    xPid.setTolerance(0.01);
    yPid.setTolerance(0.01);

    //create a path from current position to finalPoint
    PathPlannerTrajectory newPath = PathPlanner.generatePath(constraints, startPoint, endPoint);
    System.out.println("Path time: " + newPath.getTotalTimeSeconds());
    
    PPSwerveControllerCommand pathCommand = new PPSwerveControllerCommand(
      newPath, 
      sdt::getPose, // Pose supplier
      sdt.getKinematics(), // SwerveDriveKinematics
      xPid, // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      yPid, // Y controller (usually the same values as X controller)
      anglePid, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      sdt::drive, // Module states consumer
      false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      sdt // Requires this drive subsystem
    );
    return pathCommand;
  }
}
