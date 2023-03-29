package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.HorizontalSubstationLane;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.swerve.RotateTo;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.util.PoseMath;

public class MoveToFactory extends CommandBase {
    // controlllers
    CommandXboxController driver = RobotContainer.RC().dc.Driver();
    CommandXboxController operator = RobotContainer.RC().dc.Operator();

    // state vars
    HorizontalScoringLane horizLane;
    HorizontalSubstationLane subLane;
    CollectivePositions armPos;
    SequentialCommandGroup cmd;

    /**
     * Creates a new MoveToFactory object.
     * 
     * @param triggers The triggers currently used so they're exposed in RobotContainer
     */
    public MoveToFactory(Trigger... triggers) {
       // do nothing
    }

    public void initialize() {
        cmd = new SequentialCommandGroup();
        // Station lane
        if (driver.leftBumper().getAsBoolean()) horizLane = HorizontalScoringLane.Left;
        else if (driver.rightBumper().getAsBoolean()) horizLane = HorizontalScoringLane.Right;
        else horizLane = HorizontalScoringLane.Center;

        // Substation lane
        if (operator.leftBumper().getAsBoolean()) subLane = HorizontalSubstationLane.Left;
        else if (operator.rightBumper().getAsBoolean()) subLane = HorizontalSubstationLane.Right;
        else subLane = HorizontalSubstationLane.Center;

        // Arm mid/high
        armPos = (operator.povUp().getAsBoolean()) ? CollectivePositions.placeConeHighFS : CollectivePositions.placeConeMidFS;

        Rotation2d scoreRotation = new Rotation2d((DriverStation.getAlliance().equals(Alliance.Blue)) ? 0.0 : 180.0);

        System.out.println("Ready to create SCG, Horizontal Scoring Lane: " + horizLane.toString() + 
                            ", Substation Lane: " + subLane.toString() + ", arm position: " + armPos.toString());

        cmd.addCommands(
            new PrintCommand("Starting SCG"),
            goToScoringPos(new PathConstraints(2.0, 3.0), horizLane, subLane),
            new PrintCommand("End autopath"),
            new RotateTo(scoreRotation),
            new PrintCommand("End rotation"),
            new MoveCollectiveArm(armPos),
            new PrintCommand("End SCG")
        );
        
        // bail when the driver says so
        cmd.
        // until(() -> {
        //     boolean xStickStill = (Math.pow(driver.getLeftX(), 2) + Math.pow(driver.getLeftY(), 2)) > DEADZONE2; 
        //     boolean yStickStill = (Math.pow(driver.getRightX(), 2) + Math.pow(driver.getRightY(), 2)) > DEADZONE2;
        //     return !(xStickStill && yStickStill);
        //   }).
          schedule();
    }

    @Override
    public boolean isFinished() {return true;}

    public static Command goToScoringPos(PathConstraints constraints, HorizontalScoringLane horizLane, HorizontalSubstationLane subLane) {
    Command pathCommand;
    int scoringBlock; 
    int scoringAdjusted;

    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) { //BLUE ALLIANCE
      //FOR BLUE: 2 for left (driver's point of view), 1 for center, 0 for right
      if(subLane.equals(HorizontalSubstationLane.Left)) scoringBlock = 2;
      else if(subLane.equals(HorizontalSubstationLane.Right)) scoringBlock = 0;
      else scoringBlock = 1;

      //FOR BLUE: left is largest index of scoring trio
      switch(horizLane){
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
      if(subLane.equals(HorizontalSubstationLane.Left)) scoringBlock = 0;
      else if(subLane.equals(HorizontalSubstationLane.Right)) scoringBlock = 2;
      else scoringBlock = 1;

      //FOR RED: left is smallest index of scoring trio
      switch(horizLane){
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

    return pathCommand;
    }

    private static PPSwerveControllerCommand MoveToPoseAutobuilder(PathConstraints constraints, Pose2d finalPose) {
        SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
        
        //takes contraints, final pose - returns a command to move from current post to final pose
    
        Rotation2d bearing = PoseMath.getHeading2Target(sdt.getPose(), finalPose); //direction directly from point A to B.
        //using bearing as your exit and entry angle
        PathPoint startPoint = new PathPoint(sdt.getPose().getTranslation(), bearing, sdt.getPose().getRotation());
        PathPoint endPoint = new PathPoint(finalPose.getTranslation(), bearing, finalPose.getRotation());
        System.out.println("From Point:" + sdt.getPose().getTranslation() + ", rot:" + sdt.getPose().getRotation().getDegrees());
        System.out.println("Expected End Point:" + finalPose.getTranslation()  + ", rot:" + finalPose.getRotation().getDegrees());
        
        PIDController anglePid = new PIDController(6.0, 0, 0);
        anglePid.setTolerance(Math.PI * 0.02);
        anglePid.enableContinuousInput(-Math.PI, Math.PI);
        
        //create a path from current position to finalPoint
        PathPlannerTrajectory newPath = PathPlanner.generatePath(constraints, startPoint, endPoint);
        System.out.println("Path time: " + newPath.getTotalTimeSeconds());
        
        PPSwerveControllerCommand pathCommand = new PPSwerveControllerCommand(
          newPath, 
          sdt::getPose, // Pose supplier
          sdt.getKinematics(), // SwerveDriveKinematics
          new PIDController(4.0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(4.0, 0, 0), // Y controller (usually the same values as X controller)
          anglePid, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          sdt::drive, // Module states consumer
          false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          sdt // Requires this drive subsystem
        );
        return pathCommand;
      }
    
}
