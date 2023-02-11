// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Intake;
import frc.robot.commands.Automation.CenterTapeSkew;
import frc.robot.commands.Automation.CenterTapeYaw;
import frc.robot.commands.Automation.CenterTapeYawSkew;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.FieldCentricDrive;
import frc.robot.commands.swerve.FollowPPTrajectory;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.RobotSpecs;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  static RobotContainer rc;

  // singleton accessor for robot public sub-systems
  public static RobotContainer RC() {
    return rc;
  }

  //control binding mode
  enum Bindings{
    Competition,    
    //below are testing modes, add as needed
    vision_test,
    balance_test,
    arm_test,
    claw_test
  }

  // What robot are we running?
  public final RobotSpecs robotSpecs;

  // Sub-systems
  public final Sensors_Subsystem sensors;
  public final SwerveDrivetrain drivetrain;
  public final HID_Xbox_Subsystem dc; // short for driver controls
  public final Intake intake;
  public final ArmSS armSS;
  public final Claw_Substyem claw;

  // vision systems, create on every bot
  public final PhotonVision photonVision = new PhotonVision();
  public final Limelight_Subsystem limelight = new Limelight_Subsystem();;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.rc = this; // for singleton accesor
    robotSpecs = new RobotSpecs(); // mechanism to pull different specs based on roborio SN
    dc = new HID_Xbox_Subsystem(0.3, 0.9, 0.05); // TODO: deal with driver prefs

    limelight.setPipeline(1);
    // Construct sub-systems based on robot Name Specs
    switch (robotSpecs.myRobotName) {
      case CompetitionBot:
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        intake = new Intake();
        armSS = new ArmSS();
        claw = new Claw_Substyem();
        break;

      case SwerveBot:
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        intake = null;
        armSS = null;
        claw = null;
        break;

      case BotOnBoard: // fall through
      case UnknownBot: // fall through
      default:
        sensors = null;
        drivetrain = null;
        intake = null;
        armSS = null;
        claw = null;
        break;
    }

    // set default commands, if sub-system exists
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new FieldCentricDrive(drivetrain));
    }

    // Edit the binding confiuration for testing
    configureBindings(Bindings.Competition);
  }


  private void configureBindings(Bindings bindings) {
    // bindings useful for everyone
    
    // Y button to reset current facing to zero
    if (drivetrain != null) {
      dc.Driver().y().whileTrue(new InstantCommand(() -> {
        drivetrain.resetAnglePose(new Rotation2d(0));    }));
    }

    // add bindings based on current user mode
    switch (bindings){
      case arm_test:
        break;
      case balance_test:
        dc.Driver().rightBumper().whileTrue(new ChargeStationBalance(false));
        break;

      case claw_test:
        break;

      case vision_test:
        // X button to change LL pipeline
        dc.Driver().leftBumper().onTrue(new InstantCommand(() -> {
          limelight.togglePipeline();
        }));
        dc.Driver().a().whileTrue(new CenterTapeYaw());
        dc.Driver().b().whileTrue(new CenterTapeSkew());
        dc.Driver().x().whileTrue(new CenterTapeYawSkew());      
        break;

      case Competition:
      default:
        //TODO: Put Competition bindings here
        break;      
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

<<<<<<< Updated upstream
    // return new FollowPPTrajectory(FollowPPTrajectory.pathFactoryTele(new PathConstraints(1, 1),
    //     new Pose2d(drivetrain.getPose().getX(),
    //         drivetrain.getPose().getY()+1,
    //         new Rotation2d(drivetrain.getPose().getRotation().getRadians() + Math.PI))),
    //     true);

    // return new SequentialCommandGroup(
    //     new FollowPPTrajectory(FollowPPTrajectory.pathFactoryAuto(new PathConstraints(1, 1),"rotate"),
    // false), 
    //     new PrintCommand("End of Path 1 Construction"),
    //     new WaitCommand(1),
    //     new FollowPPTrajectory(FollowPPTrajectory.pathFactoryAuto(new PathConstraints(1, 1),"rotate2"),
    // true),
    //     new PrintCommand("End of Path 2 Construction"));


ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("autopath1", new PathConstraints(1, 1));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("start", new SequentialCommandGroup(new PrintCommand("***Path Start"), new InstantCommand(drivetrain::printPose)));
eventMap.put("middle", new SequentialCommandGroup(new PrintCommand("***Path Middle"), new InstantCommand(drivetrain::printPose)));
eventMap.put("end", new SequentialCommandGroup(new PrintCommand("***Path End"), new InstantCommand(drivetrain::printPose), new ChargeStationBalance(true)));


// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    drivetrain::getPose, // Pose2d supplier
    drivetrain::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
    drivetrain.getKinematics(), // SwerveDriveKinematics
    new PIDConstants(4.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(2.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    drivetrain::drive, // Module states consumer used to output to the drive subsystem
    eventMap,
    false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    drivetrain // The drive subsystem. Used to properly set the requirements of path following commands
);

Command fullAuto = autoBuilder.fullAuto(pathGroup);
return fullAuto;
 }
=======
    return new FollowPPTrajectory(FollowPPTrajectory.pathFactoryTele(new PathConstraints(1, 1),
        new Pose2d(drivetrain.getPose().getX(),
            drivetrain.getPose().getY(),
            new Rotation2d(drivetrain.getPose().getRotation().getRadians() + Math.PI))),
        true);
    //return new FollowPPTrajectory(FollowPPTrajectory.pathFactoryAuto(new PathConstraints(1, 1),"rotate"),
    //false);
  }
>>>>>>> Stashed changes
}
