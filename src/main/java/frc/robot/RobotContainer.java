// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
      case CompetitionBot2023:
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
    // An example command will be run in autonomous
    return new FollowPPTrajectory(FollowPPTrajectory.pathFactoryTele(new PathConstraints(1, 1),
        new Pose2d(drivetrain.getPose().getX(),
            drivetrain.getPose().getY(),
            new Rotation2d(drivetrain.getPose().getRotation().getRadians() + Math.PI))),
        true);
  }
}
