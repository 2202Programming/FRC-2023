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
import frc.robot.util.RobotSpecs.RobotNames;

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

  // Sub-systems
  public final RobotSpecs robotSpecs;
  public final Sensors_Subsystem sensors; 
  public Limelight_Subsystem limelight;
  public final SwerveDrivetrain drivetrain;
  public final Intake intake;
  public final HID_Xbox_Subsystem dc; // short for driver controls
  public final PhotonVision photonVision;
  public ArmSS armSS = null;
  public Claw_Substyem claw = null;


  /**Z
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.rc = this; // for singleton accesor

    // initialize all the sub-systems
    robotSpecs = new RobotSpecs(); // mechanism to pull different specs based on roborio
    dc = new HID_Xbox_Subsystem(0.3, 0.9, 0.05); // TODO: deal with driver prefs

    // these can get created on any hardware setup
    sensors = new Sensors_Subsystem();
    intake = new Intake();
    drivetrain = new SwerveDrivetrain();
    photonVision = new PhotonVision();
    if (robotSpecs.myRobotName == RobotNames.CompetitionBot) {
      armSS = new ArmSS();
      claw = new Claw_Substyem();
    }

    // set default commands
    drivetrain.setDefaultCommand(new FieldCentricDrive(drivetrain));

    // Configure the trigger bindings
    configureBindings();
  }


  //TODO: FIGURE OUT A WAY TO HANDLE DIFFERENT TEMPORARY BUTTON BINDINGS
  private void configureBindings() {

    // X button to change LL pipeline
    dc.Driver().b().onTrue(new InstantCommand(() -> {
      limelight.togglePipeline();
    }));

    if (robotSpecs.myRobotName != RobotNames.BotOnBoard) {
      // Y button to reset current facing to zero
      dc.Driver().y().whileTrue(new InstantCommand(() -> {
        drivetrain.resetAnglePose(new Rotation2d(0));
      }));
      dc.Driver().a().whileTrue(new CenterTapeYaw());
      dc.Driver().b().whileTrue(new CenterTapeSkew());
      dc.Driver().x().whileTrue(new CenterTapeYawSkew());
    }

    // Y button to reset current facing to zero
    dc.Driver().y().whileTrue(new InstantCommand(() -> {
      drivetrain.resetAnglePose(new Rotation2d(0));
    }));

    dc.Driver().x().whileTrue(new ChargeStationBalance(false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new FollowPPTrajectory(FollowPPTrajectory.pathFactoryTele(new PathConstraints(1, 1),
        new Pose2d(drivetrain.getPose().getX(),
            drivetrain.getPose().getY()+1,
            new Rotation2d(drivetrain.getPose().getRotation().getRadians() + Math.PI))),
        true);
    //return new FollowPPTrajectory(FollowPPTrajectory.pathFactoryAuto(new PathConstraints(1, 1),"rotate"),
    //false);
  }
}
