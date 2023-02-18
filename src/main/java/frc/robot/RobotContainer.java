// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Intake;
import frc.robot.commands.Arm.ArmMoveAtSpeed;
import frc.robot.commands.Automation.CenterTapeSkew;
import frc.robot.commands.Automation.CenterTapeYaw;
import frc.robot.commands.Automation.CenterTapeYawSkew;
import frc.robot.commands.auto.autoCommand;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.ChargeStationBalanceChad;
import frc.robot.commands.swerve.FieldCentricDrive;
import frc.robot.commands.test.ArmVelocityTest;
import frc.robot.commands.test.MoveArmsTest;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Elbow;
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

  // control binding mode
  enum Bindings {
    Competition,
    // below are testing modes, add as needed
    vision_test,
    balance_test,
    arm_test,
    claw_test
  }

  // What robot are we running?
  public final RobotSpecs robotSpecs;

  // Sub-systems
  public final PhotonVision photonVision;
  public final Limelight_Subsystem limelight;
  public final Sensors_Subsystem sensors;
  public final SwerveDrivetrain drivetrain;
  public final HID_Xbox_Subsystem dc; // short for driver controls
  public final Intake intake;
  public final ArmSS armSS;
  public final Elbow elbow;
  public final Claw_Substyem claw;

  public HashMap<String, Command> eventMap;
  public SwerveAutoBuilder autoBuilder;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.rc = this; // for singleton accesor
    robotSpecs = new RobotSpecs(); // mechanism to pull different specs based on roborio SN
    dc = new HID_Xbox_Subsystem(0.3, 0.9, 0.05);

    // Construct sub-systems based on robot Name Specs
    switch (robotSpecs.myRobotName) {
      case CompetitionBot2023:
        photonVision = null; //new PhotonVision();
        limelight = null; //new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        intake = new Intake();
        //armSS = new ArmSS();
        //elbow = new Elbow();
        //claw = new Claw_Substyem();
        armSS = null;
        elbow = null;
        claw = null;
        break;

      case SwerveBot:
        photonVision = new PhotonVision();
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        intake = null;
        armSS = null;
        elbow = null;
        claw = null;
        break;

      case ChadBot:
        photonVision = new PhotonVision();
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        intake = null;
        armSS = null;
        elbow = null;
        claw = null;
        break;

      case BotOnBoard: // fall through
      case UnknownBot: // fall through
      default:
        photonVision = null;
        limelight = null;
        sensors = null;
        drivetrain = null;
        intake = null;
        //armSS = new ArmSS();
        armSS = null;
        elbow = null;
        claw = null;
        break;
    }

    if (limelight != null) {
      // apriltag is pipeline 0
      limelight.setPipeline(0);
    }

    initEvents(); // setup event hashmap

    // set default commands, if sub-system exists
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(new FieldCentricDrive(drivetrain));

      autoBuilder = new SwerveAutoBuilder(
          drivetrain::getPose, // Pose2d supplier, gyro for our facing
          drivetrain::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
          drivetrain.getKinematics(), // SwerveDriveKinematics
          new PIDConstants(4.0, 0.0, 0.0), // PID for create the X and Y
          new PIDConstants(2.0, 0.0, 0.0), // PID correct for rotation error
          drivetrain::drive, // Swerve Module states consumer, used to drive the drivetrain
          RobotContainer.RC().eventMap, // events that may be in the path
          true, // correct path for mirrored depending on alliance color.
          drivetrain);
    }

    // Edit the binding confiuration for testing
    configureBindings(Bindings.Competition);
  }

  private void configureBindings(Bindings bindings) {
    // add bindings based on current user mode
    switch (bindings) {
      case arm_test:
        dc.Driver().a().whileTrue(new MoveArmsTest(20.0, 25.0));
        dc.Driver().b().whileTrue(new ArmVelocityTest(2.0, 3.0, 1.0));
        dc.Driver().povUp().whileTrue(new ArmMoveAtSpeed(10.0));
        dc.Driver().povDown().whileTrue(new ArmMoveAtSpeed(-5.0));
        break;
      case balance_test:
        if (drivetrain == null) break;
        dc.Driver().rightBumper().whileTrue(new ChargeStationBalanceChad(false));
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
        if (drivetrain==null) break;
        // everything subject to change
        dc.Driver().x().whileTrue(new ChargeStationBalance());
        dc.Driver().y().whileTrue(new InstantCommand(() -> {
          // calibrate robot gryo to to field 0 degrees
          drivetrain.resetAnglePose(new Rotation2d(0));
        }));

        /******************************************************
         * WIP - Commands are needed, names will change, confirm with Drive team
         * dc.Driver().leftTrigger().whileTrue(new RobotOrFieldCentric());
         * dc.Driver().rightTrigger().whileTrue(new ActivatePlacer());
         * 
         * dc.Operator().leftTrigger().whileTrue(new leftColumn());
         * dc.Operator().rightTrigger().whileTrue(new rightColumn());
         * dc.Operator().leftBumper().whileTrue(new toggleIntake());
         * dc.Operator().rightBumper().whileTrue(new operatorPlaceConfirm());
         * dc.Operator().a().whileTrue(new activateIntake());
         * dc.Operator().b().whileTrue(new intakeOrOrientatorRunBack());
         * dc.Operator().x().whileTrue(new bottomRow());
         * dc.Operator().y().whileTrue(new topRow());
         ********************************************************/
        break;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new autoCommand().andThen(new ChargeStationBalanceChad());
  }

  /**
   * Add Event terms to the hashmap so they may be used in the pathing sequence
   */
  public void initEvents() {
    eventMap = new HashMap<>();
    if (drivetrain == null)
      return; // guard for bot-on-board

    eventMap.put("start",
        new SequentialCommandGroup(new PrintCommand("***Path Start"), new InstantCommand(drivetrain::printPose)));
    eventMap.put("middle",
        new SequentialCommandGroup(new PrintCommand("***Path Middle"), new InstantCommand(drivetrain::printPose)));
    eventMap.put("end", new SequentialCommandGroup(new PrintCommand("***Path End"),
        new InstantCommand(drivetrain::printPose), new ChargeStationBalance(true)));
    eventMap.put("score", new SequentialCommandGroup(new PrintCommand("***Path score"),
        new InstantCommand(drivetrain::printPose)));
  }
}