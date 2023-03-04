// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.ArmMoveAtSpeed;
import frc.robot.commands.Automation.CenterTapeSkew;
import frc.robot.commands.Automation.CenterTapeYaw;
import frc.robot.commands.Automation.CenterTapeYawSkew;
import frc.robot.commands.Intake.Washer.CarwashForward;
import frc.robot.commands.Intake.Washer.CarwashReverse;
import frc.robot.commands.Intake.Washer.IntakeForward;
import frc.robot.commands.Intake.Washer.IntakeReverse;
import frc.robot.commands.Intake.Washer.intakeCompetitionToggle;
import frc.robot.commands.Intake.Washer.outtakeCompetitionToggle;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.swerve.AllianceAwareGyroReset;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.FieldCentricDrive;
import frc.robot.commands.swerve.MoveToPoseAutobuilder;
import frc.robot.commands.swerve.RobotCentricDrive;
import frc.robot.commands.test.ArmMoveAtSpeed_L_R_test;
import frc.robot.commands.test.ArmPositionTest;
import frc.robot.commands.test.ArmVelocityTest;
import frc.robot.commands.test.LockoutExampleCmd;
import frc.robot.commands.test.MoveArmsTest;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Intake;
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
    claw_test,
    simulation
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
  public final BlinkyLights lights;

  public HashMap<String, Command> eventMap;
  public SwerveAutoBuilder autoBuilder;

  Command myauto; // fix names later

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    RobotContainer.rc = this; // for singleton accesor
    robotSpecs = new RobotSpecs(); // mechanism to pull different specs based on roborio SN
    dc = new HID_Xbox_Subsystem(0.3, 0.9, 0.05);
    lights = new BlinkyLights(); // lights are constructed for every robot, protected if they dont exist

    // Construct sub-systems based on robot Name Specs
    switch (robotSpecs.myRobotName) {
      case CompetitionBot2023:
        photonVision = new PhotonVision();
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        intake = new Intake();
        armSS = new ArmSS();
        elbow = new Elbow();
        claw = new Claw_Substyem();
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
        armSS = null;
        elbow = null;
        claw = null;
        break;
    }

    // Allow PV to get odometry
    if (photonVision != null) {
      photonVision.setDrivetrain(drivetrain);
    }
    initEvents(); // setup event hashmap

    if (limelight != null) {
      // apriltag is pipeline 0
      limelight.setPipeline(0);
    }
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
          
      // myauto = autoBuilder.fullAuto(PathPlanner.loadPath("A4 Pass Fetch Place",
      //     new PathConstraints(2, 3))).andThen(new ChargeStationBalance());
      myauto = autoBuilder.fullAuto(PathPlanner.loadPath("visiontest1",
      new PathConstraints(1, 1)));
    }

    // Edit the binding confiuration for testing
    configureBindings(Bindings.vision_test);



    // Quiet some of the noise
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings(Bindings bindings) {
    // add bindings based on current user mode
    switch (bindings) {
      case arm_test:
        dc.Driver().a().whileTrue(new MoveArmsTest(35.0, 18.0).WithLockout(20.0));
        dc.Driver().b().whileTrue(new ArmVelocityTest(2.0, 3.0, 1.0));
        dc.Driver().povUp().whileTrue(new ArmMoveAtSpeed(10.0, true));
        dc.Driver().povDown().whileTrue(new ArmMoveAtSpeed(-5.0, true));
        dc.Driver().x().whileTrue(new ArmMoveAtSpeed_L_R_test(-1.0).WithLockout(5.0));
        dc.Driver().leftBumper().whileTrue(new LockoutExampleCmd().WithLockout(5.0));
        armSS.setDefaultCommand(new ArmPositionTest());
        break;
      case balance_test:
        if (drivetrain == null)
          break;
        dc.Driver().rightBumper().whileTrue(new ChargeStationBalance(false));
        break;

      case simulation:
        break;

      case claw_test:
        dc.Driver().rightTrigger().onTrue(new InstantCommand(() -> {
          claw.open();
        }));
        dc.Driver().leftTrigger().onTrue(new InstantCommand(() -> {
          claw.close();
        }));
        break;

      case vision_test:
        // X button to change LL pipeline
        dc.Driver().a().whileTrue(new CenterTapeYaw());
        dc.Driver().b().whileTrue(new CenterTapeSkew());
        dc.Driver().x().onTrue(new AllianceAwareGyroReset(false));
        dc.Driver().y().onTrue(new AllianceAwareGyroReset(true)); //disable vision rot

        dc.Driver().povLeft().onTrue(new goToScoringPosition(new PathConstraints(2, 3), goToScoringPosition.ScoringTrio.Left));
        dc.Driver().povUp().onTrue(new goToScoringPosition(new PathConstraints(2,3), goToScoringPosition.ScoringTrio.Center));
        dc.Driver().povRight().onTrue(new goToScoringPosition(new PathConstraints(2,3), goToScoringPosition.ScoringTrio.Right));
        break;

      case Competition:
      default:
        if (drivetrain == null)
          break;
        // DRIVER
        dc.Driver().x().whileTrue(new ChargeStationBalance(false));
        dc.Driver().y().whileTrue(new InstantCommand(() -> {
          // calibrate robot gryo to to field 0 degrees
          drivetrain.resetAnglePose(new Rotation2d(0));
        }));
        dc.Driver().leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));

        // OPERATOR
        dc.Operator().a().whileTrue(new intakeCompetitionToggle());
        dc.Operator().b().whileTrue(new outtakeCompetitionToggle());

        // testing deploying / retracting intake on bumpers
        dc.Operator().leftBumper().onTrue(new InstantCommand(() -> {
          intake.deploy();
        }));
        dc.Operator().rightBumper().onTrue(new InstantCommand(() -> {
          intake.retract();
        }));
        // testing on pov
        dc.Operator().povLeft().whileTrue(new IntakeForward());
        dc.Operator().povRight().whileTrue(new IntakeReverse());
        dc.Operator().povUp().whileTrue(new CarwashForward());
        dc.Operator().povDown().whileTrue(new CarwashReverse());

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
    return myauto;
  }

  /**
   * Add Event terms to the hashmap so they may be used in the pathing sequence
   */
  public void initEvents() {
    eventMap = new HashMap<>();
    if (drivetrain == null)
      return; // guard for bot-on-board

    eventMap.put("start",
        new SequentialCommandGroup(
            new PrintCommand("***Path Start"),
            new InstantCommand(drivetrain::printPose)));

    eventMap.put("middle",
        new SequentialCommandGroup(
            new PrintCommand("***Path Middle"),
            new InstantCommand(drivetrain::printPose)));

    eventMap.put("end",
        new SequentialCommandGroup(
            new PrintCommand("***Path End"),
            new InstantCommand(drivetrain::printPose),
            new ChargeStationBalance(true)));

    eventMap.put("score", new SequentialCommandGroup(
        new PrintCommand("***Path score"),
        new InstantCommand(drivetrain::printPose)));

    if (intake != null)
    eventMap.put("eject_start",
        new SequentialCommandGroup(
            new PrintCommand("***Eject Start"),
            new InstantCommand(drivetrain::printPose),
            new InstantCommand(intake::deploy),
            new WaitCommand(0.20),
            new InstantCommand(intake::intakeOnReverse),
            new WaitCommand(0.20),
            new InstantCommand(intake::retract)));

    if (intake != null)
    eventMap.put("eject",
        new SequentialCommandGroup(
            new PrintCommand("***Eject"),
            new InstantCommand(drivetrain::printPose),
            new InstantCommand(intake::deploy),
            new InstantCommand(intake::intakeOnReverse),
            new WaitCommand(0.20),
            new InstantCommand(intake::retract)));

    eventMap.put("balance",
        new SequentialCommandGroup(
            new PrintCommand("***Balance"),
            new InstantCommand(drivetrain::printPose),
            new ChargeStationBalance(false)));

    if (intake != null)
    eventMap.put("intake_on",
        new SequentialCommandGroup(
            new PrintCommand("***Intake On"),
            new InstantCommand(() -> {
              intake.deploy();
              intake.intakeOn();
            })));

    if (intake != null)
    eventMap.put("intake_off",
        new SequentialCommandGroup(
            new PrintCommand("***Intake Off"),
            new InstantCommand(intake::intakeOff)));
  }

}