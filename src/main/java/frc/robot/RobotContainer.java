// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.commands.test.GenericAlignElement.GenericAlignEelementFactory;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.PowerOnPos;
import frc.robot.commands.JoystickRumbleEndless;
import frc.robot.commands.PickFromShelf;
import frc.robot.commands.takeConeFromShelf;
import frc.robot.commands.Arm.ArmLockForDriving;
import frc.robot.commands.Arm.ArmMoveAtSpeed;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Automation.CenterTapeSkew;
import frc.robot.commands.Automation.Pickup;
import frc.robot.commands.Automation.Pickup.Substation;
import frc.robot.commands.EndEffector.ToggleClaw;
import frc.robot.commands.EndEffector.WheelsIn;
import frc.robot.commands.EndEffector.WheelsOut;
import frc.robot.commands.Intake.Washer.DeployIntake;
import frc.robot.commands.Intake.Washer.IntakeReverse;
import frc.robot.commands.Intake.Washer.intakeCompetitionToggle;
import frc.robot.commands.Intake.Washer.outtakeCompetitionToggle;
import frc.robot.commands.auto.autoTest;
import frc.robot.commands.auto.goToPickupPosition;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.swerve.AllianceAwareGyroReset;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.FieldCentricDrive;
import frc.robot.commands.swerve.RobotCentricDrive;
import frc.robot.commands.swerve.RotateTo;
import frc.robot.commands.test.ArmMoveAtSpeed_L_R_test;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.ColorSensors.GamePiece;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.RobotSpecs;
import frc.robot.util.VelocityControlled;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the
 * structure of the robot (including subsystems, commands, and trigger
 * mappings) should be declared here.
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
    simulation,
    pickup_test
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
  public final ColorSensors colorSensors;

  public HashMap<String, Command> eventMap;
  public SwerveAutoBuilder autoBuilder;

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
        photonVision = null;// new PhotonVision();
        limelight = new Limelight_Subsystem();
        sensors = new Sensors_Subsystem();
        drivetrain = new SwerveDrivetrain();
        intake = new Intake();
        armSS = new ArmSS();
        elbow = new Elbow();
        claw = new Claw_Substyem();
        colorSensors = new ColorSensors();
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
        colorSensors = null;
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
        colorSensors = new ColorSensors();
        break;

      case BotOnBoard: // fall through
      case UnknownBot: // fall through
      default:
        photonVision = null;
        limelight = null;
        sensors = null;
        drivetrain = null;
        intake = null;
        armSS = new ArmSS();
        elbow = new Elbow();
        claw = new Claw_Substyem();
        colorSensors = null;
        break;
    }

    // Allow PV to get odometry
    if (photonVision != null) {
      photonVision.setDrivetrain(drivetrain);
    }

    if (limelight != null) {
      // apriltag is pipeline 0
      limelight.setPipeline(0);
    }
    // set default commands, if sub-system exists
    if (drivetrain != null) {
      initEvents();

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
      // new PathConstraints(2, 3))).andThen(new ChargeStationBalance());
      // myauto = new autoCommand();
    }

    // Edit the binding confiuration for testing
    configureBindings(Bindings.Competition);

    //Keep the wrist down at power up + 5 deg to put some pressure on it
    claw.setWristAngle(PowerOnPos.wrist +5.0);  

    // Quiet some of the noise
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings(Bindings bindings) {
    // some short hand to simplify bindings
    var driver = dc.Driver();
    var oper = dc.Operator();
    // var sb = dc.SwitchBoard();

    // add bindings based on current user mode
    switch (bindings) {
      case arm_test:
        // turn on some networking watch/updates for debugging
        armSS.getWatcher();
        claw.getWatcher();
        elbow.getWatcher();
        // claw.setElbowDoubleSupplier(elbow::getPosition);
        VelocityControlled wrist = claw.getWrist();

        // Setup some alignment tooling for the arm's components
        // MAKE SURE THE BUTTONS DON"T COLLIDE WITH OTHER COMMANDS
        // claw.setWristAngle(PowerOnPos.wrist);

        GenericAlignEelementFactory(armSS, 1.0, driver.a(), driver.povUp(), driver.povDown());
        GenericAlignEelementFactory(elbow, 2.0, driver.a(), driver.povRight(), driver.povLeft());
        GenericAlignEelementFactory(wrist, 5.0, driver.b(), driver.povRight(), driver.povLeft());

        // We need a way to put the arm back to Power-On
        driver.y().onTrue(new MoveCollectiveArm(CollectivePositions.power_on));

        driver.leftBumper().and(driver.a()).onTrue(new MoveCollectiveArm(CollectivePositions.placeConeMidFS));
        driver.leftBumper().and(driver.b()).onTrue(new MoveCollectiveArm(CollectivePositions.placeConeHighFS));
        driver.leftBumper().and(driver.y()).onTrue(new MoveCollectiveArm(CollectivePositions.placeCubeHighFS));
        driver.leftBumper().and(driver.x()).onTrue(new MoveCollectiveArm(CollectivePositions.placeCubeMidFS));
        driver.leftBumper().and(driver.leftTrigger())
            .onTrue(new MoveCollectiveArm(CollectivePositions.pickupShelfFS));
        driver.leftBumper().and(driver.rightTrigger()).onTrue(new MoveCollectiveArm(CollectivePositions.travelFS));

        // USE A and LR POV to align the arm to a NEW ZERO (operator :=port 1)
        oper.a().whileTrue(new ArmMoveAtSpeed_L_R_test(2.0, 1).WithLockout(10.0));
        oper.b().whileTrue(new ArmMoveAtSpeed_L_R_test(-0.5, 1).WithLockout(10.0));
        oper.povUp().whileTrue(new ArmMoveAtSpeed(5.0, true));
        oper.povDown().whileTrue(new ArmMoveAtSpeed(-2.0, true));
        break;

      case simulation:
        break;

      case pickup_test:
        driver.povLeft()
            .onTrue(new goToPickupPosition(new PathConstraints(2, 3), Substation.Left));
        driver.povRight()
            .onTrue(new goToPickupPosition(new PathConstraints(2, 3), Substation.Right));
        driver.x().onTrue(new takeConeFromShelf());
        driver.b().onTrue(new takeConeFromShelf());
        driver.leftBumper().onTrue(new PickFromShelf(Substation.Left));
        driver.rightBumper().onTrue(new PickFromShelf(Substation.Right));
        break;

      case vision_test:
        // X button to change LL pipeline
        // driver.a().whileTrue(new CenterTapeYaw());
        driver.a().onTrue(new RotateTo(Rotation2d.fromDegrees(90)));
        driver.b().whileTrue(new CenterTapeSkew());
        driver.x().onTrue(new AllianceAwareGyroReset(false));
        driver.y().onTrue(new AllianceAwareGyroReset(true)); // disable vision rot
        driver.leftTrigger().whileTrue(new JoystickRumbleEndless(Id.Driver));

        driver.povLeft().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringLane.Left));
        // up and down for center trio request per Alek
        driver.povUp().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringLane.Center));
        driver.povDown().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringLane.Center));
        driver.povRight().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringLane.Right));
        break;

      case Competition:
      default:
        if (drivetrain == null)
          break;
        // enable any network table watchers we want in competion
        armSS.getWatcher(); // remove for comp
        claw.getWatcher(); // ditto
        elbow.getWatcher(); // ditto
        // DRIVER
        driver.x().whileTrue(new ChargeStationBalance(false));
        driver.y().onTrue(new AllianceAwareGyroReset(false)); // gyro reset, without disabling vision
        driver.leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));

        // driver.povLeft().onTrue(new goToScoringPosition(new PathConstraints(2,
        // 3), HorizontalScoringLane.Left));
        // up and down for center trio request per Alek
        // driver.povUp().onTrue(new goToScoringPosition(new PathConstraints(2,3),
        // HorizontalScoringLane.Center));
        // driver.povDown().onTrue(new goToScoringPosition(new
        // PathConstraints(2,3), HorizontalScoringLane.Center));
        // driver.povRight().onTrue(new goToScoringPosition(new
        // PathConstraints(2,3), HorizontalScoringLane.Right));

        // OPERATOR
        oper.a().whileTrue(new intakeCompetitionToggle());
        oper.b().whileTrue(new outtakeCompetitionToggle());

        oper.y().onTrue(new MoveCollectiveArm(CollectivePositions.power_on)); // no piece, backside
        oper.povUp().onTrue(
            new SequentialCommandGroup(
                new PrintCommand("^^^^^^^^^^^^^dpad up detected"),
                new ArmLockForDriving()).withTimeout(20.0));

        // PLACEMENT

        Trigger placeTrigger = driver.povLeft(); // save right tigger for concinseness in the next new commands
        // Top Place
        // placeTrigger.and(oper.leftBumper()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Left, VerticalScoringLane.Top));
        // placeTrigger.and(oper.rightBumper()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Right, VerticalScoringLane.Top));
        // Middle Place
        // placeTrigger.and(oper.leftTrigger()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Left, VerticalScoringLane.Middle));
        // placeTrigger.and(oper.rightTrigger()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Right, VerticalScoringLane.Middle));
        // Bottom Place
        // placeTrigger.and(oper.povDown()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Center, VerticalScoringLane.Bottom));
        // placeTrigger.and(oper.povLeft()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Left, VerticalScoringLane.Bottom));
        // placeTrigger.and(oper.povRight()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Right, VerticalScoringLane.Bottom));

        // MONDAY TESTING 3/20/23 TODO REMOVE BEFORE COMP

        // THIS IS FANCY COMPLEX ONE for picking up from sehlf may f up
        // driver.rightTrigger().and(oper.povUp()).onTrue(
        //     new ParallelCommandGroup(
        //         new Pickup(Substation.Left, GamePiece.ConeFacingFront)));

        oper.povDown().onTrue(new MoveCollectiveArm(CollectivePositions.placeConeMidFS));
        oper.povRight().onTrue(new MoveCollectiveArm(CollectivePositions.placeCubeMidFS));

        oper.povLeft().onTrue(
            new SequentialCommandGroup(
                new MoveCollectiveArm(CollectivePositions.travelFS), // trackFS
                new MoveCollectiveArm(CollectivePositions.travelLockFS))); // free mode to lock

        oper.leftTrigger().whileTrue(new WheelsIn());
        oper.rightTrigger().whileTrue(new WheelsOut());

        // Testing Claw TODO MONDAY NIGHT REMOVE BEFORE COMP
        oper.leftBumper().onTrue(new InstantCommand(() -> {
          claw.open();
        }));
        oper.rightBumper().onTrue(new InstantCommand(() -> {
          claw.close();
        }));

        // TODO confirm they work with drive team
        placeTrigger.and(oper.povLeft()).onTrue(
            new goToScoringPosition(new PathConstraints(3, 4), HorizontalScoringLane.Left));
        placeTrigger.and(oper.povDown()).onTrue(
            new goToScoringPosition(new PathConstraints(3, 4), HorizontalScoringLane.Center));
        placeTrigger.and(oper.povRight()).onTrue(
            new goToScoringPosition(new PathConstraints(3, 4), HorizontalScoringLane.Right));

        // ELBOW TRIM - Button not finalized TODO- FIX BUTTONS
        oper.a().and(oper.povDown()).onTrue(new InstantCommand(() -> {
          elbow.decrementTrim();
        }));
        // oper.a().and(oper.povUp()).onTrue(new InstantCommand(() -> {
        //   elbow.incrementTrim();
        // }));

        break;

    }
  }

  private void driverIndividualBindings() {
    CommandXboxController driver = dc.Driver();

    // Triggers / shoulder Buttons
    driver.leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));

    // xyab
    driver.x().whileTrue(new ChargeStationBalance(false));
    driver.y().onTrue(new AllianceAwareGyroReset(false));

    // dpad
  }

  private void operatorIndividualBindings() {
    CommandXboxController operator = dc.Operator();

    /**
     * =======================
     * SINGLE-BUTTON BIDNINGS
     * =======================
     */

    // Triggers + shoulder buttons
    operator.leftTrigger().whileTrue(new WheelsIn());
    operator.rightTrigger().whileTrue(new WheelsOut());

    // xyab
    operator.x().onTrue(new ToggleClaw());
    operator.y().onTrue(new ArmLockForDriving()); // TODO add arm return home cmd
    operator.a().whileTrue(new intakeCompetitionToggle());
    operator.b().whileTrue(new outtakeCompetitionToggle());

    // dpad TODO use any new cmds to make sure chain doesn't snapy snap
    operator.povUp().onTrue(new MoveCollectiveArm(CollectivePositions.placeConeHighFS));
    operator.povRight().onTrue(new MoveCollectiveArm(CollectivePositions.placeConeMidFS));
    operator.povDown().onTrue(new MoveCollectiveArm(CollectivePositions.pickupShelfFS));
    operator.povLeft().onTrue(new SequentialCommandGroup(new MoveCollectiveArm(CollectivePositions.travelFS),
        new MoveCollectiveArm(CollectivePositions.travelLockFS)));

    // WI only manual scoring TODO remove
    // pickup
    operator.povDown().and(operator.x())
        .onTrue(new Pickup(Substation.Left, GamePiece.Cube)); // substation doesn't matter

    operator.povDown().and(operator.leftTrigger())
        .onTrue(new Pickup(Substation.Left, GamePiece.ConeFacingFront)); // which cone doesn't matter

    // score
    operator.povUp().and(operator.x())
        .onTrue(new MoveCollectiveArm(CollectivePositions.placeCubeHighFS));

    operator.povUp().and(operator.rightTrigger())
        .onTrue(new MoveCollectiveArm(CollectivePositions.placeConeHighFS));

    operator.povRight().and(operator.x())
        .onTrue(new MoveCollectiveArm(CollectivePositions.placeCubeMidFS));

    operator.povRight().and(operator.rightTrigger())
        .onTrue(new MoveCollectiveArm(CollectivePositions.placeConeMidFS));
  }

  public void testPeriodic() {
    elbow.periodic();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoBuilder.fullAuto(PathPlanner.loadPathGroup("MKE-EdgeNoBalance",
    // //Add stop point at the position want to change constraints
    // new PathConstraints(4, 4), //2 orig, 3 worked for all speed @3.5
    // new PathConstraints(2, 2), // worked @1.75/2.25 respectively
    // new PathConstraints(4, 4),
    // new PathConstraints(2, 2),
    // new PathConstraints(4, 4)));

    // return autoBuilder.fullAuto(PathPlanner.loadPath("MKE-FarHailMaryNoBalance",
    // new PathConstraints(4, 4)));

    return new autoTest();
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

    eventMap.put("end",
        new SequentialCommandGroup(
            new PrintCommand("***Path End"),
            new InstantCommand(drivetrain::printPose),
            new ChargeStationBalance(true)));

    eventMap.put("score", new SequentialCommandGroup(
        new PrintCommand("***Path score"),
        new InstantCommand(drivetrain::printPose)));

    eventMap.put("intake_on",
        new SequentialCommandGroup(
            new PrintCommand("***Intake On"),
            new intakeCompetitionToggle().withTimeout(3.0)));

    eventMap.put("intake_off",
        new SequentialCommandGroup(
            new PrintCommand("***Intake Off")));

    eventMap.put("deploy_intake",
        new SequentialCommandGroup(
            new PrintCommand("***Deploying intake"),
            new DeployIntake()));

    if (intake != null)
      eventMap.put("eject_start",
          new SequentialCommandGroup(
              new PrintCommand("***Eject Start"),
              new InstantCommand(drivetrain::printPose),
              new outtakeCompetitionToggle().withTimeout(0.75)));

    if (intake != null)
      eventMap.put("eject_no_deploy",
          new SequentialCommandGroup(
              new PrintCommand("***Eject No Deploy Start"),
              new InstantCommand(drivetrain::printPose),
              new IntakeReverse().withTimeout(0.5)));

    if (intake != null)
      eventMap.put("eject_piece",
          new SequentialCommandGroup(
              new PrintCommand("***Eject2"),
              new InstantCommand(drivetrain::printPose),
              new outtakeCompetitionToggle().withTimeout(2.00)
          // ,new WaitCommand(1.5)
          ));

    eventMap.put("balance",
        new SequentialCommandGroup(
            new PrintCommand("***Balance"),
            new InstantCommand(drivetrain::printPose),
            new ChargeStationBalance(false)));
  }
}