// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.commands.test.GenericAlignElement.GenericAlignEelementFactory;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.PowerOnPos;
import frc.robot.commands.JoystickRumbleEndless;
import frc.robot.commands.PickFromShelf;
import frc.robot.commands.takeConeFromShelf;
import frc.robot.commands.Arm.ArmMoveAtSpeed;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Arm.MoveCollectiveArm.CollectiveMode;
import frc.robot.commands.Automation.CenterTapeSkew;
import frc.robot.commands.Automation.CenterTapeYaw;
import frc.robot.commands.Intake.Washer.DeployIntake;
import frc.robot.commands.Intake.Washer.IntakeReverse;
import frc.robot.commands.Intake.Washer.intakeCompetitionToggle;
import frc.robot.commands.Intake.Washer.outtakeCompetitionToggle;
import frc.robot.commands.auto.autoSTL;
import frc.robot.commands.auto.autoTest;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.auto.goToPickupPosition;
import frc.robot.commands.swerve.AllianceAwareGyroReset;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.FieldCentricDrive;
import frc.robot.commands.swerve.RobotCentricDrive;
import frc.robot.commands.test.ArmMoveAtSpeed_L_R_test;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.RobotSpecs;
import frc.robot.util.RobotSpecs.RobotNames;
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
    configureBindings(Bindings.vision_test);

    // Quiet some of the noise
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings(Bindings bindings) {
    // add bindings based on current user mode
    switch (bindings) {
      case arm_test:
        // turn on some networking watch/updates for debugging
        armSS.getWatcher();
        claw.getWatcher();
        elbow.getWatcher();
        //claw.setElbowDoubleSupplier(elbow::getPosition);
        VelocityControlled wrist = claw.getWrist();

        // Setup some alignment tooling for the arm's components
        // MAKE SURE THE BUTTONS DON"T COLLIDE WITH OTHER COMMANDS
       // claw.setWristAngle(PowerOnPos.wrist);
        
        GenericAlignEelementFactory(armSS, 1.0, dc.Driver().a(), dc.Driver().povUp(), dc.Driver().povDown());
        GenericAlignEelementFactory(elbow, 2.0, dc.Driver().a(), dc.Driver().povRight(), dc.Driver().povLeft());
        GenericAlignEelementFactory(wrist, 5.0, dc.Driver().b(), dc.Driver().povRight(), dc.Driver().povLeft());

        // We need a way to put the arm back to Power-On
        dc.Driver().y().onTrue(new MoveCollectiveArm(CollectiveMode.power_on));

        dc.Driver().leftBumper().and(dc.Driver().a()).onTrue(new MoveCollectiveArm(CollectiveMode.placeConeMidFS));
        dc.Driver().leftBumper().and(dc.Driver().b()).onTrue(new MoveCollectiveArm(CollectiveMode.placeConeHighFS));
        dc.Driver().leftBumper().and(dc.Driver().y()).onTrue(new MoveCollectiveArm(CollectiveMode.placeCubeHighFS));
        dc.Driver().leftBumper().and(dc.Driver().x()).onTrue(new MoveCollectiveArm(CollectiveMode.placeCubeMidFS));
        dc.Driver().leftBumper().and(dc.Driver().leftTrigger()).onTrue(new MoveCollectiveArm(CollectiveMode.pickupShelfFS));
        dc.Driver().leftBumper().and(dc.Driver().rightTrigger()).onTrue(new MoveCollectiveArm(CollectiveMode.travelFS));
        // dc.Operator().rightTrigger().onTrue(new InstantCommand(() -> {
        //   claw.open();
        // }));
        // dc.Operator().rightBumper().onTrue(new InstantCommand(() -> {
        //   claw.close();
        // }));
      

        //USE A and LR POV to align the arm to a NEW ZERO (operator :=port 1)
        dc.Operator().a().whileTrue(new ArmMoveAtSpeed_L_R_test(2.0, 1).WithLockout(10.0));
        dc.Operator().b().whileTrue(new ArmMoveAtSpeed_L_R_test(-1.0, 1).WithLockout(10.0));
        dc.Operator().povUp().whileTrue(new ArmMoveAtSpeed(5.0, false));
        dc.Operator().povDown().whileTrue(new ArmMoveAtSpeed(-2.0, false));
        dc.Operator().x().whileTrue(new intakeCompetitionToggle());
        dc.Operator().y
        
        ().whileTrue(new outtakeCompetitionToggle());
        
        /************
         * pick what you need for testing only **********************
         * dc.Driver().rightBumper().whileTrue(new GenericZeroPos(elbow));
         * dc.Driver().a().whileTrue(new GenericPositionTest(elbow, 45.0, 90.0, 30.0));
         * dc.Driver().b().whileTrue(new GenericVelocityTest(elbow, 90.0, 1.50, 1.0));
         * 
         * dc.Driver().povUp().whileTrue(new ArmMoveAtSpeed(10.0, false));
         * dc.Driver().povDown().whileTrue(new ArmMoveAtSpeed(-5.0, false));
         * 
         * dc.Driver().povUp().whileTrue(new ArmMoveAtSpeed(10.0, false));
         * dc.Driver().povDown().whileTrue(new ArmMoveAtSpeed(-5.0, false));
         * 
         * dc.Driver().x().onTrue(new MoveCollectiveArm(CollectiveMode.travelFS));
         * dc.Driver().a().onTrue(new MoveCollectiveArm(CollectiveMode.pickupShelfFS));
         * dc.Driver().rightTrigger().onTrue(new
         * MoveCollectiveArm(CollectiveMode.reversePickupShelfFS));
         * dc.Driver().leftTrigger().onTrue(new
         * MoveCollectiveArm(CollectiveMode.testShelfTopFS));
         * dc.Driver().b().onTrue(new
         * MoveCollectiveArm(CollectiveMode.pickupTransitionFS));
         * 
         * dc.Driver().b().onTrue(new MoveCollectiveArm(CollectiveMode.midFS));
         * dc.Driver().leftBumper().onTrue(new
         * MoveCollectiveArm(CollectiveMode.placeMidFS));
         * dc.Driver().rightBumper().onTrue(new
         * MoveCollectiveArm(CollectiveMode.placeHighFS));
         * armSS.setDefaultCommand(new GenericJoystickPositionTest(armSS,
         * dc.Driver()::getLeftY, 0.0, 20.0, 5.0));
         *******************************************/
        break;
      case balance_test:
        if (drivetrain == null)
          break;
        dc.Driver().rightBumper().whileTrue(new ChargeStationBalance(false));
        break;

      case simulation:
        break;

      case pickup_test:
        dc.Driver().povLeft().onTrue(new goToPickupPosition(new PathConstraints(2, 3), goToPickupPosition.MoveDirection.Left));
        dc.Driver().povRight().onTrue(new goToPickupPosition(new PathConstraints(2, 3), goToPickupPosition.MoveDirection.Right));
        dc.Driver().x().onTrue(new takeConeFromShelf()); 
        dc.Driver().b().onTrue(new takeConeFromShelf()); 
        dc.Driver().leftBumper().onTrue(new PickFromShelf(goToPickupPosition.MoveDirection.Left)); 
        dc.Driver().rightBumper().onTrue(new PickFromShelf(goToPickupPosition.MoveDirection.Right));
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
        dc.Driver().y().onTrue(new AllianceAwareGyroReset(true)); // disable vision rot
        dc.Driver().leftTrigger().whileTrue(new JoystickRumbleEndless(Id.Driver));

        dc.Driver().povLeft().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringLane.Left));
        // up and down for center trio request per Alek
        dc.Driver().povUp().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringLane.Center));
        dc.Driver().povDown().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringLane.Center));
        dc.Driver().povRight().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringLane.Right));
        break;

      case Competition:
      default:
        if (drivetrain == null)
          break;
        // DRIVER
        dc.Driver().x().whileTrue(new ChargeStationBalance(false));
        dc.Driver().y().onTrue(new AllianceAwareGyroReset(false)); // gyro reset, without disabling vision
        dc.Driver().leftTrigger().whileTrue(new RobotCentricDrive(drivetrain, dc));

        // dc.Driver().povLeft().onTrue(new goToScoringPosition(new PathConstraints(2,
        // 3), HorizontalScoringLane.Left));
        // up and down for center trio request per Alek
        // dc.Driver().povUp().onTrue(new goToScoringPosition(new PathConstraints(2,3),
        // HorizontalScoringLane.Center));
        // dc.Driver().povDown().onTrue(new goToScoringPosition(new
        // PathConstraints(2,3), HorizontalScoringLane.Center));
        // dc.Driver().povRight().onTrue(new goToScoringPosition(new
        // PathConstraints(2,3), HorizontalScoringLane.Right));

        // OPERATOR
        dc.Operator().a().whileTrue(new intakeCompetitionToggle());
        dc.Operator().b().whileTrue(new outtakeCompetitionToggle());

        // testing deploying / retracting intake on bumpers
        /*
         * dc.Operator().leftBumper().onTrue(new InstantCommand(() -> {
         * intake.deploy();
         * }));
         * dc.Operator().rightBumper().onTrue(new InstantCommand(() -> {
         * intake.retract();
         * }));
         */
        // testing on pov
        // dc.Operator().povLeft().whileTrue(new IntakeForward());
        // dc.Operator().povRight().whileTrue(new IntakeReverse());
        // dc.Operator().povUp().whileTrue(new CarwashForward());
        // dc.Operator().povDown().whileTrue(new CarwashReverse());

        // PLACEMENT
        Trigger placeTrigger = dc.Driver().povLeft(); // save right tigger for concinseness in the next new commands
        // Top Place
        // placeTrigger.and(dc.Operator().leftBumper()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Left, VerticalScoringLane.Top));
        // placeTrigger.and(dc.Operator().rightBumper()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Right, VerticalScoringLane.Top));
        // Middle Place
        // placeTrigger.and(dc.Operator().leftTrigger()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Left, VerticalScoringLane.Middle));
        // placeTrigger.and(dc.Operator().rightTrigger()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Right, VerticalScoringLane.Middle));
        // Bottom Place
        // placeTrigger.and(dc.Operator().povDown()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Center, VerticalScoringLane.Bottom));
        // placeTrigger.and(dc.Operator().povLeft()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Left, VerticalScoringLane.Bottom));
        // placeTrigger.and(dc.Operator().povRight()).onTrue(new Place(colorSensors,
        // HorizontalScoringLane.Right, VerticalScoringLane.Bottom));

        placeTrigger.and(dc.Operator().povLeft())
            .onTrue(new goToScoringPosition(new PathConstraints(3, 4), HorizontalScoringLane.Left));
        placeTrigger.and(dc.Operator().povDown())
            .onTrue(new goToScoringPosition(new PathConstraints(3, 4), HorizontalScoringLane.Center));
        placeTrigger.and(dc.Operator().povRight())
            .onTrue(new goToScoringPosition(new PathConstraints(3, 4), HorizontalScoringLane.Right));

        // ELBOW TRIM - Button not finalized
        dc.Operator().a().and(dc.Operator().povDown()).onTrue(new InstantCommand(() -> {
          elbow.decrementTrim();
        }));
        dc.Operator().a().and(dc.Operator().povUp()).onTrue(new InstantCommand(() -> {
          elbow.incrementTrim();
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

  public void testPeriodic() {
    elbow.periodic();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoBuilder.fullAuto(PathPlanner.loadPathGroup("MKE-EdgeNoBalance", //Add stop point at the position want to change constraints
    //   new PathConstraints(4, 4), //2 orig, 3 worked for all speed @3.5
    //   new PathConstraints(2, 2), // worked @1.75/2.25 respectively
    //   new PathConstraints(4, 4),
    //   new PathConstraints(2, 2),
    //   new PathConstraints(4, 4)));

    // return autoBuilder.fullAuto(PathPlanner.loadPath("MKE-FarHailMaryNoBalance", new PathConstraints(4, 4)));
    
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
          new WaitCommand(0.75),
          new PrintCommand("***Placing Piece"),
          new InstantCommand(drivetrain::printPose)));

      eventMap.put("Slow Down",
      new SequentialCommandGroup(
        new PrintCommand("***Slow Down"),
        new InstantCommand(drivetrain::printPose)));

      eventMap.put("Speed Up",
      new SequentialCommandGroup(
        new PrintCommand("***Speed Up"),
        new InstantCommand(drivetrain::printPose)));

      eventMap.put("end",
        new SequentialCommandGroup(
        new PrintCommand("***Path End"),
        new InstantCommand(drivetrain::printPose),
        new ChargeStationBalance(true)));


    // eventMap.put("start",
    //     new SequentialCommandGroup(
    //         new PrintCommand("***Path Start"),
    //         new InstantCommand(drivetrain::printPose)));

    // eventMap.put("middle",
    //     new SequentialCommandGroup(
    //         new PrintCommand("***Path Middle"),
    //         new InstantCommand(drivetrain::printPose)));

    // 

    // eventMap.put("score", new SequentialCommandGroup(
    //     new PrintCommand("***Path score"),
    //     new InstantCommand(drivetrain::printPose)));

    // if (intake != null)
    //   eventMap.put("eject_start",
    //       new SequentialCommandGroup(
    //           new PrintCommand("***Eject Start"),
    //           new InstantCommand(drivetrain::printPose),
    //           new outtakeCompetitionToggle().withTimeout(0.75)));

    // if (intake != null)
    //   eventMap.put("eject_no_deploy",
    //       new SequentialCommandGroup(
    //           new PrintCommand("***Eject No Deploy Start"),
    //           new InstantCommand(drivetrain::printPose),
    //           new IntakeReverse().withTimeout(0.5)));

    // if (intake != null)
    //   eventMap.put("eject_piece",
    //       new SequentialCommandGroup(
    //           new PrintCommand("***Eject2"),
    //           new InstantCommand(drivetrain::printPose),
    //           new outtakeCompetitionToggle().withTimeout(2.00)
    //       // ,new WaitCommand(1.5)
    //       ));

    // eventMap.put("balance",
    //     new SequentialCommandGroup(
    //         new PrintCommand("***Balance"),
    //         new InstantCommand(drivetrain::printPose),
    //         new ChargeStationBalance(false)));

    // eventMap.put("intake_on",
    //     new SequentialCommandGroup(
    //         new PrintCommand("***Intake On"),
    //         new intakeCompetitionToggle().withTimeout(3.0)));

    // eventMap.put("intake_off",
    //     new SequentialCommandGroup(
    //         new PrintCommand("***Intake Off")));

    // eventMap.put("deploy_intake",
    //     new SequentialCommandGroup(
    //         new PrintCommand("***Deploying intake"),
    //         new DeployIntake()));
  }
}