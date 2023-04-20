// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.Constants.HorizontalScoringBlock;
import frc.robot.Constants.HorizontalScoringSubstation;
import frc.robot.Constants.PowerOnPos;
import frc.robot.Constants.VerticalScoringLane;
import frc.robot.commands.JoystickRumbleEndless;
import frc.robot.commands.PickFromShelf;
import frc.robot.commands.takeConeFromShelf;
import frc.robot.commands.Arm.ArmLockForDrivingBS;
import frc.robot.commands.Arm.ArmLockForDrivingFS;
import frc.robot.commands.Arm.ArmMoveAtSpeed;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.TrackThenMove;
import frc.robot.commands.Automation.AutoCubePickup;
import frc.robot.commands.Automation.AutoUprightConePickup;
import frc.robot.commands.Automation.CenterTapeSkew;
import frc.robot.commands.Automation.Pickup.Substation;
import frc.robot.commands.Automation.PlaceHybrid;
import frc.robot.commands.Automation.PlaceMidHighJR;
import frc.robot.commands.Automation.PlaceTele;
import frc.robot.commands.EndEffector.CloseClawWithGate;
import frc.robot.commands.EndEffector.InWheelsWithGate;
import frc.robot.commands.EndEffector.ToggleClaw;
import frc.robot.commands.EndEffector.WheelsIn;
import frc.robot.commands.EndEffector.WheelsOut;
import frc.robot.commands.Intake.Washer.DeployIntake;
import frc.robot.commands.Intake.Washer.IntakeReverse;
import frc.robot.commands.Intake.Washer.intakeCompetitionToggle;
import frc.robot.commands.Intake.Washer.outtakeCompetitionToggle;
import frc.robot.commands.auto.autoChamps;
import frc.robot.commands.auto.goToPickupPosition;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.swerve.AllianceAwareGyroReset;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.FieldCentricDrive;
import frc.robot.commands.swerve.PrecisionMode;
import frc.robot.commands.swerve.RobotCentricDrive;
import frc.robot.commands.swerve.RotateTo;
import frc.robot.commands.swerve.VelocityMove;
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
import frc.robot.subsystems.hid.CommandSwitchboardController;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.RobotSpecs;

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

    //Keep the wrist down at power up + 2 deg to put some pressure on it - 4/11/23 stall check
    claw.setWristAngle(PowerOnPos.wrist + 2.0);  

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
        //VelocityControlled wrist = claw.getWrist();

        // Setup some alignment tooling for the arm's components
        // MAKE SURE THE BUTTONS DON"T COLLIDE WITH OTHER COMMANDS
        // claw.setWristAngle(PowerOnPos.wrist);

        //GenericAlignEelementFactory(armSS, 1.0, driver.a(), driver.povUp(), driver.povDown());
        //GenericAlignEelementFactory(elbow, 2.0, driver.a(), driver.povRight(), driver.povLeft());
        //GenericAlignEelementFactory(wrist, 5.0, driver.b(), driver.povRight(), driver.povLeft());

        // We need a way to put the arm back to Power-On
        driver.y().onTrue(new ArmLockForDrivingBS());
        driver.x().onTrue(new ArmLockForDrivingFS());
        driver.rightBumper().onTrue(new TrackThenMove(CollectivePositions.pickupShelfFS, 0.2)
                                        .andThen(new CloseClawWithGate()));


        // USE A and LR POV to align the arm to a NEW ZERO (operator :=port 1)
        //oper.a().whileTrue(new ArmMoveAtSpeed_L_R_test(2.0, 1).WithLockout(10.0));
        oper.b().whileTrue(new ArmMoveAtSpeed_L_R_test(-0.5, 1).WithLockout(10.0));

        oper.y().onTrue(new AllianceAwareGyroReset(true)); // disable vision rot
        oper.povUp().whileTrue(new ArmMoveAtSpeed(5.0, true));
        oper.povDown().whileTrue(new ArmMoveAtSpeed(-1.0, true));
        oper.x().onTrue(new InstantCommand(()->{ limelight.enableLED();}));

        driver.leftBumper().onTrue(new PlaceTele(CollectivePositions.placeConeHighFS));

        driver.a().onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Right, HorizontalScoringSubstation.Right, VerticalScoringLane.High));
        driver.b().onTrue(new VelocityMove(0.5, 0, 1.0));
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

        // driver.povLeft().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringBlock.Left));
        // // up and down for center trio request per Alek
        // driver.povUp().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringBlock.Center));
        // driver.povDown().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringBlock.Center));
        driver.povLeft().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringBlock.Left, HorizontalScoringSubstation.Left));
        driver.povUp().onTrue(new goToScoringPosition(new PathConstraints(2, 3), HorizontalScoringBlock.Center, HorizontalScoringSubstation.Center));
        driver.povRight().onTrue(new goToScoringPosition(new PathConstraints(4, 3), HorizontalScoringBlock.Right, HorizontalScoringSubstation.Right));

        driver.leftBumper().onTrue(new InstantCommand(() -> {limelight.toggleLED();}));

        break;

      case Competition:
      default:
        if (drivetrain == null)
          break;
        // enable any network table watchers we want in competion
        armSS.getWatcher(); // TODO remove for comp
        claw.getWatcher(); // ditto
        elbow.getWatcher(); // ditto
        
        driverIndividualBindings();
        operatorIndividualBindings();
        automationPickupPlace();

        dc.SwitchBoard().sw24().onTrue(new InstantCommand(() -> {limelight.enableLED();})); // TODO REMOVE!!!
        dc.SwitchBoard().sw24().onFalse(new InstantCommand(() -> {limelight.disableLED();})); // TODO REMOVE!!!

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
    CommandSwitchboardController sb = dc.SwitchBoard();
    Trigger manual = sb.sw16();

    /*
     * =======================
     * SINGLE-BUTTON BIDNINGS
     * =======================
     */

    // xyab
    operator.x().onTrue(new ToggleClaw());

    operator.y().onTrue(new AutoUprightConePickup());

    operator.a().whileTrue(new intakeCompetitionToggle());

    operator.b().whileTrue(new outtakeCompetitionToggle());

    // shoulder buttons / triggers
    operator.rightTrigger().onTrue(new AutoCubePickup());

    
    // dpad
    operator.povLeft().onTrue(new ArmLockForDrivingFS());

    operator.povDown().and(operator.leftTrigger().negate())
      .onTrue(
        new PrecisionMode(true)
        .andThen(new TrackThenMove(CollectivePositions.pickupShelfFS, 0.2))
        .andThen(new CloseClawWithGate()));

      operator.povDown().and(operator.leftTrigger())
        .onTrue(
          new PrecisionMode(true)
          .andThen(new TrackThenMove(CollectivePositions.pickupShelfFS, 0.2))
          .andThen(new InWheelsWithGate()));


    /*
     * =======================
     * MANUAL MODE
     * ======================= 
     */


    // Triggers + shoulder buttons
    manual.and(operator.leftBumper())
      .whileTrue(new WheelsIn());
    manual.and(operator.rightBumper())
      .whileTrue(new WheelsOut());

    // dpad
    manual.and(operator.povUp())
          .onTrue(
            new PrecisionMode(true)
            .andThen(new TrackThenMove(CollectivePositions.placeConeHighFS, 0.2)));
    
    manual.and(operator.povRight())
          .onTrue( 
            new PrecisionMode(true)
            .andThen(new TrackThenMove(CollectivePositions.placeConeMidFS, 0.2)));

    manual.and(operator.y()).onTrue(new ArmLockForDrivingBS());

    /*
     * =======================
     * TRIM MODE
     * ======================= 
     */
    
    Trigger trim = sb.sw26();

    trim.and(operator.povDown()).onTrue(new InstantCommand(() -> {
      elbow.decrementTrim();
    }));

    trim.and(operator.povUp()).onTrue(new InstantCommand(() -> {
      elbow.incrementTrim();
    }));
  }

  private void automationPickupPlace() {
    CommandSwitchboardController sb = dc.SwitchBoard();
    CommandXboxController driver = dc.Driver();
    CommandXboxController operator = dc.Operator();

    // manual not turned on and driver hits dpad left
    Trigger automation = sb.sw16().negate().and(driver.povLeft()); // this must be pressed to trigger all the followin

    // Scoring positions
    Trigger high = operator.povUp();
    Trigger low = operator.povDown();
    Trigger mid = high.negate().and(low.negate());

    // Station
    Trigger leftStation = driver.leftBumper();
    Trigger rightStation = driver.rightBumper();
    Trigger centerStation = leftStation.negate().and(rightStation.negate());

    // Substation
    Trigger leftSubstation = operator.leftBumper();
    Trigger rightSubstation = operator.rightBumper();
    Trigger centerSubstation = leftSubstation.negate().and(rightSubstation.negate());
    

    /*
     * =======================
     * PICKUP
     * =======================
     */

    

     /*
      * =======================
      * PLACE HYBRID
      * =======================
      */

      // left station
      automation.and(low).and(leftStation).and(leftSubstation)
        .onTrue(new PlaceHybrid(HorizontalScoringBlock.Left, HorizontalScoringSubstation.Left));
      
      automation.and(low).and(leftStation).and(centerSubstation)
        .onTrue(new PlaceHybrid(HorizontalScoringBlock.Left, HorizontalScoringSubstation.Center));

      automation.and(low).and(leftStation).and(rightSubstation)
        .onTrue(new PlaceHybrid(HorizontalScoringBlock.Left, HorizontalScoringSubstation.Right));

      // center station
      automation.and(low).and(centerStation).and(leftSubstation)
        .onTrue(new PlaceHybrid(HorizontalScoringBlock.Center, HorizontalScoringSubstation.Left));
      
      automation.and(low).and(centerStation).and(centerSubstation)
        .onTrue(new PlaceHybrid(HorizontalScoringBlock.Center, HorizontalScoringSubstation.Center));

      automation.and(low).and(centerStation).and(rightSubstation)
        .onTrue(new PlaceHybrid(HorizontalScoringBlock.Center, HorizontalScoringSubstation.Right));

      // right station
      automation.and(low).and(rightStation).and(leftSubstation)
        .onTrue(new PlaceHybrid(HorizontalScoringBlock.Right, HorizontalScoringSubstation.Left));
      
      automation.and(low).and(rightStation).and(centerSubstation)
        .onTrue(new PlaceHybrid(HorizontalScoringBlock.Right, HorizontalScoringSubstation.Center));

      automation.and(low).and(rightStation).and(rightSubstation)
        .onTrue(new PlaceHybrid(HorizontalScoringBlock.Right, HorizontalScoringSubstation.Right));
      
      /*
       * =======================
       * PLACE MID
       * =======================
       */

      // left station
      automation.and(mid).and(leftStation).and(leftSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Left, HorizontalScoringSubstation.Left, VerticalScoringLane.Middle));
      
      automation.and(mid).and(leftStation).and(centerSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Left, HorizontalScoringSubstation.Center, VerticalScoringLane.Middle));

      automation.and(mid).and(leftStation).and(rightSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Left, HorizontalScoringSubstation.Right, VerticalScoringLane.Middle));

      // center station
      automation.and(mid).and(centerStation).and(leftSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Center, HorizontalScoringSubstation.Left, VerticalScoringLane.Middle));
      
      automation.and(mid).and(centerStation).and(centerSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Center, HorizontalScoringSubstation.Center, VerticalScoringLane.Middle));

      automation.and(mid).and(centerStation).and(rightSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Center, HorizontalScoringSubstation.Right, VerticalScoringLane.Middle));

      // right station
      automation.and(mid).and(rightStation).and(leftSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Right, HorizontalScoringSubstation.Left, VerticalScoringLane.Middle));
      
      automation.and(mid).and(rightStation).and(centerSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Right, HorizontalScoringSubstation.Center, VerticalScoringLane.Middle));

      automation.and(mid).and(rightStation).and(rightSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Right, HorizontalScoringSubstation.Right, VerticalScoringLane.Middle));

      /*
       * =======================
       * PLACE HIGH
       * =======================
       */

      // left station
      automation.and(high).and(leftStation).and(leftSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Left, HorizontalScoringSubstation.Left, VerticalScoringLane.High));
      
      automation.and(high).and(leftStation).and(centerSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Left, HorizontalScoringSubstation.Center, VerticalScoringLane.High));

      automation.and(high).and(leftStation).and(rightSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Left, HorizontalScoringSubstation.Right, VerticalScoringLane.High));

      // center station
      automation.and(high).and(centerStation).and(leftSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Center, HorizontalScoringSubstation.Left, VerticalScoringLane.High));
      
      automation.and(high).and(centerStation).and(centerSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Center, HorizontalScoringSubstation.Center, VerticalScoringLane.High));

      automation.and(high).and(centerStation).and(rightSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Center, HorizontalScoringSubstation.Right, VerticalScoringLane.High));

      // right station
      automation.and(high).and(rightStation).and(leftSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Right, HorizontalScoringSubstation.Left, VerticalScoringLane.High));
      
      automation.and(high).and(rightStation).and(centerSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Right, HorizontalScoringSubstation.Center, VerticalScoringLane.High));

      automation.and(high).and(rightStation).and(rightSubstation)
        .onTrue(new PlaceMidHighJR(HorizontalScoringBlock.Right, HorizontalScoringSubstation.Right, VerticalScoringLane.High));
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

    return new autoChamps();
    //return autoBuilder.fullAuto(PathPlanner.loadPath("MoveTwentyFeet", new PathConstraints(1.0, 1.0)));
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
              //was new IntakeReverse().withTimeout(0.5)));  below will finish when gate is cleared
              new IntakeReverse(true).withTimeout(0.5)));

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