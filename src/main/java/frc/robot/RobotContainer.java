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
      //everything subject to change
      dc.Driver().x().whileTrue(new ChargeStationBalance(false));
      dc.Driver().y().whileTrue(new CalibrateFieldCentric());
      dc.Driver().leftStick().whileTrue(new );
      dc.Driver().rightStick().whileTrue(new );
      dc.Driver().leftTrigger().whileTrue(new RobotOrFieldCentric());
      dc.Driver().rightTrigger().whileTrue(new ActivatePlacer());
      dc.Operator().leftTrigger().whileTrue(new leftColumn());
      dc.Operator().rightTrigger().whileTrue(new rightColumn());
      dc.Operator().leftBumper().whileTrue(new toggleIntake());
      dc.Operator().rightBumper().whileTrue(new operatorPlaceConfirm());
      dc.Operator().a().whileTrue(new activateIntake());
      dc.Operator().b().whileTrue(new intakeOrOrientatorRunBack());
      dc.Operator().x().whileTrue(new bottomRow());
      dc.Operator().y().whileTrue(new topRow());
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
  /**
   * Driver xbox controller button bindings
   * <ul>
   * <li>B - *nothing bound*</li>
   * <li>A - Invert Controls</li>
   * <li>Y - Autoshift Toggle</li>
   * <li>X - Robot balance toggle</li>
   * </ul>
   */
  // void setDriverButtons() {
  //   // B - Toggle drive mode
  //   if (m_robotSpecs.getSubsystemConfig().HAS_DRIVETRAIN && m_robotSpecs.getSubsystemConfig().IS_COMPETITION_BOT) {
  //     //driverControls.bind(Id.Driver, XboxButton.B).whenPressed(m_driveController::cycleDriveMode);
  //     driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand(() -> { drivetrain.resetAnglePose(Rotation2d.fromDegrees(-180)); })); //-180 reset if intake faces drivers
  //     driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).whenPressed(m_driveController::setRobotCentric);
  //     driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).whenReleased(m_driveController::setFieldCentric);   
  //     driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenPressed(m_driveController::turnOnShootingMode);
  //     driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenReleased(m_driveController::turnOffShootingMode);
  //   }
  //   if (m_robotSpecs.getSubsystemConfig().HAS_DRIVETRAIN && !m_robotSpecs.getSubsystemConfig().IS_COMPETITION_BOT) {
  //     driverControls.bind(Id.Driver, XboxButton.B).whenPressed(m_driveControllerDrivetrain::cycleDriveMode);
  //     driverControls.bind(Id.Driver, XboxButton.Y).whenPressed(new InstantCommand(() -> { drivetrain.resetAnglePose(Rotation2d.fromDegrees(-180)); })); //-180 reset if intake faces drivers
  //     driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).whenPressed(m_driveControllerDrivetrain::setRobotCentric);
  //     driverControls.bind(Id.Driver, XboxAxis.TRIGGER_LEFT).whenReleased(m_driveControllerDrivetrain::setFieldCentric);   
  //     driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenPressed(m_driveControllerDrivetrain::turnOnShootingMode);
  //     driverControls.bind(Id.Driver, XboxAxis.TRIGGER_RIGHT).whenReleased(m_driveControllerDrivetrain::turnOffShootingMode);
  //   }


  //   // RB limelight toggle
  //   if (m_robotSpecs.getSubsystemConfig().HAS_LIMELIGHT)
  //     driverControls.bind(Id.Driver, XboxButton.X).whenPressed(new InstantCommand(limelight::toggleLED));

  //   //temporary for navx/pigeon testing
  //   driverControls.bind(Id.Driver, XboxPOV.POV_UP).whenPressed(new InstantCommand(()->{ sensors.disableNavx(true); }));
  //   driverControls.bind(Id.Driver, XboxPOV.POV_DOWN).whenPressed(new InstantCommand(()->{ sensors.disableNavx(false); }));

  // }
  // void setAssistantButtons() {
  //   // LB - Intake up/down toggle
  //   // B - Intake run back
  //   // A - bottom row
  //   // RT - right column
  //   if (driverControls.isConnected(Id.SwitchBoard)) {
  //     driverControls.bind(Id.SwitchBoard, SBButton.Sw13).whenActive(new ResetPosition(Autonomous.startPose3));
  //   }
    
  //   if (m_robotSpecs.getSubsystemConfig().HAS_INTAKE) {
  //     driverControls.bind(Id.Assistant, XboxButton.LB).whenPressed(new MoveIntake(DeployMode.Toggle));
  //     //vertical intake controls - manual control of intake and side rollers,not the magazine
  //     driverControls.bind(Id.Assistant, XboxButton.A).whileHeld(new IntakeCommand((() -> 0.6), () -> 0.5, IntakeMode.LoadCargo));
  //     driverControls.bind(Id.Assistant, XboxButton.B).whileHeld(new IntakeCommand((() -> 0.35), () -> 0.5, IntakeMode.ExpellCargo));
  //   }

  //   if (m_robotSpecs.getSubsystemConfig().HAS_MAGAZINE && m_robotSpecs.getSubsystemConfig().HAS_SHOOTER) {
  //     // Positioner binds :)
  //     driverControls.bind(Id.Assistant, XboxButton.RB).whenPressed(new MovePositioner(PositionerMode.Toggle));

  //     // Magazine Commands with intake sides, and intake roller
  //     driverControls.bind(Id.Assistant, XboxButton.X).whileHeld(mag_default_cmd.getFeedCmd());
  //     driverControls.bind(Id.Assistant, XboxButton.Y).whileHeld(mag_default_cmd.getEjectCmd());

  //     driverControls.bind(Id.Assistant, XboxAxis.TRIGGER_RIGHT).whileHeld(new VelShootGatedCommand(Shooter.DefaultSettings,     mag_default_cmd));
  //     driverControls.bind(Id.Assistant, XboxPOV.POV_LEFT)      .whileHeld(new VelShootGatedCommand(Shooter.shortVelocity,       mag_default_cmd));
  //     driverControls.bind(Id.Assistant, XboxPOV.POV_UP)        .whileHeld(new VelShootGatedCommand(Shooter.shortMediumVelocity, mag_default_cmd));
  //     driverControls.bind(Id.Assistant, XboxPOV.POV_DOWN)      .whileHeld(new VelShootGatedCommand(Shooter.mediumVelocity,      mag_default_cmd));
  //     driverControls.bind(Id.Assistant, XboxPOV.POV_RIGHT)     .whileHeld(new VelShootGatedCommand(Shooter.longVelocity,        mag_default_cmd));
  //   }
  // }
}