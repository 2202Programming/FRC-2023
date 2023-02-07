// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Intake;
import frc.robot.commands.swerve.ChargeStationBalance;
import com.pathplanner.lib.PathConstraints;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.commands.Automation.CenterTapeSkew;
import frc.robot.commands.Automation.CenterTapeYaw;
import frc.robot.commands.Automation.CenterTapeYawSkew;
import frc.robot.commands.swerve.FieldCentricDrive;
import frc.robot.subsystems.ArmSS;
import frc.robot.commands.swerve.FollowPPTrajectory;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.RobotSpecs;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.RobotSpecs.RobotNames;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  static RobotContainer rc;
  //Sub-systems 
  public final RobotSpecs robotSpecs;
  public final Sensors_Subsystem sensors;
  public final SwerveDrivetrain drivetrain;
  public final Intake intake;
  public final HID_Xbox_Subsystem dc;           //short for driver controls
  public final ArmSS armSS;

  // Note: we may replace HID_Xbox with CommandPS4Controller or CommandJoystick if needed
 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public PhotonVision photonVision;
  public Limelight_Subsystem limelight;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotContainer.rc = this;

    m_robotSpecs = new RobotSpecs(System.getenv("serialnum"));  //mechanism to pull different specs based on roborio serial

  // these can get created on any hardware setup
  photonVision = new PhotonVision();
  limelight = new Limelight_Subsystem();
  limelight.setPipeline(1);

  if (m_robotSpecs.myRobotName != RobotNames.BotOnBoard){
    sensors = new Sensors_Subsystem();
    drivetrain = new SwerveDrivetrain();
  // these can get created on any hardware setup
  photonVision = new PhotonVision();
  limelight = new Limelight_Subsystem();
  limelight.setPipeline(1);

  if (m_robotSpecs.myRobotName != RobotNames.BotOnBoard){
    sensors = new Sensors_Subsystem();
    drivetrain = new SwerveDrivetrain();
    drivetrain.setDefaultCommand(new FieldCentricDrive(drivetrain));
  }

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

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
    // An example command will be run in autonomous
    return 
    //null;
    new FollowPPTrajectory(FollowPPTrajectory.pathFactoryTele(new PathConstraints(1, 1), 
                                                              new Pose2d(drivetrain.getPose().getX() + 1, 
                                                                          drivetrain.getPose().getY() + 1, 
                                                                          new Rotation2d(drivetrain.getPose().getRotation().getRadians() + Math.PI))), 
                                                                          true);
  }
}
