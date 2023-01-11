// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotSpecs;
import frc.robot.Constants.DriverControls.Autonomous;
import frc.robot.Constants.DriverControls.DriverPrefs;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.commands.sensors.ResetPosition;
import frc.robot.commands.swerve.DriveControllerDrivetrain;
import frc.robot.commands.swerve.LimelightDriveCmd;
import frc.robot.subsystems.hid.CommandSwitchboardController;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.sensors.Sensors_Subsystem;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight_Subsystem;
import frc.robot.ux.Dashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final HID_Xbox_Subsystem driverControls;
  public Sensors_Subsystem sensors;
  public SwerveDrivetrain drivetrain;
  public Limelight_Subsystem limelight;

  public static String auto_path_name = "NONE";

  public DriveControllerDrivetrain m_DriveControllerDrivetrain;
  public Command drivetrainCommand;

  public RobotSpecs m_robotSpecs;
  public String rioSN;

  LimelightDriveCmd swd;

  public final Dashboard dashboard;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // ID what type of robot it is
    rioSN = System.getenv("serialnum");
    rioSN = (rioSN == null) ? "sim" : rioSN;  // catch null on simulated robot
    m_robotSpecs = Constants.keysAndBots.get(rioSN);
    System.out.println("***** Rio S/N: " + rioSN + " *****");
    System.out.println("***** Robot Type: " + m_robotSpecs.toString() + " *****");

    // Create subsystemes based on specific robot
    sensors = new Sensors_Subsystem();
    driverControls = new HID_Xbox_Subsystem(DriverPrefs.VelExpo, DriverPrefs.RotationExpo, DriverPrefs.StickDeadzone);
    // These are hardware specific
    if (m_robotSpecs.subsysConfig.HAS_DRIVETRAIN)
      drivetrain = new SwerveDrivetrain(sensors, m_robotSpecs);
    if (m_robotSpecs.subsysConfig.HAS_INTAKE)
      // TODO: not developed yet!
    if (m_robotSpecs.subsysConfig.HAS_LIMELIGHT)
      limelight = new Limelight_Subsystem();
    
      m_DriveControllerDrivetrain = new DriveControllerDrivetrain(drivetrain, driverControls, sensors);
      drivetrainCommand = m_DriveControllerDrivetrain;

    dashboard = new Dashboard(this);

    // Configure the trigger bindings
    configureDriverBindings();
    configureOperatorBindings();
    configureSwitchboardBindings();
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
  private void configureDriverBindings() {
   CommandXboxController driverController = (CommandXboxController) driverControls.deviceMap.get(Id.Driver); 
  
    driverController.b().onTrue(new InstantCommand(m_DriveControllerDrivetrain::cycleDriveMode));
    driverController.y().onTrue(new InstantCommand(() -> {drivetrain.resetAnglePose(Rotation2d.fromDegrees(-180));})); //-180 reset if intake faces drivers
    driverController.leftTrigger().onTrue(new InstantCommand(m_DriveControllerDrivetrain::setRobotCentric));
    driverController.leftTrigger().onFalse(new InstantCommand(m_DriveControllerDrivetrain::setFieldCentric));

    if (m_robotSpecs.subsysConfig.HAS_LIMELIGHT) driverController.x().onTrue(new InstantCommand(limelight::toggleLED));
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
  private void configureOperatorBindings() {
   
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
  private void configureSwitchboardBindings() {
   CommandSwitchboardController switchboard = (CommandSwitchboardController) driverControls.deviceMap.get(Id.SwitchBoard);

   switchboard.sw13().onTrue(new ResetPosition(drivetrain, Autonomous.startPose3));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return dashboard.getAutonomousCommand();
  }
}
