// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.MatchTimer;
import frc.robot.subsystems.BlinkyLights.BlinkyLightController;
import frc.robot.util.NeoServo;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package 
 * after creating this project, you must also update the build.gradle 
 * file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    BlinkyLightController.onRobotInit();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran during disabled, autonomous, teleoperated 
   * and test.
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands,
    // removing finished or interrupted commands,  and running subsystem 
    // periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  /** This function is called FIRST on every mode (tele, auto, test) init */
  public void enabledInit() {
    // 4-10-2023 DPL and nren
    // if it's stuck on carwash (or anywhere) integral term gain is going to be very high,
    // we remove the I gain so it's 0, which, yes, removes the i-term, but it is
    // mostly inconsequential since the I term is so small. If we don't do this it would
    // result in a very high integral term which is obviously bad --> this solves the
    // problem of the windeup on a high cmd vel when disabling / renabling
    //
    // NOTE: we also took out the kI term on the HW PID for the wrist, so this should do
    // nothing, but hey we might add it back
    // --END-- 4/10/2023
    ((NeoServo) RobotContainer.RC().claw.getWrist()).clearHwPID();
  }

  @Override
  public void disabledPeriodic() {
    BlinkyLightController.onDisabledPeriodic();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    enabledInit();

    if (m_robotContainer.drivetrain != null)
      m_robotContainer.drivetrain.disableVisionPose();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    BlinkyLightController.onAutomousdInit();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    enabledInit();

    // 2023 post-auto path changes for Red only so we drive in red coords
    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      Rotation2d currPose = m_robotContainer.drivetrain.getPose().getRotation();
      currPose = currPose.minus(Rotation2d.fromDegrees(180));
      m_robotContainer.drivetrain.resetAnglePose(currPose);
    }

    if (m_robotContainer.drivetrain != null)
      m_robotContainer.drivetrain.enableVisionPose();
      m_robotContainer.drivetrain.disableVisionPoseRotation();

    // This makes sure that the autonomous stops running when teleop starts running.
    // If you want the autonomous to continue until interrupted by another command,
    // remove this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    new MatchTimer().schedule(); // a match time reporter for the telop portion of the match
    BlinkyLightController.onTeleopInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    enabledInit();

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    BlinkyLightController.onTestInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    BlinkyLightController.onTestPeriodic();
    RobotContainer.RC().testPeriodic();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    if (m_robotContainer.drivetrain != null)
      m_robotContainer.drivetrain.simulationInit();

    if (m_robotContainer.photonVision != null)
      m_robotContainer.photonVision.simulationInit();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
