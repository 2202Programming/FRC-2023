// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.commands.test.GenericAlignElement.GenericAlignEelementFactory;

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
import frc.robot.RobotContainer;
import frc.robot.Constants.PowerOnPos;
import frc.robot.commands.JoystickRumbleEndless;
import frc.robot.commands.PickFromShelf;
import frc.robot.commands.takeConeFromShelf;
import frc.robot.commands.Arm.ArmLockForDrivingBS;
import frc.robot.commands.Arm.ArmLockForDrivingFS;
import frc.robot.commands.Arm.ArmMoveAtSpeed;
import frc.robot.commands.Arm.ArmMoveTo;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.ElbowMoveTo;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Arm.TrackThenMove;
import frc.robot.commands.Automation.CenterTapeSkew;
import frc.robot.commands.Automation.MoveToFactory;
import frc.robot.commands.Automation.Pickup.Substation;
import frc.robot.commands.EndEffector.CloseClawWithGate;
import frc.robot.commands.EndEffector.InWheelsWithGate;
import frc.robot.commands.EndEffector.ToggleClaw;
import frc.robot.commands.EndEffector.WheelsIn;
import frc.robot.commands.EndEffector.WheelsOut;
import frc.robot.commands.Intake.Washer.DeployIntake;
import frc.robot.commands.Intake.Washer.IntakeReverse;
import frc.robot.commands.Intake.Washer.intakeCompetitionToggle;
import frc.robot.commands.Intake.Washer.outtakeCompetitionToggle;
import frc.robot.commands.auto.autoTest;
import frc.robot.commands.auto.goToPickupPosition;
import frc.robot.commands.swerve.AllianceAwareGyroReset;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.commands.swerve.FieldCentricDrive;
import frc.robot.commands.swerve.PrecisionMode;
import frc.robot.commands.swerve.RobotCentricDrive;
import frc.robot.commands.swerve.RotateTo;
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
import frc.robot.util.VelocityControlled;
import frc.robot.subsystems.Claw_Substyem.ClawTrackMode;

public class PlaceHighAuto extends SequentialCommandGroup {
  /** Creates a new PlaceHighAuto. */
  private Claw_Substyem claw = RobotContainer.RC().claw;
  private Elbow elbow = RobotContainer.RC().elbow;
  private ArmSS arm = RobotContainer.RC().armSS;

  public PlaceHighAuto() {
   addCommands(
        new InstantCommand(() -> {
          claw.close();
        }),
        new InstantCommand(() -> {
          claw.setTrackElbowMode(ClawTrackMode.backSide);
        }),
        new ArmMoveTo(10.0),
        new ElbowMoveTo(70.0),
        new InstantCommand(() -> {
          claw.setTrackElbowMode(ClawTrackMode.frontSide);
        }),
        new ElbowMoveTo(155.0),
        new ArmMoveTo(37.0),
        new ElbowMoveTo(135.0),
        new InstantCommand(() -> {
          claw.close();
        }),
        new MoveCollectiveArm(CollectivePositions.power_on));

    // Use addRequirements() here to declare subsystem dependencies.
      }
}
