// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.HorizontalSubstationLane;
import frc.robot.Constants.VerticalScoringLane;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.EndEffector.WheelsOut;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.swerve.RotateTo;
import frc.robot.commands.swerve.VelocityMove;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.ColorSensors.GamePiece;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.DynamicSCG;

public class PlaceMidHigh extends DynamicSCG {

  // SSs
  private HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
  public ColorSensors colorSensors = RobotContainer.RC().colorSensors;
  private Claw_Substyem claw = RobotContainer.RC().claw;

  // constants
  final double DEADZONE2 = 0.025; // [percent^2] this number is squared!!
  final double SPEED_MOVE = 0.5; // [m/s] speed moving to / from target
  final double TIME_MOVE = 1.0; // [s] time to move to / from target
  final double TIME_DROP = 0.5; // [s] time to wait after claw opens / wheels spin out before going back

  // state vars
  private HorizontalScoringLane horizontalRequest;
  private HorizontalSubstationLane substationRequest;
  private VerticalScoringLane verticalRequest;
  private GamePiece piece;
  private Command dropthis;

  /**
   * This class acts as a command factory.
   * 
   * Constructs and schedules a new Place command. Encompasses sdt movement, arm
   * extension / retraction.
   * Assumes piece is already in claw (this should be done upon picking up).
   * 
   * @param horizontalRequest
   * @param verticalRequest
   */
  public PlaceMidHigh(HorizontalScoringLane horizontalRequest, 
                      HorizontalSubstationLane substationRequest, 
                      VerticalScoringLane verticalRequest, 
                      GamePiece piece) {
    this.horizontalRequest = horizontalRequest;
    this.substationRequest = substationRequest;
    this.verticalRequest = verticalRequest;
    this.piece = piece;
  }

  // Called when the command is initially scheduled.
  @Override
  public void doFirstOnInit() {
    // 1. Move to safe location for arm extension 
    move();

    // 2. Move arm out
    switch (piece) {
      case Cube:
        Cube();   //moves arm to delivery point
        dropthis = new WheelsOut().withTimeout(TIME_DROP);
        break;
      case ConeFacingBack:
        ConeBack();
        dropthis = new InstantCommand(() -> {
          claw.open();
        }).withTimeout(TIME_DROP);
        break;
      case ConeFacingFront:
        ConeFront();
        dropthis = new InstantCommand(() -> {
          claw.open();
        }).withTimeout(TIME_DROP);
        break;
      case None:
        break;
    }

    // 3. Move above target, place, moves back
    // NOTE: THIS COULD TRACK LimeLight via reflective tape, center and dist estimate
    MovePlace(); // <-- this uses odometry to move forward to "right" openloop position

    // 4. Go to travel position
    Retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinishedCondition() {
    return dc.rightStickMotionDriver();
  }

  /**
   * Adds sdt movement to target node.
   */
  private void move() {
    // 1. move to general vicinity
    this.addCommands(new goToScoringPosition(new PathConstraints(2, 3), horizontalRequest, substationRequest));

    // 2. correct for OTF path generation rotation error
    // untested below
    this.addCommands(new RotateTo(new Rotation2d((DriverStation.getAlliance().equals(Alliance.Blue)) ? 0 : 180)));
  }

  /**
   * Constructs placing this based on object being a cone facing backwards.
   */
  private void ConeBack() {
    switch (verticalRequest) {
      case Top:
        this.addCommands(new MoveCollectiveArm(CollectivePositions.placeConeHighFS)); // TODO add back track constants
        break;
      case Middle:
        this.addCommands(new MoveCollectiveArm(CollectivePositions.placeConeMidFS)); // TODO add back track constants
        break;
      default:
        break;
    }
  }

  /**
   * Constructs pacing this based on object being a cone facing forward.
   */
  private void ConeFront() {
    switch (verticalRequest) {
      case Top:
        this.addCommands(new MoveCollectiveArm(CollectivePositions.placeConeHighFS));
        break;
      case Middle:
        this.addCommands(new MoveCollectiveArm(CollectivePositions.placeConeMidFS));
        break;
      default:
        break;
    }
  }

  /**
   * Constructs placing this based on object being a cube.
   * Coming out of travel position.
   */
  private void Cube() {
    switch (verticalRequest) {
      case Top:
        this.addCommands(new MoveCollectiveArm(CollectivePositions.placeCubeHighFS));
        break;
      case Middle:
        this.addCommands(new MoveCollectiveArm(CollectivePositions.placeCubeMidFS));
        break;
      default:
        break;
    }
  }

  /**
   * Moves to / from the target with arm extended and places the piece on the
   * target
   */
  private void MovePlace() {
    this.addCommands(
      // 1. Move to       
      new VelocityMove(-SPEED_MOVE, 0.0, TIME_MOVE),
      // 2. Drop piece
        dropthis,
      // 3. Move back
      new VelocityMove(SPEED_MOVE, 0.0, TIME_MOVE));      
    }

  /**
   * Retracts piece
   */
  private void Retract() {
    this.addCommands(
      new MoveCollectiveArm(CollectivePositions.travelFS), 
      new MoveCollectiveArm(CollectivePositions.travelLockFS));
  }
}