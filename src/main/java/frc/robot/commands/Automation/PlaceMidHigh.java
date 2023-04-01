// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.HorizontalSubstationLane;
import frc.robot.Constants.VerticalScoringLane;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.ElbowMoveTo;
import frc.robot.commands.EndEffector.WheelsOut;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.ColorSensors;
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

  /**
   * This class is a DynamicSCG.
   * 
   * Constructs and schedules a new Place command. Encompasses sdt movement, arm
   * extension / retraction.
   * Assumes piece is already in claw (this should be done upon picking up).
   * 
   * @param horizontalRequest The station (macro-level) request
   * @param substationRequest The substation (micro-level) request
   * @param verticalRequest The height
   */
  public PlaceMidHigh(HorizontalScoringLane horizontalRequest, 
                      HorizontalSubstationLane substationRequest, 
                      VerticalScoringLane verticalRequest) {
    this.horizontalRequest = horizontalRequest;
    this.substationRequest = substationRequest;
    this.verticalRequest = verticalRequest;
  }

  // Called when the command is initially scheduled.
  @Override
  public void doFirstOnInit() {
    // 1. Move to safe location for arm extension 
    move();

    // 2. Move arm out and drop, then wait 1sec
    switch (substationRequest) {
      case Center:
        Cube();
        break;
      default:
        Cone();
        break;
    }

    // 3. Move back and retract arm to travel position
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
  }

  /**
   * Constructs pacing this based on object being a cone facing forward.
   */
  private void Cone() {
    switch (verticalRequest) {
      case High:
        this.addCommands(new PlaceTele(CollectivePositions.placeConeHighFS));
        break;
      case Middle:
        this.addCommands(new PlaceTele(CollectivePositions.placeConeMidFS));
        break;
      default:
        break;
    }

    this.addCommands(new InstantCommand(() -> {
      claw.open();
    }).andThen(new WaitCommand(TIME_DROP)));
  }

  /**
   * Constructs placing this based on object being a cube.
   * Coming out of travel position.
   */
  private void Cube() {
    switch (verticalRequest) {
      case High:
        this.addCommands(new PlaceTele(CollectivePositions.placeConeHighFS));
        break;
      case Middle:
        this.addCommands(new PlaceTele(CollectivePositions.placeConeMidFS));
        break;
      default:
        break;
    }

    this.addCommands(new WheelsOut().withTimeout(TIME_DROP));
  }

  /**
   * Retracts piece
   */
  private void Retract() {
    this.addCommands(
      new ElbowMoveTo(145.0), //return to high position to avoid low post
      new DisengageTelePlace(new PathConstraints(0.5,0.1), 0.2) //move slowly back while retracting arm
      );
  }
}