// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.VerticalScoringLane;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Arm.MoveCollectiveArm.CollectiveMode;
import frc.robot.commands.Intake.Washer.outtakeCompetitionToggle;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.ColorSensors.GamePiece;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;


public class Place extends CommandBase {

  // SSs
  private HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
  public ColorSensors colorSensors = RobotContainer.RC().colorSensors;

  // deadzone
  final double DEADZONE = 0; // [percent]

  // state vars
  private HorizontalScoringLane horizontalRequest;
  private VerticalScoringLane verticalRequest;
  private GamePiece piece;

  // the cmd
  private SequentialCommandGroup cmd = new SequentialCommandGroup();

  /**
   * This class acts as a command factory.
   * 
   * Constructs and schedules a new Place command. Encompasses sdt movement, arm extension / retraction.
   * Assumes piece is already in claw (this should be done upon picking up).
   * 
   * @param horizontalRequest
   * @param verticalRequest
   */
  public Place(HorizontalScoringLane horizontalRequest, VerticalScoringLane verticalRequest) {
    this.horizontalRequest = horizontalRequest;
    this.verticalRequest = verticalRequest;

    addRequirements(dc, colorSensors);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // 1. Move to scoring position
    move();

    // 2. Move arm out
    piece = colorSensors.getCurrentGamePiece();

    switch (piece) {
      case Cube:
        Cube();
        break;
      case ConeFacingBack:
        ConeBack();
        break;
      case ConeFacingFront:
        ConeFront();
        break;
      case None:
        break;
      }

    //MrL's comments: 3/13/23, this is pretty complicated, looks like a simple math.abs() would be good enough?
    // nren 3/16/23: I mean it's just finding the magnitude of the vector of the stick in 2d space, but I think consistency is important so can't just check one axis
    cmd.until(() -> {
      boolean xStickStill = (Math.sqrt(Math.pow(dc.Driver().getLeftX(), 2) + Math.pow(dc.Driver().getLeftY(), 2)) > DEADZONE); 
      boolean yStickStill = (Math.sqrt(Math.pow(dc.Driver().getRightX(), 2) + Math.pow(dc.Driver().getRightY(), 2)) > DEADZONE);
      return (xStickStill && yStickStill);
    }).schedule();
  }

  /**
   * Adds sdt movement to target node.
   */
  private void move() {
    // 1. move to general vicinity
    cmd.addCommands(new goToScoringPosition(new PathConstraints(2,3), horizontalRequest));

    // 2. correct for OTF path generation rotation error
    // TODO check which alliance is 0/180, pull RotateTo in from other branch
    //cmd.addCommands(new RotateTo(new Rotation2d((DriverStation.getAlliance().equals(Alliance.Blue)) ? 0 : 180)));
  }

  /**
   * Adds command(s) necessary for bottom scoring.
   */
  private void Bottom() {
    cmd.addCommands(new outtakeCompetitionToggle().withTimeout(5.0));
  }

  /**
   * Adds commands necessary for mid/high scoring.
   */
  private void MidHigh(CollectiveMode armLocation) {
    cmd.addCommands(
      new MoveCollectiveArm(armLocation),
      // TODO new command: claw open until lightgate breaks, end condition close claw
      new MoveCollectiveArm(CollectiveMode.travelFS)
    );
  }

  /**
   * Constructs placing cmd based on object being a cone facing backwards.
   */
  private void ConeBack() {
    switch (verticalRequest) {
      case Top: 
        MidHigh(CollectiveMode.placeConeHighFS); //TODO cone/cube customization in enum
        break;
      case Middle:
        MidHigh(CollectiveMode.placeConeMidFS); //TODO cone/cube customization in enum
        break;    
      case Bottom:
        Bottom();
        break;
    }
  }

  /**
   * Constructs pacing cmd based on object being a cone facing forward.
   */
  private void ConeFront() {
  switch (verticalRequest) {
    case Top: 
      MidHigh(CollectiveMode.placeConeHighFS); //TODO cone/cube customization in enum
      break;
    case Middle:
      MidHigh(CollectiveMode.placeConeMidFS);  //TODO cone/cube customization in enum
      break;    
    case Bottom:
      Bottom();
      break;
  }
}

  /**
   * Constructs placing cmd based on object being a cube.
   */
  private void Cube() {  
    switch (verticalRequest) {
      case Top: 
        MidHigh(CollectiveMode.placeCubeHighFS); //TODO cone/cube customization in enum
        break;
      case Middle:
        MidHigh(CollectiveMode.placeCubeMidFS);  //TODO cone/cube customization in enum
        break;    
      case Bottom:
        Bottom();
        break;
    }
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    // do nothing
  }
  
  @Override
  public void end(boolean interrupted) {
    // do nothing
  }
  
  @Override
  public boolean isFinished() {
    // it's movement cmd path factory, should be done after init / upon first time execute is called
    return true;
  }
}