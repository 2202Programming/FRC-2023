// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.commands.Arm.ArmMoveTo;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.ColorSensors.GamePiece;
import frc.robot.Constants;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.VerticalScoringLane;

public class Place extends CommandBase {

  public ColorSensors sensors;
  /** Creates a new Place. */
  private HorizontalScoringLane horizontalRequest;
  private VerticalScoringLane verticalRequest;
  public Place(ColorSensors sensors,HorizontalScoringLane horizontalRequest, VerticalScoringLane verticalRequest) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.horizontalRequest = horizontalRequest;
    this.verticalRequest = verticalRequest;
    this.sensors = sensors;
  }
  
  private void Cone(boolean needsFlip) {
    switch (horizontalRequest){
      case Left:
      //drive train move Left 14.876 rotations Left JASON!!
      case Right: 
      //drive train move Left 14.876 rotations Right
    }
    //TODO depending on how arm clears, we may want to stall this until the above is finished
    switch (verticalRequest) {
      case Top: 
      //arm move out to desired points (Top) JASON!!
      case Middle:
      //arm move out to desired points (Middle) JASON!!
    }
  }
  private void Cube() {
    switch (verticalRequest){
      case Top: 
      //arm move out to desired points (Top Center) JASON!!
      case Middle:
      //arm move out to desired points (Middle Center) JASON!!
    }
  }
  private void Bottom() {
    //TODO place Bottom. Flip 180?
    switch (horizontalRequest) {
      case Left:

      case Center:

      case Right:

    }
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Next move robot to placement position based on cube/isLeft
    //placeholder: Check which color/orentation of peice
    GamePiece piece = sensors.getCurrentGamePiece();
    if (verticalRequest == VerticalScoringLane.Bottom) {
      Bottom();
    } else {
      switch (piece) {
        case Cube:
          Cube();
        case ConeFacingBack:
          Cone(false); //TODO: CHECK assumption
        case ConeFacingFront:
          Cone(true); //TODO: Check assumption
        case None:
      }
    }

    //Move arns/ elbow out to position, pull from constants
    //Flip flipper if needsFlip is true
    //Move wrist to 180 - theta, taken care of by armMoveTo
    //parallel command
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  //after that all finishes, stow the arm wrist and elbow, pull from constants. 
  //TODO put into command scheduler
  @Override
  public boolean isFinished() {
    return false;
  }
}
