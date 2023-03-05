// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.VerticalScoringLane;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.swerve.RobotCentricDrive;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.ColorSensors.GamePiece;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

public class Place extends CommandBase {

  public ColorSensors sensors;
  private HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
  /** Creates a new Place. */
  private goToScoringPosition position;
  private HorizontalScoringLane horizontalRequest;
  private VerticalScoringLane verticalRequest;
  public Place(ColorSensors sensors,HorizontalScoringLane horizontalRequest, VerticalScoringLane verticalRequest) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.horizontalRequest = horizontalRequest;
    this.verticalRequest = verticalRequest;
    this.sensors = sensors;

    until(dc.Driver().leftStick().or(dc.Driver().rightStick()));
  }
  private void move() {
    position = new goToScoringPosition(new PathConstraints(2,3), horizontalRequest); 
    position.schedule();
  }
  private void Cone(boolean needsFlip) {
    move();
      //drive train move Left 14.876 rotations Right
    //TODO depending on how arm clears, we may want to stall this until the above is finished
    switch (verticalRequest) {
      case Top: 
        //arm move out to desired points (Top) JASON!!
        break;
      case Middle:
        //arm move out to desired points (Middle) JASON!!
        break;
      default:
        break;
    }
  }
  private void Cube() {
    move();    
    switch (verticalRequest){
      case Top: 
        //arm move out to desired points (Top Center) JASON!!
        break;
      case Middle:
        //arm move out to desired points (Middle Center) JASON!!
        break;
      default:
        break;
    }
  }
  private void Bottom() {
    move();
    //TODO place Bottom. Flip 180?
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Next move robot to placement position based on cube/isLeft
    //placeholder: Check which color/orentation of peice
    //GamePiece piece = sensors.getCurrentGamePiece();
    if (verticalRequest == VerticalScoringLane.Bottom) {
      Bottom();
    } /*else {
      switch (piece) {
        case Cube:
          Cube();
          break;
        case ConeFacingBack:
          Cone(false); //TODO: CHECK assumption
          break;
        case ConeFacingFront:
          Cone(true); //TODO: Check assumption
          break;
        case None:

          break;
      }
    }*/

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
