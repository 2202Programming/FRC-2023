// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Placement;

import frc.robot.commands.Arm.ArmMoveTo;
import frc.robot.commands.auto.goToScoringPosition;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.ColorSensors.GamePiece;
import frc.robot.Constants;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.VerticalScoringLane;
import frc.robot.commands.EndEffector.MoveWrist;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.Washer.outtakeCompetitionToggle;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;


public class Place extends CommandBase {

  public ColorSensors sensors;
  /** Creates a new Place. */
  private HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
  private double deadzone = 0; // [percent]
  private goToScoringPosition position;
  private HorizontalScoringLane horizontalRequest;
  private VerticalScoringLane verticalRequest;
  private double armAngle;
  public Place(ColorSensors sensors,HorizontalScoringLane horizontalRequest, VerticalScoringLane verticalRequest) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.horizontalRequest = horizontalRequest;
    this.verticalRequest = verticalRequest;
    this.sensors = sensors;
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
        new ArmMoveTo(Constants.ArmGeometry.coneMiddle, Constants.ArmGeometry.elbowMiddle); //TODO set in constants
        this.armAngle = Constants.ArmGeometry.elbowMiddle;
        break;
      case Middle:
        new ArmMoveTo(Constants.ArmGeometry.coneTop, Constants.ArmGeometry.elbowTop);  //TODO set in constants  
        this.armAngle = Constants.ArmGeometry.elbowTop;
        break;    
      default:
        System.out.println("Vertical Request has returned neither true or false, something has gone very wrong, please help");
        break;
    }
  }
  private void Cube() {
    move();    
    switch (verticalRequest){
      case Top: 
        new ArmMoveTo(Constants.ArmGeometry.cubeMiddle, Constants.ArmGeometry.elbowMiddle); //TODO set in constants
        this.armAngle = Constants.ArmGeometry.elbowMiddle;
        break;
      case Middle:
        new ArmMoveTo(Constants.ArmGeometry.cubeTop, Constants.ArmGeometry.elbowTop);  //TODO set in constants  
        this.armAngle = Constants.ArmGeometry.elbowTop;
        break;
      default:
        System.out.println("Vertical Request has returned neither true or false, something has gone very wrong, please help");
        break;
    }
  }
  private void Bottom() {
    move();
    new outtakeCompetitionToggle().withTimeout(5.0).schedule();
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
    new MoveWrist(180-armAngle,3); //Change Max Vel

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
  public void end(boolean interrupted) {
    position.cancel();
  }
  // Returns true when the command should end.
  //after that all finishes, stow the arm wrist and elbow, pull from constants. 
  //TODO put into command scheduler
  @Override
  public boolean isFinished() {
    //MrL's comments: 3/13/23, this is pretty complicated, looks like a simple math.abs() would be good enough?
    boolean leftStickStill = Math.sqrt(Math.pow(dc.Driver().getLeftX(), 2) + Math.pow(dc.Driver().getLeftY(), 2)) > deadzone;
    boolean rightStickStill = Math.sqrt(Math.pow(dc.Driver().getRightX(), 2) + Math.pow(dc.Driver().getRightY(), 2)) > deadzone;
    return (leftStickStill && rightStickStill);
  }
}
