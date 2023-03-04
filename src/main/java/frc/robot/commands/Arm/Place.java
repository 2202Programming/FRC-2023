// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.commands.Arm.ArmMoveTo;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.flipper;

public class Place extends CommandBase {
  //TODO swich to Enums, dont worry Mr. L
  enum isCube{
    cube,
    cone
  } 
  enum isLeft{
    left,
    right,
    center
  }
  enum isMiddle{
    middle,
    top
  }
  public boolean needsFlip;
  public double armExtend;
  private flipper flipper;
  /** Creates a new Place. */
  public Place() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO Drive TO starting point
    //TODO pick up peice from carwash + move elbow to know pos
    //Next move robot to placement position based on cube/isLeft
    switch(isCube) {
      case cube: 
        //oreant robot
    }
    if (isCube = cube) {
      //orent robot
    } if (isCube == false) {
      if (isLeft == true) {
        //Robot go left
      } if (isLeft == false) {
        //robot go right
      }
    }
    //Move arns/ elbow out to position, pull from constants
    //Flip flipper if needsFlip is true
    //Move wrist to 180 - theta, taken care of by armMoveTo
    //parallel command

    if (isMiddle == true) {
      armExtend = Double.NaN; //TODO pull from constants
    } else {
      armExtend = Double.NaN; //TODO pull from constants
    }  
  ArmMoveTo(armExtend,Double.NaN);
  flipper.invert();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  //after that all finishes, stow the arm wrist and elbow, pull from constants. 
  //TODO put into command scheduler
  ArmMoveTo(null,null);
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
