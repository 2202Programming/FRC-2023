//TODO Figure out how to set needToOpenClaw & needToCloseClaw, or how it will be determined (Probably calling a method)

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class PlaceGamePiece extends CommandBase {
  final Claw claw;
  double angleNeeded;
  boolean needToOpenClaw;
  boolean needToCloseClaw;
  /** Creates a new PlaceGamePiece. */
  public PlaceGamePiece() {
    //Don't know class name, claw for now, for the claw angle to place
    angleNeeded = claw.getDesiredAngle();
    needToOpenClaw = false;
    needToCloseClaw = false;
  }

  public void operateClaw(){
    //Change angle of claw to what's needed
    //TODO change currAngle/adjustAngle to actual variable names when known
    if(angleNeeded != claw.currAngle){
      claw.currAngle = angleNeeded;
    }
    //If the claw should be opened open the claw
    if(needToOpenClaw){
      claw.openClaw;
      needToOpenClaw = false;
    }
    //If the claw should be closed close the claw
    if(needToCloseClaw){
      claw.closeClaw;
      needToCloseClaw = false;
    }
    else{
      isFinished();
    }
  }  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  //If interrupted, open the claw
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      claw.openClaw;
    }
  }

  //Assuming default is open, if claw doesn't need to be closed, close the claw
  @Override
  public boolean isFinished() {
    if(claw.position = open && !needToCloseClaw){
      return true;
    }
    return false;
  }
}
