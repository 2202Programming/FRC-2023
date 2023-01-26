
//TODO Figure out how to set needToOpenClaw & needToCloseClaw, or how it will be determined (Probably calling a method)
/**The goal is relatively simple, essentially establish whether the claw needs to be open or closed and do that
 * along with changing the claw to a certain angle if needed
*/
package frc.robot.commands.EndAffector;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndAffector.GamePieceAngle;
import frc.robot.subsystems.Claw;



public class PlaceGamePiece extends SequentialCommandGroup {
 


  private Claw claw;


/** Creates a new PlaceGamePiece. */
  public PlaceGamePiece(Claw claw) {
    super();
    //Don't know class name, claw for now, for the claw angle to place
    this.addCommands(
      //Angles obv. not accurate, just put in random numbers
      new GamePieceAngle(claw, "nonbottomgrid", 50.0),
      new GamePieceAngle(claw, "bottomgrid", 30.0),
      new GamePieceAngle(claw, "grab", 25.0),
      new OpenCloseClaw(claw, "closedclaw", 0.0),
      new OpenCloseClaw(claw, "cubepick-up", 2.5),
      new OpenCloseClaw(claw, "conepick-up", 2.0)
    );
    this.claw = claw;


  }


  public void operateClaw(){
    //Change angle of claw to what's needed
    //TODO change currAngle/adjustAngle to actual variable names when known


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


  }


  //Assuming default is open, if claw doesn't need to be closed, close the claw
  @Override
  public boolean isFinished() {
 
    return super.isFinished();
  }
}




