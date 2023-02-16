//TODO Figure out how to set needToOpenClaw & needToCloseClaw, or how it will be determined (Probably calling a method)
/**The goal is relatively simple, essentially establish whether the claw needs to be open or closed and do that
 * along with changing the claw to a certain angle if needed
*/
package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Claw_Substyem;


public class PlaceGamePiece extends SequentialCommandGroup {
  
  final Claw_Substyem claw;

  /** Creates a new PlaceGamePiece. */
  public PlaceGamePiece(Claw_Substyem claw) {
 
    //Don't know class name, claw for now, for the claw angle to place
    this.addCommands(
      //Angles obv. not accurate, just put in random numbers
      new GamePieceAngle("nonbottomgrid", 50.0),
      new GamePieceAngle("bottomgrid", 30.0),
      new GamePieceAngle("grab", 25.0)
      /*

      Mr.L - these open/close will happen very fast, what is the intent?
      new OpenCloseClaw("closedclaw", 0.0),
      new OpenCloseClaw("cubepick-up", 2.5),
      new OpenCloseClaw("conepick-up", 2.0) 
      */
    );
    this.claw = claw;

  }

}
