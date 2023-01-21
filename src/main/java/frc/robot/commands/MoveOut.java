//In-Progress, 1/21/23 done for now, waiting on subsystem build-up

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmGeometry;
import frc.robot.subsystems.Arm;

/*
 * comments form Mr.L
 * 
 * Dont think in terms of MoveOut, MoveIn, one command should handle both.
 * 
 * Think in terms of moveTo.  Arm is at X,Y,Z and we want it at X2, Y2,Z2.
 * 
 * It's hard to start commands without a sub-sys API, focus on that.
 * 
 * I've added a few hints and cleanups/.
 * 
 */



 /*
  TODO Add description of what/how this command should work,whats your intent?
  
  */

public class MoveOut extends CommandBase {
  /** Creates a new MoveOut. */
  final Arm arm;
  final double length;
  final double angle;
  final boolean syncArms;
  double[] lengths;
  double desiredArmLength;

  
    //State Vars
  
  //instantiate object and call it

  /**
   * XYZ are blah blah
   * 
   * @param arm      required sub-system
   * @param cmdX     target positions [tbd units]
   * @param cmdY
   * @param cmdZ
   */
  public MoveOut(Arm arm, double cmdX, double cmdY, double cmdZ) {

    // Use addRequirements() here to declare subsystem dependencies.
    //Assuming z is the final position
    double z = ArmGeometry.geoZ;   //example getting geometry from constanst
    this.arm = arm;      // you can use RobotContainer.RC().arm to avoid arm in signature

    addRequirements(arm);
  }



  // Setting Z to desired value
  public void setArmPos(double z){
    arm.setPos(z);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      //Set arm to retracted position, guessing variable name
      //TODO put setRetractedPosition to variable in subsystems

      //MrL - unlike would want to set another position, more likely stop motors or 
      // just keep running so control loops hold the position.
      arm.setRetractedPosition;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //End once arm pos gets to desired pos
    //MrL - 
    if(arm.getPos = arm.setPos){
      return true;
    }
    return false;
  }
}
