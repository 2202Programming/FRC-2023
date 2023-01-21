//In-Progress, 1/21/23 done for now, waiting on subsystem build-up

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmGeometry;

public class MoveOut extends CommandBase {
  /** Creates a new MoveOut. */
  final Arm arm;
  final double length;
  final double angle;
  final boolean syncArms;
  double[] lengths;
  double desiredArmLength;

//instantiate object and call it
  public  

  public MoveOut(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    //Assuming z is the final position
    double z = ArmGeometry.getZLength();
    this.arm = arm;


    else {

    }
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
      arm.setRetractedPosition;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //End once arm pos gets to desired pos
    if(arm.getPos = arm.setPos){
      return true;
    }
    return false;
  }
}
