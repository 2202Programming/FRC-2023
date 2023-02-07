//In-Progress, 1/21/23 done for now, waiting on subsystem build-up

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSS;

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
 * feedback 2/7/23
 * 
 * Changed command name MoveArmTo
 * 
 *   You likely won't need accessors here, you're using the Arm sub-system
 *  It will have 
 * 
 */

/*
 * Moves arm and elbow to a specific postion using default velocities.
 * The real work is in the ArmSS.
 */
public class ArmMoveTo extends CommandBase {
  /** Creates a new MoveOut. */
  final ArmSS armSS;
  final double length;
  final double angle;

  
  public ArmMoveTo(double arm_length_cm, double elbow_angle_deg) {
    armSS = RobotContainer.RC().armSS;
    length = arm_length_cm;
    angle = elbow_angle_deg;
    addRequirements(armSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSS.setPositions(length, angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // nothing to do, ss servos will do their thing.

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // everything is arm extension and elbo angle
    return armSS.everythingAtPosition();
  }
}
