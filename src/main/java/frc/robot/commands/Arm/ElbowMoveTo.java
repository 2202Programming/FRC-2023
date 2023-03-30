//In-Progress, 1/21/23 done for now, waiting on subsystem build-up

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elbow;


public class ElbowMoveTo extends CommandBase {
  /** Creates a new MoveOut. */
  final double angle;
  final Elbow elbow;

  
  public ElbowMoveTo(double elbow_angle_deg) {
    elbow = RobotContainer.RC().elbow;
    angle = elbow_angle_deg;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elbow.setSetpoint(angle);
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
    // everything is elbow angle
  return elbow.atSetpoint();
  }
}
