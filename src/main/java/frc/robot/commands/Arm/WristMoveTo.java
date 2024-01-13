//In-Progress, 1/21/23 done for now, waiting on subsystem build-up

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw_Substyem;

public class WristMoveTo extends Command {
  /** Creates a new MoveOut. */
  final Claw_Substyem claw;
  final double angle;

  
  public WristMoveTo(double claw_angle_deg) {
    claw = RobotContainer.RC().claw; 
    angle = claw_angle_deg;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setWristAngle(angle);
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
    return claw.wristAtSetpoint();
  }
}
