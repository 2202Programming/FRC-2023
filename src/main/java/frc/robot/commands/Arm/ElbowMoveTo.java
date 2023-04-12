//In-Progress, 1/21/23 done for now, waiting on subsystem build-up

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elbow;


public class ElbowMoveTo extends CommandBase {
  /** Creates a new MoveOut. */
  final double angle;
  final Elbow elbow;
  final double maxVel;
  double old_maxVel;

  public ElbowMoveTo(double elbow_angle_deg, double maxVel)
  {
    elbow = RobotContainer.RC().elbow;
    angle = elbow_angle_deg;
    this.maxVel = (maxVel < 0.0) ? elbow.getMaxVel() : maxVel;

  } 
  public ElbowMoveTo(double elbow_angle_deg) {
   this(elbow_angle_deg, -1.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    old_maxVel = elbow.getMaxVel();
    elbow.setMaxVel(maxVel);
    elbow.setSetpoint(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elbow.setMaxVel(old_maxVel);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return elbow.atSetpoint();
  }
}
