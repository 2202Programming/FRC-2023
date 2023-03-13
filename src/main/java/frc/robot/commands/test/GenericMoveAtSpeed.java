

package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.VelocityControlled;

public class GenericMoveAtSpeed extends CommandBase {

  final VelocityControlled device;
  final boolean zero_pos_when_done;
  final double speed;
  double old_max_speed;

  /** Creates a new GenericMoveAtSpeed. */
  public GenericMoveAtSpeed(VelocityControlled device, double speed, boolean zero_pos_when_done) {

    this.device = device;
    this.speed = speed;
    this.zero_pos_when_done = zero_pos_when_done;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    old_max_speed = device.getMaxVel();
    device.setMaxVel(speed);
    device.setVelocityCmd(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    device.setVelocityCmd(0.0);
    if (zero_pos_when_done) {
      device.setPosition(0.0);
      device.setSetpoint(0.0);
    }
    device.hold();
    device.setMaxVel(old_max_speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // rely on the button to stop
  }
}
