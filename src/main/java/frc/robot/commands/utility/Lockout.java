package frc.robot.commands.utility;

import edu.wpi.first.wpilibj2.command.Command;

public interface Lockout extends Command {

    /** Creates a new Lockout. */
    public default Command WithLockout(double lockout_period) {
        return new LockoutCmd(this, lockout_period);
    }

}
