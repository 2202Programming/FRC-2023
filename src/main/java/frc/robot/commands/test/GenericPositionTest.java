package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.utility.Lockout;
import frc.robot.util.VelocityControlled;

public class GenericPositionTest extends CommandBase implements Lockout {

    final VelocityControlled device;
    boolean setpointAtZero;
    final int WaitCount = 50;   // frames to wait until position command changes, 2 seconds
    int count = 0;

    // Test constraints
    final double minPos, maxPos;
    final double velocity_limit;

    double old_velocity_limit;

    public GenericPositionTest(VelocityControlled device, double minPos, double maxPos, double velocity_limit) {
        this.device = device;
        this.minPos = minPos;
        this.maxPos = maxPos;
        this.velocity_limit = velocity_limit;
    }

    @Override
    public void initialize() {
        old_velocity_limit = device.getMaxVel();
        device.setMaxVel(velocity_limit);
        setpointAtZero = true;
        count = 0;
    }

    @Override
    public void execute() {
        device.setSetpoint(setpointAtZero ? maxPos : minPos);
        // when we get to setpoint, wait 50 frames about 1 sec to flip
        if (device.atSetpoint() && ++count > WaitCount) {
            setpointAtZero = !setpointAtZero;
            count = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // do nothing
        device.hold(); // safety off. note: this uses different control mode than normal
        device.setMaxVel(old_velocity_limit);
    }

    @Override
    public boolean isFinished() {
        return false; // to kill disable or press controller button
    }
}
