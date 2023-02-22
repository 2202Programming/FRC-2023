package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSS;

public class MoveArmsTest extends CommandBase {

    ArmSS arm = RobotContainer.RC().armSS;
    boolean setpointAtZero;
    double count = 0;

    // Test constraints
    final double distance_cm;
    final double velocity_limit;

    // save vel limit so we can restore after test
    double old_velocity_limit;

    /*
     * Assumes arm is at 0.0 and will move
     */

    public MoveArmsTest(double distance_cm) {
        this(distance_cm, 5.0);
    }

    public MoveArmsTest(double distance_cm, double velocity_limit) {
        this.distance_cm = distance_cm;
        this.velocity_limit = velocity_limit;
    }

    @Override
    public void initialize() {
        old_velocity_limit = arm.getVelocityLimit();
        arm.setVelocityLimit(velocity_limit);
        arm.setPositions(0.0);
        setpointAtZero = true;
        count = 0;
    }

    @Override
    public void execute() {
        arm.setSetpoints(setpointAtZero ? distance_cm : 0.0);
        //when we get to setpoint, wait 50 frames about 1 sec to flip
        if (arm.armsAtPosition() && ++count > 50) {
            setpointAtZero = !setpointAtZero;
            count = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // do nothing
        arm.hold(); // safety off. note: this uses different control mode than normal
        arm.setVelocityLimit(old_velocity_limit);
    }

    @Override
    public boolean isFinished() {
        return false; // to kill disable or press controller button
    }
}
