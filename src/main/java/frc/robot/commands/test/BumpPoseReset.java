package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public class BumpPoseReset extends CommandBase {
    // subsystems
    final Sensors_Subsystem sensors = RobotContainer.RC().sensors;
    final SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;

    // state vars
    Pose2d poseResettingTo;
    boolean goingOverBump = false;
    int framesStable;

    // constants
    final int SUFFICIENT_FRAMES_STABLE = 5; // TODO move to constants if 5 is ok
    final double MIN_PITCH_RATE = 1.0; // [deg/s] (? probably) TODO move to constants if 0.1 is ok

    /**
     * Constructs the BumpPoseReset command, which resets the pose to a given
     * position when the bot goes over the bump.
     * 
     * @param poseResettingTo the pose to reset TO.
     */
    public BumpPoseReset(Pose2d poseResettingTo) {
        this.poseResettingTo = poseResettingTo;
    }

    @Override
    public void initialize() {
        // do nothing
    }

    @Override
    public void execute() {
        if (!goingOverBump) {
            if (sensors.getPitchRate() > MIN_PITCH_RATE) goingOverBump = true;
        } else {
            framesStable += (sensors.getPitchRate() < MIN_PITCH_RATE) ? 1 : -(framesStable);
        }
    }

    @Override
    public boolean isFinished() {
        return framesStable >= 5;
    }

    @Override
    public void end(boolean interrupted) {
        sdt.setPose(poseResettingTo);
    }
}
