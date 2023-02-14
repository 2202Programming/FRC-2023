package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSS;

public class MoveArmsTest extends CommandBase {
    
    ArmSS arm = RobotContainer.RC().armSS;
    boolean setpointAtZero;
    double count = 0;

    public MoveArmsTest() {
        // do nothing
    }

    @Override
    public void initialize() {
        arm.setPositions(0);
        setpointAtZero = true;
    }

    @Override
    public void execute() {
        if (count <= 5) { count++; return; }

        if (arm.armsAtPosition()) {
            arm.setPositions(setpointAtZero ? 10.0 : 0.0); // TODO: is 10.0 reasonable?
            setpointAtZero ^= true;
        }

    }

    @Override
    public void end(boolean interrupted) {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return false; // to kill disable or press controller button
    }
}
