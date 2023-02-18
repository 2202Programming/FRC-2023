package frc.robot.commands.Intake.Washer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeReverse extends CommandBase {

    Intake intake;

    public IntakeReverse() {
        intake = RobotContainer.RC().intake;
    }
    
    @Override
    public void initialize() {
        intake.intakeOnReverse();
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeOff();
    }
    
}
