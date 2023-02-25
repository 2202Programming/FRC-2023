package frc.robot.commands.Intake.Washer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class IntakeForward extends CommandBase {

    Intake intake;

    public IntakeForward() {
        intake = RobotContainer.RC().intake;
    }
    
    @Override
    public void initialize() {
        intake.intakeOn();
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
