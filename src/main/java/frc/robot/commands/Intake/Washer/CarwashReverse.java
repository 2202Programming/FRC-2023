package frc.robot.commands.Intake.Washer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class CarwashReverse extends Command {

    Intake intake;

    public CarwashReverse() {
        intake = RobotContainer.RC().intake;
    }
    
    @Override
    public void initialize() {
        intake.carwashOnReverse();
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
        intake.carwashOff();
    }
    
}
