package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw_Substyem;

public class WheelsOut extends Command {
    Claw_Substyem claw = RobotContainer.RC().claw;
    
    public WheelsOut() {
        // don't care do nothing
    }

    @Override
    public void initialize() {
        claw.open();
        claw.wheelsOut();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {return false;}

    @Override
    public void end(boolean interrupted) {
        claw.wheelsOff();
    }
}
