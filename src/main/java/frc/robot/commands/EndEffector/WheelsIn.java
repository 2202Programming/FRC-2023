package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw_Substyem;

public class WheelsIn extends CommandBase {
    Claw_Substyem claw = RobotContainer.RC().claw;
    
    public WheelsIn() {
        // don't care do nothing
    }

    @Override
    public void initialize() {
        claw.wheelsIn();
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
