package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw_Substyem;

public class ToggleClaw extends Command {
    // SSs
    Claw_Substyem claw = RobotContainer.RC().claw;
    
    public ToggleClaw() {
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        if (claw.isOpen()) claw.close(); else claw.open();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
}