package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmLockForDrivingFS;
import frc.robot.commands.Arm.SafeMoveToCubePickup;
import frc.robot.commands.EndEffector.InWheelsWithGate;
import frc.robot.subsystems.Claw_Substyem;

public class CubeIntoClaw extends SequentialCommandGroup {
    Claw_Substyem claw = RobotContainer.RC().claw;

    public CubeIntoClaw() {
        addCommands(
            new SafeMoveToCubePickup(),
            new InWheelsWithGate(),
            new ArmLockForDrivingFS()
        );
        
    }
}
