package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmLockForDrivingFS;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.EndEffector.InWheelsWithGate;
import frc.robot.subsystems.Claw_Substyem;

public class CubeIntoClaw extends SequentialCommandGroup {
    Claw_Substyem claw = RobotContainer.RC().claw;

    public CubeIntoClaw() {
        addCommands(
            new MoveCollectiveArm(CollectivePositions.cubeToClaw),
            new InWheelsWithGate(),
            new ArmLockForDrivingFS()
        );
        
    }
}
