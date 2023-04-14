package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmLockForDrivingFS;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.EndEffector.CloseClawWithGate;

public class AutoUprightConePickup extends SequentialCommandGroup {
    public AutoUprightConePickup() {
        addCommands(
            new InstantCommand(() -> {RobotContainer.RC().claw.open();}),
            new MoveCollectiveArm(CollectivePositions.uprightConePickup),
            new CloseClawWithGate(),
            new WaitCommand(0.25),
            new MoveCollectiveArm(CollectivePositions.uprightConeTravelHalfway),
            new ArmLockForDrivingFS()
        );
    }
    
}
