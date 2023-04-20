package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmLockForDrivingFS;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.EndEffector.InWheelsWithGate;
import frc.robot.commands.EndEffector.MoveWrist;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Claw_Substyem.ClawTrackMode;

public class AutoCubePickup extends SequentialCommandGroup {
    final Claw_Substyem claw = RobotContainer.RC().claw;

    public AutoCubePickup() {
        addCommands(
            new InstantCommand(() -> {
                claw.open();
                // this below is veerrryyy dangerous without knowing the setpoint, assuming it's travel mode
                claw.setTrackElbowMode(ClawTrackMode.free);
            }),
            new MoveCollectiveArm(CollectivePositions.uprightConePickup),
            new MoveWrist(CollectivePositions.uprightConePickup.pos_info.wristPos),
            new InWheelsWithGate(),
            new WaitCommand(0.25),
            new MoveCollectiveArm(CollectivePositions.uprightConeTravelHalfway),
            new ArmLockForDrivingFS()
        );
    }
    
}
