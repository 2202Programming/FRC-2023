package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw_Substyem;

public class TrackThenMove extends SequentialCommandGroup {
    // SSs
    Claw_Substyem claw = RobotContainer.RC().claw;

    public TrackThenMove(CollectivePositions armPos, double wait) {
        addCommands(
                new InstantCommand(() -> {
                    claw.setTransitionClawTrackMode();
                }),
                new WaitCommand(wait),
                new MoveCollectiveArm(armPos));
    }
}
