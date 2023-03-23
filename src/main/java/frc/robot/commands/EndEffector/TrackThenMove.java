package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.subsystems.Claw_Substyem;

public class TrackThenMove extends SequentialCommandGroup {
    // SSs
    Claw_Substyem claw = RobotContainer.RC().claw;

    public TrackThenMove(CollectivePositions armPos) {
        addCommands(
                new InstantCommand(() -> {
                    claw.setNearestClawTrackMode();
                }),

                new MoveCollectiveArm(armPos));
    }
}
