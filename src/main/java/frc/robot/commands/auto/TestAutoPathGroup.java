package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.SwitchboardController.SBButton;

/**
 * Tests the auto path group by only running certain parts of the path, split by stop points.
 * 
 * To run the first part, enable Sw13, first and second, Sw13 and Sw14, etc. all the way to Sw16 --> Sw21 --> Sw26 for a max of 10 parts
 */
public class TestAutoPathGroup extends CommandBase {

    String pathName;

    PathConstraints constraints;

    SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
    HID_Xbox_Subsystem dc = RobotContainer.RC().dc;

    List<PathPlannerTrajectory> pathGroup;
    List<PathPlannerTrajectory> pathGroupFinal = new ArrayList<PathPlannerTrajectory>();

    List<Boolean> sbValues = new ArrayList<Boolean>();

    public TestAutoPathGroup(String pathName, PathConstraints constraints) {
        this.pathName = pathName;
        this.constraints = constraints;
    }

    @Override
    public void initialize() {
        sbValues.add(dc.readSideboard(SBButton.Sw13));
        sbValues.add(dc.readSideboard(SBButton.Sw14));
        sbValues.add(dc.readSideboard(SBButton.Sw15));
        sbValues.add(dc.readSideboard(SBButton.Sw16));
        sbValues.add(dc.readSideboard(SBButton.Sw21));
        sbValues.add(dc.readSideboard(SBButton.Sw22));
        sbValues.add(dc.readSideboard(SBButton.Sw23));
        sbValues.add(dc.readSideboard(SBButton.Sw24));
        sbValues.add(dc.readSideboard(SBButton.Sw25));
        sbValues.add(dc.readSideboard(SBButton.Sw26));

        this.pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(0, 0));

        for (int i = 0; i < this.pathGroup.size() - 1; i++) {
            if (sbValues.get(i)) pathGroupFinal.add(pathGroup.get(i));
        }

        SequentialCommandGroup cmd = new SequentialCommandGroup(RobotContainer.RC().autoBuilder.fullAuto(pathGroupFinal));

        if (dc.readSideboard(SBButton.Sw12)) cmd.addCommands(new ChargeStationBalance());

        cmd.schedule();
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public void end(boolean interrupted) {
        sdt.stop();
    }

    @Override
    public boolean isFinished() {
        // build and schedules path command, then it's done. First time this is called said cmd should already be scheduled
        return true;
    }
    
}
