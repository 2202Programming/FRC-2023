package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.SwitchboardController.SBButton;

public class autoSTL extends CommandBase {
    // Subsystems
    SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
    HID_Xbox_Subsystem dc = RobotContainer.RC().dc;

    // Constraints
    double maxVel = 4.5;
    double macAccel = 4.0;

    // Path info
    String pathName = "stl";
    SequentialCommandGroup cmd;
    List<PathPlannerTrajectory> pathGroup;

    public autoSTL() {
        addRequirements(sdt, dc);
    }

    @Override
    public void initialize() {
        // get location
        if (dc.readSideboard(SBButton.Sw11)) pathName += "Edge";
        else if (dc.readSideboard(SBButton.Sw12)) pathName += "Mid";
        else if (dc.readSideboard(SBButton.Sw13)) pathName += "Far";

        // if it's hail mary
        if (dc.readSideboard(SBButton.Sw15)) pathName += "HailMary";

        // if it's no balance
        if (!dc.readSideboard(SBButton.Sw21)) pathName += "NoBalance";

        // load the path group
        pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(maxVel, macAccel));

        if (pathName.contains("Mid")) {
            if (dc.readSideboard(SBButton.Sw22)) {
                pathGroup.remove(1);
                pathGroup.remove(1); // assume index 1 is right pickup
            }
            else {
                pathGroup.remove(2); // assume index 2 is left pickup
                pathGroup.remove(2);
            }
        }

        cmd = new SequentialCommandGroup(RobotContainer.RC().autoBuilder.fullAuto(pathGroup));
        
        cmd.schedule();
    }

    @Override
    public void execute() {
        // do nothing
    }

    @Override
    public void end(boolean interrupted) {
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
