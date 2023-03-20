package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.SwitchboardController.SBButton;

public class autoTest extends CommandBase {
    // Subsystems
    SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
    HID_Xbox_Subsystem dc = RobotContainer.RC().dc;

    // Constraints
    double maxVel = 3.0; // for mid going over charge station (STL elims) prev 4.5
    double macAccel = 3.0; // for mid going over charge station (STL elims) pre 4.0

    // Path info
    String pathName = "MKE-";
    SequentialCommandGroup cmd;
    List<PathPlannerTrajectory> pathGroup;

    public autoTest() {
        addRequirements(sdt, dc);
    }

    @Override
    public void initialize() {
        // get location
        if (dc.readSideboard(SBButton.Sw11)) pathName += "Edge";
        else if (dc.readSideboard(SBButton.Sw12)) pathName += "Mid";
        else if (dc.readSideboard(SBButton.Sw13)) pathName += "Far";
        else pathName += "Mid";   //added for sim testing where no sideboard exists, npe otherwise.

        // if it's hail mary
        if (dc.readSideboard(SBButton.Sw15)) pathName += "HailMary";

        // if it's no balance
        if (!dc.readSideboard(SBButton.Sw21)) pathName += "NoBalance";
        else pathName += "Balance";

        // load the path group
        new PrintCommand(pathName).schedule();

        if (pathName.contains("Edge")) {
            pathGroup = PathPlanner.loadPathGroup(pathName,
            new PathConstraints(4, 4), //2 orig, 3 worked for all speed @3.5
              new PathConstraints(2, 2), // worked @1.75/2.25 respectively
              new PathConstraints(4, 4),
              new PathConstraints(2, 2),
              new PathConstraints(4, 4));
        }
        else{
            pathGroup = PathPlanner.loadPathGroup(pathName, new PathConstraints(maxVel, macAccel));
        }

        // STL elims we're not that fancy
        
        // if (pathName.contains("Mid")) {
        //     if (dc.readSideboard(SBButton.Sw22)) {
        //         pathGroup.remove(1);
        //         pathGroup.remove(1); // assume index 1 is right pickup
        //     }
        //     else {
        //         pathGroup.remove(2); // assume index 2 is left pickup
        //         pathGroup.remove(2);
        //     }
        // }

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
