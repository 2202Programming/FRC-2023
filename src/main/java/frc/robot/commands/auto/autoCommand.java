// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.SwitchboardController.SBButton;

public class autoCommand extends CommandBase {

  SwerveDrivetrain drivetrain;
  HID_Xbox_Subsystem dc = RobotContainer.RC().dc;

  public autoCommand() {
    this.drivetrain = RobotContainer.RC().drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<PathPlannerTrajectory> pathGroup; 

    // Starting pos 1
    if (dc.initialSideboard(SBButton.Sw21)) pathGroup = PathPlanner.loadPathGroup("pos1", new PathConstraints(1, 3));  //5,3 tested and ok
    // Starting pos 2
    else if (dc.initialSideboard(SBButton.Sw22)) pathGroup = PathPlanner.loadPathGroup("pos2", new PathConstraints(1, 3));  //5,3 tested and ok
    // Starting pos 3
    else if (dc.initialSideboard(SBButton.Sw23)) pathGroup = PathPlanner.loadPathGroup("pos3", new PathConstraints(1, 3));  //5,3 tested and ok
    // Starting pos 4
    else if (dc.initialSideboard(SBButton.Sw24)) pathGroup = PathPlanner.loadPathGroup("pos4", new PathConstraints(1, 3));  //5,3 tested and ok
    // No auto
    else return;

    List<PathPlannerTrajectory> filteredGroup = new ArrayList<PathPlannerTrajectory>();

    // index 0: initial pathing to get out of community zone, place object currently held

    // this assumes that the getting / placing / getting out of community zone all end at the same place so everything can be neatly linked

    // SW12: Get out of community zone
    if (dc.initialSideboard(SBButton.Sw12)) filteredGroup.add(pathGroup.get(1));

    // SW13: Get / place 1st position object
    if (dc.initialSideboard(SBButton.Sw13)) filteredGroup.add(pathGroup.get(2));

    // SW14: Get / place 2nd position object
    if (dc.initialSideboard(SBButton.Sw14)) filteredGroup.add(pathGroup.get(3));

    // SW15: Get / place 3rd position object
    if (dc.initialSideboard(SBButton.Sw15)) filteredGroup.add(pathGroup.get(4));

    // SW 16: Get / place 4th position object
    if (dc.initialSideboard(SBButton.Sw16)) filteredGroup.add(pathGroup.get(5));

    // SW 26: Move from position to the other side of the field (we'll undoubtedely be stopped in the midst of it but whatever)
    if (dc.initialSideboard(SBButton.Sw26)) filteredGroup.add(pathGroup.get(6));

    // SW11: Charge station balance
    if (dc.initialSideboard(SBButton.Sw11)) filteredGroup.add(pathGroup.get(6));
    
    // Schedule full group
    RobotContainer.RC().autoBuilder.fullAuto(filteredGroup).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}