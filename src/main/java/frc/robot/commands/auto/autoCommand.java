// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.Washer.outtakeCompetitionToggle;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.subsystems.hid.SwitchboardController.SBButton;

public class autoCommand extends CommandBase {

  SwerveDrivetrain drivetrain;
  HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
  double maxVelocity = 3.0;
  double maxAcceleration = 3.0;

  public autoCommand() {
    this.drivetrain = RobotContainer.RC().drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<PathPlannerTrajectory> pathGroup;
    List<PathPlannerTrajectory> finalPathGroup = new ArrayList<PathPlannerTrajectory>();

    // Starting pos 1 SW21
    if (dc.readSideboard(SBButton.Sw21)) {
      System.out.println("***Running autopath1");
      pathGroup = PathPlanner.loadPathGroup("autopath1", new PathConstraints(maxVelocity, maxAcceleration)); // 5,3
                                                                                                             // tested
                                                                                                             // and ok
    }
    // Starting pos 2 SW22
    else if (dc.readSideboard(SBButton.Sw22)) {
      System.out.println("***Running autopath2");
      pathGroup = PathPlanner.loadPathGroup("autopath2", new PathConstraints(maxVelocity, maxAcceleration)); // 5,3
                                                                                                             // tested
                                                                                                             // and ok
    }

    else {
      System.out.println("?????????? Running no path, POTATO AUTO ???????????");
      return;
    }

    // run first part of path (exit community zone, come back to scoring position)
    // RobotContainer.RC().autoBuilder.fullAuto(pathGroup.get(0)).schedule();
    finalPathGroup.add(pathGroup.get(0));

    // if SW12 on, continue on to balance
    if (dc.readSideboard(SBButton.Sw12)) {
      System.out.println("***Running second part of path to balance");
      // RobotContainer.RC().autoBuilder.fullAuto(pathGroup.get(1)).schedule();
      finalPathGroup.add(pathGroup.get(1));
    }
    SequentialCommandGroup tempCommand = new SequentialCommandGroup(
      RobotContainer.RC().autoBuilder.fullAuto(pathGroup.get(0)),
      new outtakeCompetitionToggle().withTimeout(2));
      
      if (dc.readSideboard(SBButton.Sw12)){
        tempCommand.addCommands(RobotContainer.RC().autoBuilder.fullAuto(pathGroup.get(1)));
      }

    tempCommand.schedule();
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