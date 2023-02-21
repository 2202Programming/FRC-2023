// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverControls.Id;

public class MatchTimer extends CommandBase {
  /** Counts down seconds left in match, does some driver notification */
  public MatchTimer() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int timeLeft = (int)DriverStation.getMatchTime();  //drop fraction of seconds
    if (timeLeft % 30 < 1) {
      System.out.println("**Match time left: " + timeLeft);
      if(timeLeft > 125) { 
        new JoystickRumble(Id.Driver, 1).schedule();
      }
      else if(timeLeft < 125 && timeLeft > 65) { 
        new JoystickRumble(Id.Driver, 1, 2).schedule();
      }
      else if(timeLeft < 65 && timeLeft > 35) { 
        new JoystickRumble(Id.Driver, 1, 3).schedule();
      }
      else if (timeLeft < 35){
        new JoystickRumble(Id.Driver, 1, 4).schedule();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
