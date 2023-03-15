// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverControls.Id;

public class MatchTimer extends CommandBase {
  /** Counts down seconds left in match, does some driver notification */

  boolean buzzing = false;

  public MatchTimer() {}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int timeLeft = (int)DriverStation.getMatchTime();  //drop fraction of seconds
    switch(timeLeft){
      case(30):
        if(!buzzing){
          new JoystickRumble(Id.Operator, 1, 3).schedule();
          buzzing = true;
        }
        break;
      case(28):
        buzzing = false;
        break;
      case(10):
        if(!buzzing){
          new JoystickRumble(Id.Operator, 1, 1).schedule();
          buzzing = true;
        }
        break;
      case(8):
        buzzing = false;
        break;
      case(5):
        if(!buzzing){
          new JoystickRumble(Id.Operator, 1, 1).schedule();
          buzzing = true;
        }
        break;
      case(3):
        buzzing = false;
        break;       
      default:
        break;
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
