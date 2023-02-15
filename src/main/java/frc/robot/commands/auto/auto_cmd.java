// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.swerve.ChargeStationBalance;
import frc.robot.subsystems.hid.SwitchboardController;
import frc.robot.subsystems.hid.SwitchboardController.SBButton;

public class auto_cmd extends CommandBase {
  
  SwitchboardController sbButton = new SwitchboardController(0);
  ArrayList Button = new ArrayList<>();
  ChargeStationBalance balance = new ChargeStationBalance();
  //TODO: If in diff branch get things correct Fetch item = new Fetch();
  
  /** Creates a new auto_cmd. */
  public auto_cmd() {
    if(sbButton.getSw11()){
      Button.add(balance);
    }
    else if(sbButton.getSw12()){
    // TODO: Fetch  Button.add(item);
    }
    else if(sbButton.getSw13()){
    // TODO: start_position Button.add();
    }
    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
