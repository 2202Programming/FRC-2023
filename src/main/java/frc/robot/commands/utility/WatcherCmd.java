// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utility;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.NetworkTableUtil;

public abstract class WatcherCmd extends Command implements NetworkTableUtil{
  /** Creates a new Watcher. */
   final NetworkTable table;
  final String name;
   
  public WatcherCmd() {
    this.name = getTableName();
    this.table = NetworkTableInstance.getDefault().getTable(name);
    ntcreate();
    this.runsWhenDisabled();
    this.schedule();
  }

  public NetworkTable getTable() {
    return table;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ntupdate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
