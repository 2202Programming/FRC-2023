// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.VelocityControlled;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NeoWatcher extends CommandBase {
  final VelocityControlled device;
  final String name;
  NetworkTable table;
  /** Creates a new NeoWatcher. */
  public NeoWatcher(String name, VelocityControlled device) {
    this.name = name;
    this.device = device;
    table = NetworkTableInstance.getDefault().getTable(name);
    // Use addRequirements() here to declare subsystem dependencies.
    ntcreate();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ntUpdates();
  }
  NetworkTableEntry nt_currentPos;
  NetworkTableEntry nt_desiredPos;
  NetworkTableEntry nt_desiredVel;
  NetworkTableEntry nt_currentVel;
  
  public void ntcreate(){
    nt_currentPos = table.getEntry("Position");
    nt_currentVel = table.getEntry("Velocity");
    nt_desiredPos = table.getEntry("PositionCmd");
    nt_desiredVel = table.getEntry("VelocityCmd");
  }
  public void ntUpdates(){
    nt_currentPos.setDouble(device.getPosition());
    nt_currentVel.setDouble(device.getVelocity());
    nt_desiredPos.setDouble(device.getSetpoint());
    nt_desiredVel.setDouble(device.getVelocityCmd());

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
