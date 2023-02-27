// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSS;

public class ArmMoveAtSpeed extends CommandBase {
 
  ArmSS arm;
  boolean zero_position = false;
  double speed; //[cm/s]
  double old_max_speed;

  /** Creates a new ArmMoveAtSpeed. */
  public ArmMoveAtSpeed(double speed) {
    this(speed, false);
  }

  public ArmMoveAtSpeed(double speed, boolean zero_position) {
    this.arm = RobotContainer.RC().armSS;
    this.speed = speed;
    this.zero_position = zero_position;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    old_max_speed = arm.getMaxVel();
    arm.setMaxVel(speed);
    arm.setVelocityCmd(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (zero_position) arm.setPosition(0.0);
    arm.hold();
    arm.setMaxVel(old_max_speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;   //rely on the button to stop
  }
}
