// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.utility.Lockout;
import frc.robot.subsystems.ArmSS;

public class ArmMoveAtSpeed_L_R_test extends CommandBase implements Lockout {
private final GenericHID m_hid;

  ArmSS arm;
  boolean zero_position = false;
  double speed; //[cm/s]
  double old_max_speed;

  public ArmMoveAtSpeed_L_R_test(double speed,int HIDPORT) {
    this.arm = RobotContainer.RC().armSS;
    this.speed = speed;
    m_hid = new GenericHID(HIDPORT);

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    old_max_speed = arm.getMaxVel();
    arm.setMaxVel(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    int dpad = m_hid.getPOV();
      if (dpad == 90) {
        arm.setVelocityRight(speed);
        arm.setVelocityLeft(0.0); 
      }
      else if (dpad == 270 ) {
        arm.setVelocityRight(0.0);
        arm.setVelocityLeft(speed); 
      }
      else {
        arm.setVelocityCmd(0.0);
      }
      //this is a test command that is used to align pos
      // so set positions to zero
      arm.setPosition(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    arm.setPosition(0.0);
    arm.hold();
    arm.setMaxVel(old_max_speed);
    //done moving to new zero, set it so
    arm.setSetpoint(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;   //rely on the button to stop
  }
}
