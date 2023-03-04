// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Elbow;

public class MoveCollectiveArm extends CommandBase {
  ArmSS arm = RobotContainer.RC().armSS;
  Elbow elbow = RobotContainer.RC().elbow;
  Claw_Substyem claw = RobotContainer.RC().claw;

  // {arm, elbow}, wrist tracks elbow for now [cm, deg]
  static final double[] travelPos  = {0.0, 10.0};
  static final double[] midPos  = {20.0, 90.0};
  static final double[] highPos = {35.0, 120.0};
  
  static final double positions[][] = {travelPos, midPos, highPos  };

  final double target[];
  enum CollectiveMode {
    travel, mid, high
  };

  /** Creates a new MoveCollectiveArm. */
  public MoveCollectiveArm(CollectiveMode where) {
    target = positions[where.ordinal()];
    addRequirements(arm, elbow, claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setSetpoint(target[0]);
    elbow.setSetpoint(target[1]);
    claw.setElbowDoubleSupplier(elbow::getPosition);
    //claw.is
}

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
