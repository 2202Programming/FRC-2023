// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ConePickup;
import frc.robot.Constants.PowerOnPos;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Elbow;
import frc.robot.commands.Arm.*;
import frc.robot.Constants;

public class takeConeFromShelf extends CommandBase {

  Claw_Substyem claw;
  ArmSS arm;
  ArmMoveTo armMove;
  public enum commandState {
    Init, ClawOpened, ArmExtended, HasCone, ArmRetracted;
      }

      commandState currentState;

  /** Creates a new takeConeFromShelf. */
  public takeConeFromShelf() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = RobotContainer.RC().claw;
    this.arm = RobotContainer.RC().armSS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = commandState.Init; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(currentState) {
      case Init:
        if (!claw.isOpen())
          claw.open();
        else currentState = commandState.ClawOpened;
        break;
      case ClawOpened:
        if (arm.getPosition() != ConePickup.armLength)
        armMove = new ArmMoveTo(ConePickup.armLength, ConePickup.elbowAngle);
        else currentState = commandState.ArmExtended;
        break;
      case ArmExtended:
        // TODO
        if (claw.isGateBlocked()) {
          claw.close();
          currentState = commandState.HasCone;
        }
        break;
      case HasCone:
        // TODO
        if (arm.getPosition() !=  PowerOnPos.arm) {
          armMove = new ArmMoveTo(PowerOnPos.arm, PowerOnPos.elbow);
        } else {
          currentState = commandState.ArmRetracted;
        }
        break;
        
      case ArmRetracted:
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
