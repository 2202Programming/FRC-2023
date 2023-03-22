// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.Claw_Substyem;

public class takeConeFromShelf extends SequentialCommandGroup {

  Claw_Substyem claw;
  ArmSS arm;
  MoveCollectiveArm armMove;
  public enum commandState {
    Init, ClawOpened, ArmExtended, HasCone, ArmRetracted;
      }

      commandState currentState;

  /** Creates a new takeConeFromShelf. */
  public takeConeFromShelf() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = RobotContainer.RC().claw;
    this.arm = RobotContainer.RC().armSS;

    // move collective arm to shelf pickup
    // start watch on claw lightgate 

    // on lightgate end cmd and schedule rotate wrist 
    addCommands(
      new ParallelCommandGroup(
            new MoveCollectiveArm(CollectivePositions.pickupShelfFS)
            //new WatchClawGate()
      ),
      new WaitCommand(0.1),   // did we really get it?
      new ConditionalCommand(
        new MoveCollectiveArm(CollectivePositions.haveConeAtShelf) , //true
        new MoveCollectiveArm(CollectivePositions.pickupShelfFS),    //false
        claw::isGateBlocked)
    
    );
    }
}
  
/**
    switch(currentState) {
      case Init:
        if (!claw.isOpen())
          claw.open();
        else currentState = commandState.ClawOpened;
        break;
      case ClawOpened:
        //TODO: Remove magic numbers and use the Position object attributes
        if (arm.getPosition() != 20)
        armMove = new MoveCollectiveArm(MoveCollectiveArm.CollectiveMode.pickupShelfFS);
        else currentState = commandState.ArmExtended;
        break;
      case ArmExtended:
        // TODO: Figure out the else part of this which would be to move the drivetrain slowly forward
        if (claw.isGateBlocked()) {
          claw.close();
          currentState = commandState.HasCone;
        }
        break;
      case HasCone:
        //TODO: Remove magic numbers and use the Position object attributes
        // Perhaps we want to move the bot back its starting position if we're moving the drivetrain forward until the gate breaks
        // and then retract the arm
        if (arm.getPosition() !=  15) {
          armMove = new MoveCollectiveArm(MoveCollectiveArm.CollectiveMode.reversePickupShelfFS);
        } else {
          currentState = commandState.ArmRetracted;
        }
        break;
        
      case ArmRetracted:
        break;
      }
 
  }

  */

