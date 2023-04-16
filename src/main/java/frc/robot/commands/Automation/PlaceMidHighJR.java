// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.HorizontalScoringBlock;
import frc.robot.Constants.HorizontalScoringSubstation;
import frc.robot.Constants.VerticalScoringLane;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmLockForDrivingFS;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.ElbowMoveTo;
import frc.robot.commands.EndEffector.WheelsOut;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.swerve.VelocityMove;
import frc.robot.commands.utility.NT_Print;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Claw_Substyem.ClawTrackMode;
import frc.robot.subsystems.ColorSensors;

public class PlaceMidHighJR extends CommandBase {
  
    // SSs
    public ColorSensors colorSensors = RobotContainer.RC().colorSensors;
    private Claw_Substyem claw = RobotContainer.RC().claw;
  
    // constants
    final double DEADZONE2 = 0.025; // [percent^2] this number is squared!!
    final double SPEED_MOVE = 0.5; // [m/s] speed moving to / from target
    final double TIME_MOVE = 1.0; // [s] time to move to / from target
    final double TIME_DROP = 0.5; // [s] time to wait after claw opens / wheels spin out before going back
    final double CUBE_TIME_DROP = 0.75; // [s] Cube only 
  
    // Deliery request vars
    private HorizontalScoringBlock horizontalRequest;
    private HorizontalScoringSubstation substationRequest;
    private VerticalScoringLane verticalRequest;

    //State machine vars
    Command cmd;
    private CommandState commandState;
    public enum CommandState {
      Moving, Placing, Retracting, Finished;
    }

    private NetworkTable table;
    private NetworkTableEntry nt_state;
    private NetworkTableEntry nt_subState;
    public final String NT_Name = "CommandStatus"; // expose data under Command table

  /**
   * Constructs and schedules a new Place command. Encompasses sdt movement, arm
   * extension / retraction.
   * Assumes piece is already in claw (this should be done upon picking up).
   * 
   * @param horizontalRequest The station (macro-level) request
   * @param substationRequest The substation (micro-level) request
   * @param verticalRequest The height
   */
  public PlaceMidHighJR(HorizontalScoringBlock horizontalRequest, 
                      HorizontalScoringSubstation substationRequest, 
                      VerticalScoringLane verticalRequest) {
    this.horizontalRequest = horizontalRequest;
    this.substationRequest = substationRequest;
    this.verticalRequest = verticalRequest;

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    nt_state = table.getEntry("/PlaceMidHighJR State");
    nt_subState = table.getEntry("/PlaceMidHighJR SubState");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nt_state.setString("Init");
    
    // 1. Move to safe location for arm extension 
    cmd = new goToScoringPosition(new PathConstraints(2, 3), horizontalRequest, substationRequest);
    nt_subState.setString("Moving to "+horizontalRequest.toString()+","+substationRequest.toString());
    System.out.println("***PlaceMidHighJR scheduling move command...");
    cmd.schedule();
    commandState = CommandState.Moving;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    nt_state.setString(commandState.toString());

    //demorgans of the guard inside each case, 
    if (!cmd.isFinished() &&  cmd.isScheduled())
      return;

    switch(commandState){
      case Moving:
          commandState = CommandState.Placing;
          // 2. Move arm out and drop, then wait 1sec 
          cmd = (substationRequest == HorizontalScoringSubstation.Center) ? Cube() : Cone();
         break;

      case Placing:
          commandState = CommandState.Retracting;
          // 3. Move back and retract arm to travel position
          cmd = Retract();
          break;

      case Retracting:
          commandState = CommandState.Finished;
          cmd = null;
          break;
      default:
        break;
      }

      if (cmd != null) {
        cmd.schedule();
        System.out.print("****PlaceMidHighJR scheduled cmdSt= "+ commandState.toString());
      }
      nt_subState.setString(commandState.toString());
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("***PlaceMidHighJR done... interrupted=" + interrupted);
    nt_state.setString("Ended intr=" + interrupted);
    if (interrupted) 
      cmd.cancel();
    //return to normal vision updates
    RobotContainer.RC().drivetrain.enableVisionPose();
    RobotContainer.RC().drivetrain.disableVisionPoseRotation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //finished when state machine in finished state, or if driver moves stick
    if (RobotContainer.RC().dc.rightStickMotionDriver()) {
      cmd.cancel();   //driver aborting, abort the child cmd too.
    }
    return ((commandState == CommandState.Finished) || RobotContainer.RC().dc.rightStickMotionDriver());
  }



  /**
   * Constructs pacing this based on object being a cone facing forward.
   */
  private SequentialCommandGroup Cone() {
    SequentialCommandGroup command = new SequentialCommandGroup();

    switch (verticalRequest) {
      case High:
        command.addCommands(new PrintCommand("***PlaceMidHigh: Running PlaceTele high..."));
        command.addCommands(new PlaceTele(CollectivePositions.placeConeHighFS));
        break;
      case Middle:
        command.addCommands(new PrintCommand("***PlaceMidHigh: Running PlaceTele mid..."));
        command.addCommands(new PlaceTele(CollectivePositions.placeConeMidFS));
        break;
      default:
        break;
    }

    command.addCommands(
      new PrintCommand("***PlaceMidHigh: Running ElbowMove..."),
      new InstantCommand(() -> { claw.setTrackElbowMode(ClawTrackMode.frontSide); }), //level claw
      new ElbowMoveTo(95.0, 60.0), // lower to dropping position
      new PrintCommand("***PlaceMidHigh: Running Claw open..."),
      new InstantCommand(() -> { claw.open();} ),
      new WaitCommand(TIME_DROP),
      new PrintCommand("***Done with instants Claw open..."));

  return command;

  }

  /**
   * Constructs placing this based on object being a cube.
   * Coming out of travel position.
   */
  private SequentialCommandGroup Cube() {
    SequentialCommandGroup command = new SequentialCommandGroup();
    switch (verticalRequest) {
      case High:
        command.addCommands(new NT_Print(nt_subState, "Running PlaceTeleConeHigh"));
        command.addCommands(new PrintCommand("***PlaceMidHigh: Running placetele high..."));
        command.addCommands(new PlaceTele(CollectivePositions.placeConeHighFS));
        break;
      case Middle:
        command.addCommands(new NT_Print(nt_subState, "Running PlaceTeleConeMid"));
        command.addCommands(new PrintCommand("***PlaceMidHigh: Running placetele mid..."));
        command.addCommands(new PlaceTele(CollectivePositions.placeConeMidFS));
        break;
      default:
        break;
    }

    command.addCommands(
      new NT_Print(nt_subState, "Running wheels out"),
      new PrintCommand("***PlaceMidHigh: Running wheels out..."),
      new WheelsOut().withTimeout(CUBE_TIME_DROP));  //wait for claw to open and cube eject

    return command;
  }

  /**
   * Retracts piece
   */
  private SequentialCommandGroup Retract() {
    SequentialCommandGroup command = new SequentialCommandGroup();
    
    command.addCommands(
      new NT_Print(nt_subState, "Running elbow move"),
      new PrintCommand("***PlaceMidHighJR: Running elbow move..."),
      new InstantCommand(()->{claw.setTrackElbowMode(ClawTrackMode.faceUp);}), //move wrist back up
      new ElbowMoveTo(110.0), //return to high position to avoid low post
      new ParallelCommandGroup(
        new VelocityMove(0.90, 0.0, 1.11),
        new SequentialCommandGroup(
          new WaitCommand(0.5), //let the move start first for 0.5s so arm doesn't catch low pole
          new NT_Print(nt_subState, "Running arm lock for driving FS"),
          new PrintCommand("***PlaceMidHigh: Running arm lock for driving FS..."),
          new ArmLockForDrivingFS() //then start to retract arm
        )
      )
      );
    return command;
  }

}
