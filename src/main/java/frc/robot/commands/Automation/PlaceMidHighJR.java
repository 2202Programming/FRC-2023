// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.HorizontalSubstationLane;
import frc.robot.Constants.VerticalScoringLane;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmLockForDrivingFS;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.ElbowMoveTo;
import frc.robot.commands.EndEffector.WheelsOut;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.auto.moveToPoint;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

public class PlaceMidHighJR extends CommandBase {
  
    // SSs
    private HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
    public ColorSensors colorSensors = RobotContainer.RC().colorSensors;
    private Claw_Substyem claw = RobotContainer.RC().claw;
  
    // constants
    final double DEADZONE2 = 0.025; // [percent^2] this number is squared!!
    final double SPEED_MOVE = 0.5; // [m/s] speed moving to / from target
    final double TIME_MOVE = 1.0; // [s] time to move to / from target
    final double TIME_DROP = 0.5; // [s] time to wait after claw opens / wheels spin out before going back
  
    // state vars
    private HorizontalScoringLane horizontalRequest;
    private HorizontalSubstationLane substationRequest;
    private VerticalScoringLane verticalRequest;
    private Pose2d goalPose;

    private goToScoringPosition moveCommand;
    private SequentialCommandGroup placeCommand;
    private SequentialCommandGroup retractCommand;

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
  public PlaceMidHighJR(HorizontalScoringLane horizontalRequest, 
                      HorizontalSubstationLane substationRequest, 
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
    goalPose = calculateTargetPose();
    // 1. Move to safe location for arm extension 
    moveCommand = new goToScoringPosition(new PathConstraints(2, 3), horizontalRequest, substationRequest);
    nt_subState.setString("Moving to "+horizontalRequest.toString()+","+substationRequest.toString());
    System.out.println("***PlaceMidHighJR scheduling move command...");
    moveCommand.schedule();
    commandState = CommandState.Moving;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    nt_state.setString(commandState.toString());

    switch(commandState){
      case Moving:
        if(moveCommand.isFinished()){
          commandState = CommandState.Placing;
          // 2. Move arm out and drop, then wait 1sec
          switch (substationRequest) {
            case Center:
              placeCommand = Cube();
              break;
            default:
              placeCommand = Cone();
              break;
          }
          nt_subState.setString("Placing");
          System.out.println("***PlaceMidHighJR scheduling place command...");
          placeCommand.schedule();
        }
        break;
      case Placing:
        if(placeCommand.isFinished()){
          commandState = CommandState.Retracting;
          // 3. Move back and retract arm to travel position
          retractCommand = Retract();
          nt_subState.setString("Retracting");
          System.out.println("***PlaceMidHighJR scheduling retract command...");
          retractCommand.schedule();
        }
        break;
      case Retracting:
        if(retractCommand.isFinished()){
          commandState = CommandState.Finished;
          nt_subState.setString("Finishing");
          System.out.println("***PlaceMidHighJR retract done, finishing up..");
        }
        break;
      default:
        break;
      }
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("***PlaceMidHighJR done... interrupted=" + interrupted);
    nt_state.setString("Ended");
    RobotContainer.RC().drivetrain.enableVisionPose();
    RobotContainer.RC().drivetrain.disableVisionPoseRotation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //finished when state machine in finished state, or if driver moves stick
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
      new ElbowMoveTo(95.0, 60.0), // lower to dropping position
      new PrintCommand("***PlaceMidHigh: Running Claw open..."),
      new InstantCommand(() -> { claw.open();}).andThen(new WaitCommand(TIME_DROP)));

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
        command.addCommands(new PrintCommand("***PlaceMidHigh: Running placetele high..."));
        command.addCommands(new PlaceTele(CollectivePositions.placeConeHighFS));
        break;
      case Middle:
        command.addCommands(new PrintCommand("***PlaceMidHigh: Running placetele mid..."));
        command.addCommands(new PlaceTele(CollectivePositions.placeConeMidFS));
        break;
      default:
        break;
    }

    command.addCommands(
      new PrintCommand("***PlaceMidHigh: Running wheels out..."),
      new WheelsOut().withTimeout(TIME_DROP));  //wait for claw to open and cone drop

    return command;
  }

  /**
   * Retracts piece
   */
  private SequentialCommandGroup Retract() {
    SequentialCommandGroup command = new SequentialCommandGroup();
    Pose2d retractPose;
    double distance = 0.6; //how far back to move (m)

    if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) { //BLUE ALLIANCE
      retractPose = new Pose2d(goalPose.getX() + distance, goalPose.getY(), goalPose.getRotation()); //distance away from scoring station, blue side
    }
    else{
      retractPose = new Pose2d(goalPose.getX() - distance, goalPose.getY(), goalPose.getRotation()); //distance away from scoring station, red side
    }

    command.addCommands(
      new PrintCommand("***PlaceMidHigh: Running elbow move..."),
      new ElbowMoveTo(145.0), //return to high position to avoid low post
      new ParallelCommandGroup(
        new PrintCommand("***PlaceMidHigh: Running move to point retract pose..."),
        new moveToPoint(new PathConstraints(1.0, 1.0), retractPose), //move slowly back while retracting arm
        new SequentialCommandGroup(
          new WaitCommand(3.0), //let the move start first for 0.5s so arm doesn't catch low pole
          new PrintCommand("***PlaceMidHigh: Running arm lock for driving FS..."),
          new ArmLockForDrivingFS() //then start to retract arm
        )
      )
      );
    return command;
  }

  //calculate which constant scoring pose is appropriate goal
  private Pose2d calculateTargetPose(){
    //FOR BLUE: 2 for left (driver's point of view), 1 for center, 0 for right
    HorizontalScoringLane horizontalScoringLane = this.horizontalRequest;
    HorizontalSubstationLane horizontalSubstationLane = this.substationRequest;
    int scoringBlock; 
    int scoringAdjusted;
    Pose2d targetPose;

  if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) { //BLUE ALLIANCE


    if(horizontalSubstationLane.equals(HorizontalSubstationLane.Left)) scoringBlock = 2;
    else if(horizontalSubstationLane.equals(HorizontalSubstationLane.Right)) scoringBlock = 0;
    else scoringBlock = 1;

    //FOR BLUE: left is largest index of scoring trio
    switch(horizontalScoringLane){
      case Left:
        scoringAdjusted = 2;
        break;
      case Center:
        scoringAdjusted = 1;
        break;
      default:
      case Right:
        scoringAdjusted = 0;
        break;      
    }
    targetPose = Constants.FieldPoses.blueScorePoses[scoringBlock][scoringAdjusted];
  }
  else { //RED ALLIANCE
    //FOR RED: 0 for left (driver's point of view), 1 for center, 2 for right
    if(horizontalSubstationLane.equals(HorizontalSubstationLane.Left)) scoringBlock = 0;
    else if(horizontalSubstationLane.equals(HorizontalSubstationLane.Right)) scoringBlock = 2;
    else scoringBlock = 1;

    //FOR RED: left is smallest index of scoring trio
    switch(horizontalScoringLane){
      case Left:
        scoringAdjusted = 0;
        break;
      case Center:
        scoringAdjusted = 1;
        break;
      default:
      case Right:
        scoringAdjusted = 2;
        break;      
    }
    targetPose = Constants.FieldPoses.blueScorePoses[scoringBlock][scoringAdjusted];
  }
 return targetPose;
}
}
