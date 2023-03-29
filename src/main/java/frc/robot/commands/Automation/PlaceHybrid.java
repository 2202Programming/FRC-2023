// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.HorizontalSubstationLane;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.Washer.outtakeCompetitionToggle;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.swerve.RotateTo;
import frc.robot.commands.swerve.VelocityMove;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.DynamicSCG;

public class PlaceHybrid extends DynamicSCG {
  // State vars

  HorizontalScoringLane horizontalRequest;
  HorizontalSubstationLane substationRequest;

  // SSs
  HID_Xbox_Subsystem dc = RobotContainer.RC().dc; 

  
  /** Creates a new PlaceHybrid. */
  public PlaceHybrid(HorizontalScoringLane horizontalRequest, HorizontalSubstationLane substationRequest) {
    this.horizontalRequest = horizontalRequest;
    this.substationRequest = substationRequest;
  }

  // Called when the command is initially scheduled.
  @Override
  public void doFirstOnInit() {
    // 1. Move to place
    this.addCommands(
      new goToScoringPosition(new PathConstraints(2.0, 3.0), horizontalRequest, substationRequest), 
      new RotateTo(new Rotation2d((DriverStation.getAlliance().equals(Alliance.Blue)) ? 180 : 0)),
      new VelocityMove(1.0, 0.0, 0.5) 
      // 0.5 magic number results in an integral of 1/2m because that's how much we're off by 
      );

    // 2. Eject
    this.addCommands(new outtakeCompetitionToggle().withTimeout(3.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinishedCondition() {
    return dc.rightStickMotionDriver();
  }
}
