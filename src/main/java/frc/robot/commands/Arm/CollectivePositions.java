// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import frc.robot.Constants.PowerOnPos;
import frc.robot.commands.Arm.MoveCollectiveArm.Positions;
import frc.robot.subsystems.Claw_Substyem.ClawTrackMode;
import frc.robot.Constants.ConePickup;;

/** Add your docs here. */
public enum CollectivePositions {
    power_on(PowerOnPos.arm, PowerOnPos.elbow, PowerOnPos.wrist, ClawTrackMode.backSide, 10.0, -1.0), 
    travelLockNoPieceBS(0.0, PowerOnPos.elbow, PowerOnPos.wrist -5.0 , ClawTrackMode.free, -1.0, 60.0), 

    //TODO ORGANIZE OR MOVE THIS
    travelFS(0.0, -28.0, 93.0, ClawTrackMode.frontSide, -1.0, 60.0),
    travelLockFS(0.0, -28.0, 93.0, ClawTrackMode.free, -1.0, 60.0), 
   
    
    
    pickupShelfFS(ConePickup.armLength, ConePickup.elbowAngle, 0.0, ClawTrackMode.frontSide),
    haveConeAtShelf(ConePickup.armLength, ConePickup.elbowAngle, 35.0, ClawTrackMode.free),   //assumes wrist near zero
    
    placeConeMidFS(12.0, 130.0, -51.0, ClawTrackMode.frontSide),
    placeCubeMidFS(12.0, 125.0, -51.0, ClawTrackMode.frontSide),
    placeConeHighFS(37.0, 135.0, -40.0, ClawTrackMode.frontSide),
    placeCubeHighFS(37.0, 105.0, -50.0, ClawTrackMode.frontSide),
   
    pickupTransitionFS(15.0, 105.0, 0.0, ClawTrackMode.frontSide),
    placeMidFS(20.0, 90.0, 0.0, ClawTrackMode.frontSide),
    
    testShelfTopFS(38.0, 165.0, 0.0, ClawTrackMode.frontSide, -1.0, 70.0),
    reversePickupShelfFS(15.0, -90.0, 0.0, ClawTrackMode.frontSide),
    midFS(20.0, 0.0, 0.0, ClawTrackMode.frontSide),
    midBS(20.0, 0.0, 0.0, ClawTrackMode.backSide),
    placeHighFS(38.0, 105.0, 0.0, ClawTrackMode.frontSide),
    travelMidFS(20.0, -10.0, 0.0, ClawTrackMode.frontSide),
    travelMidBS(20.0, -10.0, 0.0, ClawTrackMode.backSide);

    // posistions and modes for target positions
    Positions pos_info;

    CollectivePositions(double arm, double elbow, double wrist, ClawTrackMode mode, double armMaxVel, double elbowMaxVel) {
      pos_info = new Positions(arm, elbow, wrist, mode, armMaxVel, elbowMaxVel);
    }

    CollectivePositions(double arm, double elbow, double wrist, ClawTrackMode mode) {
      pos_info = new Positions(arm, elbow, wrist, mode);
    }
  };