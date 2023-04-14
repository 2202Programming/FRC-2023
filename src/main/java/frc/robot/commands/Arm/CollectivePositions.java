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
    power_on(PowerOnPos.arm, PowerOnPos.elbow, PowerOnPos.wrist, ClawTrackMode.backSide, 18.0, 60.0),
    travelNoPieceBS(PowerOnPos.arm, 10.0, PowerOnPos.wrist, ClawTrackMode.backSide, 18.0, 120.0),  
    // 4/11/23 + 2 deg instead of 5 so we dont trip stall detect
    travelLockNoPieceBS(0.0, PowerOnPos.elbow, PowerOnPos.wrist + 2.0, ClawTrackMode.free, -1.0, 120.0), 
    safeToFlip(0.0, 70.0, 0.0, ClawTrackMode.free, -1.0, 60),
    
    //TODO ORGANIZE OR MOVE THIS
    travelFS(0.0, -20.0, 85.0, ClawTrackMode.frontSide, -1.0, 120.0), // TODO -28.0 4/8/2023
    travelLockFS(0.0, -20.0, 85.0, ClawTrackMode.free, -1.0, 120.0), // TODO -28.0 4/8/2023

    // upright cone pickup position
    uprightConePickup(0.0, 15.0, -30.0, ClawTrackMode.free),
    uprightConeTravelHalfway(0.0, 15.0, 20.0, ClawTrackMode.free),

    // cube car wash to claw
    cubeToClaw(0.0, 10.0, 0.0, ClawTrackMode.backSide),  

    pickupShelfFS(ConePickup.armLength, ConePickup.elbowAngle, 0.0, ClawTrackMode.frontSide),
    haveConeAtShelf(ConePickup.armLength, ConePickup.elbowAngle, 35.0, ClawTrackMode.free),   //assumes wrist near zero
    
    
    placeConeMidFS(12.0, 130.0, -51.0, ClawTrackMode.frontSide),
    placeCubeMidFS(12.0, 125.0, -51.0, ClawTrackMode.frontSide),

    placeConeHighFS(39.0, 130.0, -40.0, ClawTrackMode.frontSide),  
    placeCubeHighFS(39.0, 105.0, -50.0, ClawTrackMode.frontSide),
   
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