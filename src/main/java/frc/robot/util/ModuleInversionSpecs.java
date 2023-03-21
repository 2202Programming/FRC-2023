// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

  /**
   * Inversions account for rotations of the module relative to left or right side
   * of robot.
   * 
   * CANCoders are setup in Sensors and will have CCW= positve convention. Their
   * offsets are adjusted by their use in the drive train.
   */

public class ModuleInversionSpecs{
    public boolean kDriveMotorInvert;
    public boolean kAngleMotorInvert;
    public boolean kAngleCmdInvert;

    public ModuleInversionSpecs(boolean kDriveMotorInvert, boolean kAngleMotorInvert, boolean kAngleCmdInvert){
        this.kAngleMotorInvert = kDriveMotorInvert;
        this.kAngleMotorInvert = kAngleMotorInvert;
        this.kAngleCmdInvert = kAngleCmdInvert;
    }

    public ModuleInversionSpecs(){
        this(false,false,false);
    }
  }