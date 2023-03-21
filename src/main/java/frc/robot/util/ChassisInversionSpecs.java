// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

public class ChassisInversionSpecs{
  public ModuleInversionSpecs FR;
  public ModuleInversionSpecs FL;
  public ModuleInversionSpecs BR;
  public ModuleInversionSpecs BL;

  public ChassisInversionSpecs(ModuleInversionSpecs FR, ModuleInversionSpecs FL, ModuleInversionSpecs BR, ModuleInversionSpecs BL){
      this.FR = FR;
      this.FL = FL;
      this.BR = BR;
      this.BL = BL;
  }
}

