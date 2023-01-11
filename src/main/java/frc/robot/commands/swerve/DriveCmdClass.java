// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;

/* This class allows drive controller to set pitch/roll correction without knowing exactly which drive command is running */
/* Possibly could have been an interface but I didn't think that would work */

public class DriveCmdClass extends CommandBase {
    public double roll_correction = 0;
    public double pitch_correction = 0;

    void setPitchCorrection(double factor)
    {
      pitch_correction = factor;
    }
  
    void setRollCorrection(double factor)
    {
      roll_correction = factor;
    }

}
