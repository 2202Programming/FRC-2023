// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BlinkyLights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import frc.robot.subsystems.BlinkyLights;

public class BlinkyLight extends CommandBase {
  /** Creates a new BlinkyLight. */
  public BlinkyLight() {
    candle = new CANdle(0);
    config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 1.0; // Full Bright (Blind)
    candle.configAllSettings(config);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    candle.setLEDs(255, 165, 0); // set the CANdle LEDs to Orange
    candle.setLEDs(0, 0, 0); //set the CANdle LEDs to Black
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
