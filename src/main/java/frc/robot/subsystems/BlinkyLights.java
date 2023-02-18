// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkyLights extends SubsystemBase {
  // State vars
  CANdle candle;
  CANdleConfiguration config;

  /** Creates a new BlinkyLights. */
  public BlinkyLights() {
    candle = new CANdle(0);
    config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 0.5; // dim the LEDs to half brightness
    candle.configAllSettings(config);
    candle.setLEDs(255, 165, 0); // set the CANdle LEDs to Orange
    candle.setLEDs(0, 0, 0); //set the CANdle LEDs to Black

  }

/*
* Brightness on a scale from 0-1, with 1 being max brightness
*/
  public void setBrightness(double brightness) {
    config.brightnessScalar = brightness;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
