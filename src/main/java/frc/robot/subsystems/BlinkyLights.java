// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkyLights extends SubsystemBase {
  // State vars
  CANdle candle;
  CANdleConfiguration config;
  Color orange = new Color(265, 165, 0); 
  Color blue = new Color(0, 0, 255);
  Color black = new Color(0, 0, 0);
   
// consider passing through a single Color parameter here --nren
public void setColor(int r, int g, int b){
    candle.setLEDs(r, g, b);
}

// each of these should call the setColor(r/g/b) command --nren
  public void setOrange(){
    candle.setLEDs(265, 165, 0);
  }
  public void setBlue(){
    candle.setLEDs(0, 0, 255);
  }
  public void setBlack(){
    candle.setLEDs(0, 0, 0);
  }

  
  /** Creates a new BlinkyLights. */
  public BlinkyLights() {
    // do something with the config --nren
   candle = new CANdle(0);
  }
/*
* Brightness on a scale from 0-1, with 1 being max brightness
*/

  public void setBrightness(double brightness) {
    // directl set the candle's brightness, not the config's brightness --nren
    config.brightnessScalar = brightness;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  // give me some NTs --nren
}
