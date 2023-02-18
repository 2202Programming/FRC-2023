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
   
public void setColor(int r, int g, int b){
    Color color = new Color(r, g, b);
    candle.setLEDs(color);
}

  public void setOrange(){
    candle.setLEDs(orange);
  }
  public void setBlue(){
    candle.setLEDs(blue);
  }
  public void setBlack(){
    candle.setLEDs(black);
  }

  
  /** Creates a new BlinkyLights. */
  public BlinkyLights() {
   candle = new CANdle(0);
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
