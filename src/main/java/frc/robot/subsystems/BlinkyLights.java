// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BlinkyLights extends SubsystemBase {
  // State vars
  CANdle candle;
  double brightness;
  Color8Bit currentColor;
  
   
public void setColor(Color8Bit color){
    candle.setLEDs(color.red, color.blue, color.green);
    currentColor = color;
}


  
  /** Creates a new BlinkyLights. */
  public BlinkyLights() {
   candle = new CANdle(0);
   candle.configAllSettings(Constants.BlinkyLights.config);
   ntcreate();
  }
/*
* Brightness on a scale from 0-1, with 1 being max brightness
*/

  public void setBrightness(double brightness) {
    candle.configBrightnessScalar(brightness);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
ntUpdates();
  }

  /** 
   * NETWORK TABLES AHH
   */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("Blinky Lights");

  NetworkTableEntry nt_brightness;
  NetworkTableEntry nt_color_r;
  NetworkTableEntry nt_color_g;
  NetworkTableEntry nt_color_b;
  public void ntcreate(){
    nt_brightness = table.getEntry("Brightness");
    nt_color_r = table.getEntry("Color-R");
    nt_color_g = table.getEntry("Color-G");
    nt_color_b = table.getEntry("Color-B");
  }

  public void ntUpdates(){
    nt_brightness.setDouble(brightness);
    nt_color_r.setInteger(currentColor.red);
    nt_color_g.setInteger(currentColor.green);
    nt_color_b.setInteger(currentColor.blue);

  }
}
