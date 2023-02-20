// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.RobotSpecs.RobotNames;

public class BlinkyLights extends SubsystemBase {
    // State vars
    CANdle candle_l;
    CANdle candle_r;
    double brightness;
    Color8Bit currentColor;
    boolean amIReal;

    /** Creates a new BlinkyLights. */
    public BlinkyLights() {
        amIReal = (RobotContainer.RC().robotSpecs.myRobotName == RobotNames.CompetitionBot2023);
        if(amIReal) {
            System.out.println("***I have blinkylights, I must be one of the cool robots.");
            candle_l = new CANdle(3);
            candle_r = new CANdle(4);
            candle_l.configAllSettings(new CANdleConfiguration());
            candle_r.configAllSettings(new CANdleConfiguration());
            ntcreate();
        }
        else{
            System.out.println("***I have no blinkylights :( ... setting up poser methods");
        }
    }

    public void setColor(Color8Bit color) {
        if(amIReal) {
            candle_l.setLEDs(color.red, color.green, color.blue);
            candle_r.setLEDs(color.red, color.green, color.blue);
            currentColor = color;
        }
    }

    public void setBlinking(Color8Bit color){
        if(amIReal){
            Animation animation = new StrobeAnimation(color.red, color.green, color.blue,0,0.5,8);

            candle_l.animate(animation,0);
            candle_r.animate(animation,0);
        }
    }

    public void stopBlinking(){
        if(amIReal){
            candle_l.clearAnimation(0);
            candle_r.clearAnimation(0);
        }
    }


    /*
     * Brightness on a scale from 0-1, with 1 being max brightness
     */

    public void setBrightness(double brightness) {
        if(amIReal) {
            candle_l.configBrightnessScalar(brightness);
            candle_r.configBrightnessScalar(brightness);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(amIReal) {
            ntUpdates();
        }
    }

    public void setAllianceColors(){
        Alliance alliance = DriverStation.getAlliance();
        System.out.println("***** Robot Alliance: " + DriverStation.getAlliance().name());
            switch (alliance) {
                case Blue:
                    setColor(new Color8Bit(0, 0, 255));
                    break;
                case Red:
                    setColor(new Color8Bit(255, 0, 0));
                    break;
                default:
                    setColor(new Color8Bit(0, 255, 0));
                    break;
            }
    }     

    /**
     * NETWORK TABLES AHH
     */

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Blinky Lights");

    NetworkTableEntry nt_brightness;
    NetworkTableEntry nt_color_r;
    NetworkTableEntry nt_color_g;
    NetworkTableEntry nt_color_b;

    public void ntcreate() {
        nt_brightness = table.getEntry("Brightness");
        nt_color_r = table.getEntry("Color-R");
        nt_color_g = table.getEntry("Color-G");
        nt_color_b = table.getEntry("Color-B");
    }

    public void ntUpdates() {
        nt_brightness.setDouble(brightness);
        nt_color_r.setInteger(currentColor.red);
        nt_color_g.setInteger(currentColor.green);
        nt_color_b.setInteger(currentColor.blue);

    }
}
