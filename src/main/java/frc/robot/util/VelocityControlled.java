// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** Common API for Velocity controlled sub-systems where:
 *      hardware motor controller in velocity mode
 *      velocity is limits are supported
 *      software PID controls position
 *      position may be set to specific encoder location (no device movement)
 * 
 * Units are in whatever the device needs.
 *              pos = [base unit]
 *              velocity = [base unit/s]
 *  
 *  Main goal is to reuse simple tests with different devices
 */
public interface VelocityControlled extends Subsystem {

public double getSetpoint();
public void setSetpoint(double position);
public boolean atSetpoint();
public void setClamp(double min_pos, double max_pos);

//position methods
public void setPosition(double position);  //no motion, just sets encoders
public double getPosition();

// Control any needed trim bias off of position
public void setTrim(double trim);
public double getTrim();

//velocity methods
public void setMaxVel(double velocityMax);
public double getMaxVel();
public void setVelocityCmd(double velocity);
public double getVelocityCmd();
public double getVelocity();

public void hold();   

// consider exposing pids for testing/tuning

}
