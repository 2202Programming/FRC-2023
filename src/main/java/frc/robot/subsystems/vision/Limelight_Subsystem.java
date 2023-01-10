// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Shooter;

public class Limelight_Subsystem extends SubsystemBase {
  /** Creates a new Limelight_Subsystem. */

  private NetworkTable table;
  private NetworkTable outputTable;
  private NetworkTable shooterTable;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private NetworkTableEntry leds;
  private NetworkTableEntry booleanLeds;
  private NetworkTableEntry NT_hasTarget;

  private NetworkTableEntry outputTx;
  private NetworkTableEntry outputTv;

  private double x;
  private double filteredX;
  private double y;
  private double area; // area is between 0 and 100. Calculated as a percentage of image
  private boolean target;
  private boolean ledStatus; // true = ON
  private double filteredArea;

  private LinearFilter x_iir;
  private LinearFilter area_iir;
  public final String NT_Name = "DT"; // expose data under DriveTrain table
  final String NT_Shooter_Name = "Shooter"; 
  private double filterTC = 0.08;     // seconds, 2Hz cutoff T = 1/(2pi*f)  was .2hz T=.8
  private int log_counter = 0;

  public Limelight_Subsystem() {
    x_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
    area_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    outputTable = NetworkTableInstance.getDefault().getTable(NT_Name);
    shooterTable = NetworkTableInstance.getDefault().getTable(NT_Shooter_Name);

    tx = table.getEntry("tx"); // -27 degrees to 27 degrees
    ty = table.getEntry("ty"); // -20.5 to 20.5 degrees
    ta = table.getEntry("ta");
    tv = table.getEntry("tv"); // target validity (1 or 0)
    leds = table.getEntry("ledMode");
    booleanLeds = table.getEntry("booleanLeds");

    NT_hasTarget = shooterTable.getEntry("LL_Has_Target");

    outputTv = outputTable.getEntry("Limelight Valid");
    outputTx = outputTable.getEntry("Limelight X error");
    disableLED();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    target = (tv.getDouble(0) == 0) ? (false) : (true); //tv is only 0.0 or 1.0 per LL docs
    filteredX = x_iir.calculate(x);
    filteredArea = area_iir.calculate(area);
    ledStatus = (leds.getDouble(0) == 3) ? (true) : (false);

  }

  public double estimateDistance() {
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    // how many degrees back is your limelight rotated from perfectly vertical?


    // both because why not (and that's what the copy-pasta had)
    double angleToGoalDegrees = Shooter.LL_MOUNT_ANGLE_DEG + targetOffsetAngle_Vertical; 
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    //calculate distance
    return (((Shooter.GOAL_HEIGHT_TO_FLOOR_INCHES - Shooter.LL_LENS_HEIGHT_INCHES)/Math.tan(angleToGoalRadians) + Shooter.EDGE_TO_CENTER_INCHES) / Shooter.METERS_TO_INCHES);
  }

  public double getX() {
    return x;
  }

  public double getFilteredX() {
    return filteredX;
  }

  public double getFilteredArea() {
    return filteredArea;
  }

  public double getY() {
    return y;
  }

  public double getArea() {
    return area;
  }

  public boolean getTarget() {
    return target;
  }

  public boolean getLEDStatus() {
    return ledStatus;
  }

  public void disableLED() {
    leds.setNumber(1);
    ledStatus = false;
    booleanLeds.setBoolean(ledStatus);
  }

  public void enableLED() {
    leds.setNumber(3);
    ledStatus = true;
    booleanLeds.setBoolean(ledStatus);
  }

  public void toggleLED() {
    if (ledStatus) {
      disableLED();
    } else {
      enableLED();
    }

  }

  public boolean valid() {
    return target;
  }

  public void log() {
    log_counter++;
    if(log_counter%20 == 0){
      NT_hasTarget.setBoolean(target);
      outputTv.setValue(target);
      outputTx.setDouble(x);
      // SmartDashboard.putBoolean("Target Lock", target);
    }
  }

}
