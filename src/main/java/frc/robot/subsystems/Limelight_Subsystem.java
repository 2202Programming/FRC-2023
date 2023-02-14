// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Shooter;

public class Limelight_Subsystem extends SubsystemBase {
  /** Creates a new Limelight_Subsystem. */

  private NetworkTable table;
  private NetworkTable outputTable;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;
  private NetworkTableEntry leds;
  private NetworkTableEntry booleanLeds;
  private NetworkTableEntry NT_hasTarget;

  private NetworkTableEntry outputTx;
  private NetworkTableEntry outputTv;
  private NetworkTableEntry pipelineNTE;

  private NetworkTableEntry nt_botpose;

  private double x;
  private double filteredX;
  private double y;
  private double area; // area is between 0 and 100. Calculated as a percentage of image
  private boolean target;
  private boolean ledStatus; // true = ON
  private double filteredArea;

  // botpose index keys
  static final int X = 0;
  static final int Y = 1;
  static final int Z = 2;
  static final int RX = 3;
  static final int RY = 4;
  static final int RZ = 5;

  private double[] botpose;
  /*
   * DPL - see code below delete this if it works as expected
   * private double botpose_x;
   * private double botpose_y;
   * private double botpose_z;
   * private double botpose_rx;
   * private double botpose_ry;
   * private double botpose_rz;
   */
  private long pipeline;

  private LinearFilter x_iir;
  private LinearFilter area_iir;
  public final String NT_Name = "DT"; // expose data under DriveTrain table
  final String NT_Shooter_Name = "Shooter";
  private double filterTC = 0.08; // seconds, 2Hz cutoff T = 1/(2pi*f) was .2hz T=.8
  private int log_counter = 0;

  private LimelightHelpers.LimelightResults llresults;
  private Pose2d megaPose;
  private Pose2d teamPose;
  private Pose2d bluePose;
  final private String LL_NAME = "";// "limelight" for if left blank

  public Limelight_Subsystem() {
    x_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
    area_iir = LinearFilter.singlePoleIIR(filterTC, Constants.Tperiod);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    outputTable = NetworkTableInstance.getDefault().getTable(NT_Name);

    tx = table.getEntry("tx"); // -27 degrees to 27 degrees
    ty = table.getEntry("ty"); // -20.5 to 20.5 degrees
    ta = table.getEntry("ta");
    tv = table.getEntry("tv"); // target validity (1 or 0)
    leds = table.getEntry("ledMode");
    booleanLeds = table.getEntry("booleanLeds");
    pipelineNTE = table.getEntry("pipeline");

    nt_botpose = table.getEntry("botpose");

    outputTv = outputTable.getEntry("Limelight Valid");
    outputTx = outputTable.getEntry("Limelight X error");

    nt_botpose.setDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 });
    disableLED();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    target = (tv.getDouble(0) == 0) ? (false) : (true); // tv is only 0.0 or 1.0 per LL docs
    filteredX = x_iir.calculate(x);
    filteredArea = area_iir.calculate(area);
    ledStatus = (leds.getDouble(0) == 3) ? (true) : (false);
    pipeline = pipelineNTE.getInteger(0);

    botpose = nt_botpose.getDoubleArray(new double[] { 0, 0, 0, 0, 0, 0 });
    llresults = LimelightHelpers.getLatestResults("");

    // if (botpose.length > 0) {
    /*
     * DPL - this should be the same without the copy
     * botpose_x = botpose[X];
     * botpose_y = botpose[Y];
     * botpose_z = botpose[Z];
     * botpose_rx = botpose[RX];
     * botpose_ry = botpose[RY];
     * botpose_rz = botpose[RZ];
     */
    // NOTE: LL gives position from the center of the field! Need to transform to
    // the standard of 0,0 at lower left

    // botpose[X] += 8.270458; //add 1/2 field X dimension in meters
    // botpose[Y] += 4.008216; //add 1/2 field Y dimension in meters
    // }
    if (NT_hasTarget.equals(true)) {
      // new way of grabbing LL results
      double[] tempPose = LimelightHelpers.getBotPose(LL_NAME);
      megaPose = new Pose2d(tempPose[X], tempPose[Y], Rotation2d.fromDegrees(tempPose[RZ]));

      // TeamPose

      double[] teamTempPose;
      if (RobotContainer.RC().robotSpecs.isBlue()) {
        teamTempPose = LimelightHelpers.getBotPose_wpiBlue(LL_NAME);
      } else {
        teamTempPose = LimelightHelpers.getBotPose_wpiRed(LL_NAME);
      }
      double[] tempBluePose = LimelightHelpers.getBotPose_wpiBlue(LL_NAME);

      teamPose = new Pose2d(teamTempPose[X], teamTempPose[Y], Rotation2d.fromDegrees(teamTempPose[RZ]));
      bluePose = new Pose2d(tempBluePose[X], tempBluePose[Y], Rotation2d.fromDegrees(tempBluePose[RZ]));

      SmartDashboard.putNumber("LL botpose X", megaPose.getX());
      SmartDashboard.putNumber("LL botpose Y", megaPose.getY());
      SmartDashboard.putNumber("LL botpose Yaw", megaPose.getRotation().getDegrees());

      SmartDashboard.putNumber("LL teampose X", teamPose.getX());
      SmartDashboard.putNumber("LL teampose Y", teamPose.getY());
      SmartDashboard.putNumber("LL teampose Yaw", teamPose.getRotation().getDegrees());

      SmartDashboard.putNumber("LL bluePose X", bluePose.getX());
      SmartDashboard.putNumber("LL bluePose Y", bluePose.getY());
      SmartDashboard.putNumber("LL bluePose Yaw", bluePose.getRotation().getDegrees());
    }
  }

  public double estimateDistance() {
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    // how many degrees back is your limelight rotated from perfectly vertical?

    // both because why not (and that's what the copy-pasta had)
    double angleToGoalDegrees = Shooter.LL_MOUNT_ANGLE_DEG + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    // calculate distance
    return (((Shooter.GOAL_HEIGHT_TO_FLOOR_INCHES - Shooter.LL_LENS_HEIGHT_INCHES) / Math.tan(angleToGoalRadians)
        + Shooter.EDGE_TO_CENTER_INCHES) / Shooter.METERS_TO_INCHES);
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

  public void setPipeline(int pipe) {
    pipelineNTE.setInteger(pipe); // dpl these looked like ints, not doubles
    if (pipe == 1) {
      enableLED();
    } else
      disableLED();
  }

  // switch between pipeline 0 and 1
  public void togglePipeline() {
    long pipe = pipelineNTE.getInteger(0);
    if (pipe == 0) {
      setPipeline(1);
      pipeline = 1;
    } else {
      setPipeline(0);
      pipeline = 0;
    }
  }

  public long getPipeline() {
    return pipeline;
  }

  public boolean valid() {
    return target;
  }

  public void log() {
    log_counter++;
    if (log_counter % 20 == 0) {
      NT_hasTarget.setBoolean(target);
      outputTv.setValue(target);
      outputTx.setDouble(x);
      // SmartDashboard.putBoolean("Target Lock", target);
    }
  }

}
