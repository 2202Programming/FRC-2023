// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */

  // private NetworkTable table;
  // private NetworkTableEntry targetPixelsX;
  // private NetworkTableEntry targetPixelsY;
  // private NetworkTableEntry targetPixelsArea;
  // private NetworkTableEntry hasTarget;
  
  // private double m_targetPixelsX;
  // private double m_targetPixelsY;
  // private double m_targetPixelsArea;
  // private boolean m_hasTarget;

  private PhotonCamera camera;
  private boolean hasTargets;
  private List<PhotonTrackedTarget> targets;
  private PhotonTrackedTarget bestTarget;
  private double yaw;
  private double pitch;
  private double area;
  private double skew;
  private Transform3d targetPose;
  private List<TargetCorner> corners;

  private int targetID;
  private double poseAmbiguity;
  private Transform3d bestCameraToTarget;
  private Transform3d alternateCameraToTarget;

  private AprilTagFieldLayout fieldLayout;

  public PhotonVision() {
    // table = NetworkTableInstance.getDefault().getTable("photonvision");
    // targetPixelsX = table.getEntry("Global_Shutter_Camera/targetPixelsX");
    // targetPixelsY = table.getEntry("Global_Shutter_Camera/targetPixelsY");
    // targetPixelsArea = table.getEntry("Global_Shutter_Camera/targetArea");
    // hasTarget = table.getEntry("Global_Shutter_Camera/hasTarget");

    camera = new PhotonCamera("Global_Shutter_Camera");
    
    //build path to apriltag json file in deploy directory
    File deploy = Filesystem.getDeployDirectory();
    String path = deploy.getPath() + "/aprilTags.json";
    
    //load apriltag field layout
    try {
      fieldLayout = new AprilTagFieldLayout(path);
    } catch (Exception e) {
      System.out.println("***FAILED TO LOAD APRILTAG LIST***");
    };
    
  
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // m_targetPixelsX = targetPixelsX.getDouble(0);
    // m_targetPixelsY = targetPixelsY.getDouble(0);
    // m_targetPixelsArea = targetPixelsArea.getDouble(0);
    // m_hasTarget = hasTarget.getBoolean(false);

    // SmartDashboard.putNumber("targetX", m_targetPixelsX);
    // SmartDashboard.putNumber("targetY", m_targetPixelsY);
    // SmartDashboard.putNumber("targetArea", m_targetPixelsArea);
    // SmartDashboard.putBoolean("hasTarget", m_hasTarget);

    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();

    // Check if the latest result has any targets.
    hasTargets = result.hasTargets(); 
    
    if(hasTargets) {
      // Get a list of currently tracked targets.
      targets = result.getTargets();
      // Get the current best target.
      bestTarget = result.getBestTarget();

      // Get information from target.
      yaw = bestTarget.getYaw();
      pitch = bestTarget.getPitch();
      area = bestTarget.getArea();
      skew = bestTarget.getSkew();
      targetPose = bestTarget.getBestCameraToTarget();
      corners = bestTarget.getDetectedCorners();   

      // Get information from target.
      targetID = bestTarget.getFiducialId();
      double poseAmbiguity = bestTarget.getPoseAmbiguity();
      Transform3d bestCameraToTarget = bestTarget.getBestCameraToTarget();
      Transform3d alternateCameraToTarget = bestTarget.getAlternateCameraToTarget();

    }    
    

    SmartDashboard.putBoolean("hasTargets", hasTargets);
    SmartDashboard.putNumber("yaw", yaw);
    SmartDashboard.putNumber("pitch", pitch);
    SmartDashboard.putNumber("area", area);
    SmartDashboard.putNumber("skew", skew);
    SmartDashboard.putNumber("targetID", targetID);
    

  }
}
