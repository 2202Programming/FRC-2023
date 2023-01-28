// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
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
  private Transform3d robotToCam;
  private int targetID;
  private double poseAmbiguity;
  private Transform3d bestCameraToTarget;
  private Transform3d alternateCameraToTarget;
  private AprilTagFieldLayout fieldLayout;
  private RobotPoseEstimator robotPoseEstimator;
  private Pose2d currentPoseEstimate = new Pose2d();
  private Pose2d previousPoseEstimate = new Pose2d();

  public PhotonVision() {
    // table = NetworkTableInstance.getDefault().getTable("photonvision");
    // targetPixelsX = table.getEntry("Global_Shutter_Camera/targetPixelsX");
    // targetPixelsY = table.getEntry("Global_Shutter_Camera/targetPixelsY");
    // targetPixelsArea = table.getEntry("Global_Shutter_Camera/targetArea");
    // hasTarget = table.getEntry("Global_Shutter_Camera/hasTarget");

    //build path to apriltag json file in deploy directory
    File deploy = Filesystem.getDeployDirectory();
    String path = deploy.getPath() + "/aprilTags.json";

    //load apriltag field layout
    try {
      fieldLayout = new AprilTagFieldLayout(path);
    } catch (Exception e) {
      System.out.println("***FAILED TO LOAD APRILTAG LIST***");
    };

    // Assemble the list of cameras & mount locations
    camera = new PhotonCamera("Global_Shutter_Camera");
    robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(camera, robotToCam));
    robotPoseEstimator = new RobotPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);
    
  
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

    double CAMERA_HEIGHT_METERS = 0.0;
    double TARGET_HEIGHT_METERS = 0.0;
    double CAMERA_PITCH_RADIANS = 0.0;

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

      // double range =
      // PhotonUtils.calculateDistanceToTargetMeters(
      //         CAMERA_HEIGHT_METERS,
      //         TARGET_HEIGHT_METERS,
      //         CAMERA_PITCH_RADIANS,
      //         Units.degreesToRadians(result.getBestTarget().getPitch()));

      // SmartDashboard.putNumber("Range", range);

      previousPoseEstimate = currentPoseEstimate;
      currentPoseEstimate = getEstimatedGlobalPose(previousPoseEstimate).getFirst();
      SmartDashboard.putNumber("latency", getEstimatedGlobalPose(previousPoseEstimate).getSecond());

    }    
    

    SmartDashboard.putBoolean("hasTargets", hasTargets);
    SmartDashboard.putNumber("yaw", yaw);
    SmartDashboard.putNumber("pitch", pitch);
    SmartDashboard.putNumber("area", area);
    SmartDashboard.putNumber("skew", skew);
    SmartDashboard.putNumber("targetID", targetID);
    SmartDashboard.putNumber("PV Pose X", currentPoseEstimate.getX());
    SmartDashboard.putNumber("PV Pose Y", currentPoseEstimate.getY());
    

  }

      /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
   *     of the observation. Assumes a planar field and the robot is always firmly on the ground
   *  NOTE - APRIL TAG NEEDS TO BE IN 3D mode
   */
  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
        return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
    } else {
        return new Pair<Pose2d, Double>(null, 0.0);
    }
  }
}
