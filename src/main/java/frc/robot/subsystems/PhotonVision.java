// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {

  private PhotonCamera camera_global;
  private PhotonCamera camera_microsoft;
  private boolean hasAprilTargets;
  private boolean hasTapeTargets;
  private List<PhotonTrackedTarget> AprilTargets;
  private List<PhotonTrackedTarget> TapeTargets;
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
    camera_global = new PhotonCamera("Global_Shutter_Camera");
    camera_microsoft = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0,0,0)); //Cam mounted facing forward
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(camera_global, robotToCam));
    robotPoseEstimator = new RobotPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);
  }
  
  @Override
  public void periodic() {

    // Query the latest Apriltag result from PhotonVision
    var result_global = camera_global.getLatestResult();

    // Check if the latest result has any targets.
    hasAprilTargets = result_global.hasTargets(); 
    
    if(hasAprilTargets) {
      // Get a list of currently tracked targets.
      AprilTargets = result_global.getTargets();
      // Get the current best target.
      bestTarget = result_global.getBestTarget();

      // Get information from target.
      yaw = bestTarget.getYaw();
      pitch = bestTarget.getPitch();
      area = bestTarget.getArea();
      skew = bestTarget.getSkew();
      targetPose = bestTarget.getBestCameraToTarget();
      corners = bestTarget.getDetectedCorners();   

      // Get information from target.
      targetID = bestTarget.getFiducialId();
      previousPoseEstimate = currentPoseEstimate;
      currentPoseEstimate = getEstimatedGlobalPose(previousPoseEstimate).getFirst();
    }    
    
    SmartDashboard.putNumber("PV Pose X", currentPoseEstimate.getX());
    SmartDashboard.putNumber("PV Pose Y", currentPoseEstimate.getY());
    
    // Query the latest Retroreflective result from PhotonVision
    var result_microsoft = camera_microsoft.getLatestResult();

    // Check if the latest result has any targets.
    hasTapeTargets = result_microsoft.hasTargets(); 
    
    if(hasTapeTargets) {
      // Get a list of currently tracked targets.
      TapeTargets = result_microsoft.getTargets();
      // Get the current best target.
      bestTarget = result_microsoft.getBestTarget();

      // Get information from target.
      SmartDashboard.putNumber("# of PV targets", TapeTargets.size());
      SmartDashboard.putNumber("PV Yaw #1", TapeTargets.get(0).getYaw());
      SmartDashboard.putNumber("PV Area #1", TapeTargets.get(0).getArea());
      if (getNumberOfTapeTargets()>1){
        SmartDashboard.putNumber("PV Yaw #2", TapeTargets.get(1).getYaw());
        SmartDashboard.putNumber("PV Area #2", TapeTargets.get(1).getArea());
        SmartDashboard.putNumber("PV Largest Yaw", getLargestTapeTarget().getYaw());
        SmartDashboard.putNumber("PV 2nd Largest Yaw", getSecondLargestTapeTarget().getYaw());
      }

    }
  }
public boolean hasAprilTarget(){
  return hasAprilTargets;
}

public Pose2d getPoseEstimate(){
  return currentPoseEstimate;
}

public boolean hasTapeTarget(){
  return hasTapeTargets;
}

public int getNumberOfTapeTargets(){
  return TapeTargets.size();
}

public PhotonTrackedTarget getLargestTapeTarget(){
  TapeTargets.sort(new PhotonTrackedTargetComparator());
  return TapeTargets.get(0);
}

public PhotonTrackedTarget getSecondLargestTapeTarget(){
  TapeTargets.sort(new PhotonTrackedTargetComparator());
  return TapeTargets.get(1);
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

//make a comparator class to allow for list sorting
class PhotonTrackedTargetComparator implements Comparator<PhotonTrackedTarget> {
  @Override
  public int compare(PhotonTrackedTarget a, PhotonTrackedTarget b) {
    int temp = 0;
    if (a.getArea() > b.getArea()) temp = 1;
    if (a.getArea() < b.getArea()) temp = -1;
    return temp;
  }
}