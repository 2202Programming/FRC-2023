// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class PhotonVision extends SubsystemBase {

  // Cameras we have
  String camera1 = "Microsoft_LifeCam_HD-3000";
  String camera2 = "Arducam_OV9281_USB_Camera";

  // private PhotonCamera camera_global;
  private PhotonCamera camera_microsoft;
  private PhotonCamera camera_arducam;
  private boolean hasAprilTargets;
  private boolean hasTapeTargets;
  private List<PhotonTrackedTarget> AprilTargets;
  private List<PhotonTrackedTarget> TapeTargets;
  private PhotonTrackedTarget bestTarget; // was hiding access bug = new PhotonTrackedTarget();
  private double yaw;
  private double pitch;
  private double area;
  private double skew;
  private Transform3d targetPose;
  private List<TargetCorner> corners;
  private Transform3d robotToCam;
  private int targetID;
  // private double poseAmbiguity;
  // private Transform3d bestCameraToTarget;
  // private Transform3d alternateCameraToTarget;
  private AprilTagFieldLayout fieldLayout;
  private PhotonPoseEstimator robotPoseEstimator;
  private Pair<Pose2d, Double> currentPoseEstimate;
  private Pair<Pose2d, Double> previousPoseEstimate;

  public PhotonVision() {
    // build path to apriltag json file in deploy directory
    File deploy = Filesystem.getDeployDirectory();
    String path = deploy.getPath() + "/aprilTags.json";

    // load apriltag field layout
    try {
      fieldLayout = new AprilTagFieldLayout(path);
    } catch (Exception e) {
      System.out.println("***FAILED TO LOAD APRILTAG LIST***");
    }

    // Assemble the list of cameras & mount locations
    camera_microsoft = new PhotonCamera(camera1);
    camera_arducam = new PhotonCamera(camera2);
    robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.75), new Rotation3d(0, 0, 0)); // Cam mounted facing
                                                                                              // forward
    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(camera_arducam, robotToCam));

    // setup PhotonVision's pose estimator,
    robotPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, camera_arducam, robotToCam);
    previousPoseEstimate = new Pair<>(new Pose2d(), 0.0);
    currentPoseEstimate = new Pair<>(new Pose2d(), 0.0);
  }

  /* Used to set the starting postions either at Auto, or anytime the pose estimator needs a kick. */
  public void setInitialPose(Pair<Pose2d, Double> pose) {
    previousPoseEstimate = pose;
    currentPoseEstimate = pose;
  }

  @Override
  public void periodic() {
    // Query the latest Apriltag result from PhotonVision
    var result_global = camera_arducam.getLatestResult();

    // Check if the latest result has any targets.
    hasAprilTargets = result_global.hasTargets();

    if (hasAprilTargets) {
      // Get a list of currently tracked targets.
      AprilTargets = result_global.getTargets();
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

      // only targets we care about, sometime we see number outside expected range
      if (targetID < 9) {
        // guard, we can't estimate without a previousPoseEstimate
        if (previousPoseEstimate.getFirst() == null) return;

        currentPoseEstimate = getEstimatedGlobalPose(previousPoseEstimate.getFirst());

        // TODO: DPL not sure what is going on here, npe at random in sim mode, related
        // to Dr.J's issue?
        //don't update dash if there isn't a pose, photonVision return null if it doesn't have an estimate
        if (currentPoseEstimate.getFirst() != null) {
          SmartDashboard.putNumber("PV Pose X", currentPoseEstimate.getFirst().getX());
          SmartDashboard.putNumber("PV Pose Y", currentPoseEstimate.getFirst().getY());
        }
       previousPoseEstimate = currentPoseEstimate;       
      }
    }

    // Query the latest Retroreflective result from PhotonVision
    var result_microsoft = camera_microsoft.getLatestResult();

    // Check if the latest result has any targets.
    hasTapeTargets = result_microsoft.hasTargets();

    if (hasTapeTargets) {
      // Get a list of currently tracked targets.
      TapeTargets = result_microsoft.getTargets();
      // Get the current best target.
      bestTarget = result_microsoft.getBestTarget();

      // Get information from target.
      SmartDashboard.putNumber("# of PV targets", TapeTargets.size());
      SmartDashboard.putNumber("PV Yaw #1", TapeTargets.get(0).getYaw());
      SmartDashboard.putNumber("PV Area #1", TapeTargets.get(0).getArea());
      if (getNumberOfTapeTargets() > 1) {
        SmartDashboard.putNumber("PV Yaw #2", TapeTargets.get(1).getYaw());
        SmartDashboard.putNumber("PV Area #2", TapeTargets.get(1).getArea());
        SmartDashboard.putNumber("PV Largest Yaw", getLargestTapeTarget().getYaw());
        SmartDashboard.putNumber("PV 2nd Largest Yaw", getSecondLargestTapeTarget().getYaw());
      }

    }
  }

  public boolean hasAprilTarget() {
    return hasAprilTargets;
  }

  public List<PhotonTrackedTarget> getAprilTargets() {
    return AprilTargets;
  }

  public Transform3d getTarget3dPose() {
    return targetPose;
  }

  public List<TargetCorner> getCorners() {
    return corners;
  }

  // Pair<> getPoseEstimate() used by Swerve to add vision estimate
  public Pair<Pose2d, Double> getPoseEstimate() {
    Pose2d curr = currentPoseEstimate.getFirst();
    Pose2d curr_copy = new Pose2d(curr.getTranslation(), curr.getRotation());
    return new Pair<>(curr_copy, currentPoseEstimate.getSecond());
  }

  int getTargetID() {
    return targetID;
  }

  public boolean hasTapeTarget() {
    return hasTapeTargets;
  }

  public int getNumberOfTapeTargets() {
    return TapeTargets.size();
  }

  public PhotonTrackedTarget getLargestTapeTarget() {
    TapeTargets.sort(new PhotonTrackedTargetComparator());
    return TapeTargets.get(0);
  }

  public PhotonTrackedTarget getSecondLargestTapeTarget() {
    TapeTargets.sort(new PhotonTrackedTargetComparator());
    return TapeTargets.get(1);
  }

  public double getYaw() {
    return yaw;
  }

  public double getPitch() {
    return pitch;
  }

  public double getArea() {
    return area;
  }

  public double getSkew() {
    return skew;
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the
   *         field, and the time
   *         of the observation. Assumes a planar field and the robot is always
   *         firmly on the ground
   *         NOTE - APRIL TAG NEEDS TO BE IN 3D mode
   */
  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<EstimatedRobotPose> result = robotPoseEstimator.update();
    if (result.isPresent()) {
      return new Pair<Pose2d, Double>(result.get().estimatedPose.toPose2d(),
          currentTime - result.get().timestampSeconds);
    } else {
      return new Pair<Pose2d, Double>(null, 0.0);
    }

  }

  // make a comparator class to allow for list sorting
  class PhotonTrackedTargetComparator implements Comparator<PhotonTrackedTarget> {
    @Override
    public int compare(PhotonTrackedTarget a, PhotonTrackedTarget b) {
      return (a.getArea() > b.getArea()) ? 1 : -1;
    }
  }

  /* Simulation stuff */
  SimVisionSystem simVision1;
  SimVisionSystem simVision2;
  SimPhotonCamera sim_camera_global;
  SimPhotonCamera sim_camera_microsoft;
  SwerveDrivetrain sim_drivetrain; // ref the real one as WIP

  public void simulationPeriodic() {

    // Update PhotonVision based on our new robot position.
    simVision1.processFrame(sim_drivetrain.getPose());
    simVision2.processFrame(sim_drivetrain.getPose());

    /*
     * raw data mode - WIP
     * ArrayList<PhotonTrackedTarget> visibleTgtList = new
     * ArrayList<PhotonTrackedTarget>();
     * var tags = fieldLayout.getTags();
     * visibleTgtList.add(new PhotonTrackedTarget(yawDegrees, pitchDegrees, area,
     * skew, camToTargetTrans)); // Repeat for each target that you see
     * simCam.submitProcessedFrame(0.0, visibleTgtList);
     */
  }

  public void simulationInit() {
    sim_drivetrain = RobotContainer.RC().drivetrain;

    // target size in inches TODO: these are bogus fix them
    double targetWidth = Units.inchesToMeters(6.0); // Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); //
                                                    // meter
    double targetHeight = Units.inchesToMeters(6.0); // Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); //
                                                     // meters

    // Simulated Vision System.
    // Configure these to match your PhotonVision Camera,
    // pipeline, and LED setup.
    double camDiagFOV = 170.0; // degrees - assume wide-angle camera
    double camPitch = 0.0; // degrees
    double camHeightOffGround = .5; // meters
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 10; // square pixels

    simVision1 = new SimVisionSystem(
        camera1,
        camDiagFOV,
        new Transform3d(new Translation3d(0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0)),
        maxLEDRange,
        camResolutionWidth,
        camResolutionHeight,
        minTargetArea);

    simVision2 = new SimVisionSystem(
        camera2,
        camDiagFOV,
        new Transform3d(new Translation3d(0, 0, camHeightOffGround), new Rotation3d(0, camPitch, 0)),
        maxLEDRange,
        camResolutionWidth,
        camResolutionHeight,
        minTargetArea);

    // add all the tags into the PV simu
    var tags = fieldLayout.getTags();
    for (AprilTag aprilTag : tags) {
      var simVisionTarget = new SimVisionTarget(aprilTag.pose, targetWidth, targetHeight, aprilTag.ID);
      simVision1.addSimVisionTarget(simVisionTarget);
      simVision2.addSimVisionTarget(simVisionTarget);
    }

    // RAW WIP simCam = new SimPhotonCamera("MyCamera");
    // Assemble the list of cameras & mount locations
    /*
     * WIP Raw data mode
     * sim_camera_global = new SimPhotonCamera("Global_Shutter_Camera");
     * sim_camera_microsoft = new SimPhotonCamera("Microsoft_LifeCam_HD-3000");
     */
  }

}