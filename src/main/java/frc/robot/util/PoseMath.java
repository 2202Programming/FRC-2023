// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Common Pose-based calculations */
public class PoseMath {

  private static double metersToFeet = 3.28084;

  /*
   * poseDistance(x) - returns distance between two poses
   *              
   * @param pose1
   * @param pose2  
   * @return double
  */
  public static double poseDistance(Pose2d pose1, Pose2d pose2){
          return Math.sqrt(
          (Math.pow(pose1.getX() - pose2.getX(),2)) +
          (Math.pow(pose1.getY() - pose2.getY(),2)));
  }

  // takes 2 positions, gives heading from current point to target (in degrees)
  public static Rotation2d getHeading2Target(Pose2d current, Pose2d target) {
    // from -PI to +PI
    return new Rotation2d(Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));
  }

  public static Pose2d convertMetersToFeet(Pose2d pose){
    return new Pose2d(pose.getX()*metersToFeet, pose.getY()*metersToFeet, pose.getRotation());
  }

  //return angle between virtual target and actual target, from point of view of robot pose
  public static Rotation2d angleVirtualTarget(Pose2d robotPose, Pose2d actualTarget, Pose2d virtualTarget)
  { //law of cosines γ=cos-1(a2+b2﹣c2)/2ab)
    Rotation2d offsetAngle = Rotation2d.fromDegrees(Math.acos(
      (Math.pow(poseDistance(robotPose, virtualTarget),2)
       + Math.pow(poseDistance(robotPose, actualTarget),2)
       - Math.pow(poseDistance(actualTarget, virtualTarget),2)) /
       (2*poseDistance(robotPose, virtualTarget)*poseDistance(robotPose, actualTarget))));

    return offsetAngle;
  }
}
