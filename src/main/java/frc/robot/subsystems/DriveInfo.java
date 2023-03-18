// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.util.RobotSpecs;

/** Add your docs here. */
public final class DriveInfo {
    //Getting robotSpecs
    private static RobotSpecs robotSpecs;

  /**
   * Inversions account for rotations of the module relative to left or right side
   * of robot.
   * 
   * CANCoders are setup in Sensors and will have CCW= positve convention. Their
   * offsets are adjusted by their use in the drive train.
   */

    //Default for recent robots
    public static boolean kDriveMotorInvert_DL = false;
    public static boolean kAngleMotorInvert_DL = false;
    public static boolean kAngleCmdInvert_DL = false;

    public static boolean kDriveMotorInvert_DR = true;
    public static boolean kAngleMotorInvert_DR = false;
    public static boolean kAngleCmdInvert_DR = false;
    


    //Front Left
    public static boolean kDriveMotorInvert_FL;
    public static boolean kAngleMotorInvert_FL;
    public static boolean kAngleCmdInvert_FL;
    //Front Right
    public static boolean kDriveMotorInvert_FR;
    public static boolean kAngleMotorInvert_FR;
    public static boolean kAngleCmdInvert_FR;

    //Back Left
    public static boolean kDriveMotorInvert_BL;
    public static boolean kAngleMotorInvert_BL;
    public static boolean kAngleCmdInvert_BL;
    //Back Right
    public static boolean kDriveMotorInvert_BR;
    public static boolean kAngleMotorInvert_BR;
    public static boolean kAngleCmdInvert_BR;
    

    public static void SwerveInverseConstants() {
        robotSpecs = new RobotSpecs();
        switch (robotSpecs.myRobotName) {
            default:
                kDriveMotorInvert_FL = kDriveMotorInvert_DL;
                kAngleMotorInvert_FL = kAngleMotorInvert_DL;
                kAngleCmdInvert_FL = kAngleCmdInvert_DL;

                kDriveMotorInvert_FR = kDriveMotorInvert_DR;
                kAngleMotorInvert_FR = kDriveMotorInvert_DR;
                kAngleCmdInvert_FR = kAngleCmdInvert_DR;
                
                kDriveMotorInvert_BL = kDriveMotorInvert_DL;
                kAngleMotorInvert_BL = kAngleMotorInvert_DL;
                kAngleCmdInvert_BL = kAngleCmdInvert_DL;

                kDriveMotorInvert_BR = kDriveMotorInvert_DR;
                kAngleMotorInvert_BR = kDriveMotorInvert_DR;
                kAngleCmdInvert_BR = kAngleCmdInvert_DR;
        }
    }
}
