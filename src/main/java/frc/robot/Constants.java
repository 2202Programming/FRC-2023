// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.PIDFController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double FT_PER_M = 3.28084;
  public static final double M_PER_FT = 1.0 / FT_PER_M;
  public static final double IN_PER_M = FT_PER_M * 12.0;
  public static final double M_PER_IN = 1.0 / IN_PER_M;

  /* 
   * ========================================
   * SECTION 1: Overall Robot Constants
   * ========================================
   */
  public static final double DELTA_T = 0.01; // TODO: back off rom 100hz?
  public static final double T_PERIOD = 0.02; // 50Hz for LL filters
  public final static String NT_NAME_POSITION = "Position";

  public static final class CAN {
    public static final boolean BURN_FLASH = false; //swerve-mk3

    // drive train CANCoders
    public static final int DT_BL_CANCODER = 28;
    public static final int DT_BR_CANCODER = 31;
    public static final int DT_FR_CANCODER = 30;
    public static final int DT_FL_CANCODER = 7;

    //IMU
    public static final int PIGEON_IMU_CAN = 60;

    // drive train drive / angle motors - sparkmax neo
    public static final int DT_FL_DRIVE = 20;
    public static final int DT_FL_ANGLE = 21;
    public static final int DT_BL_DRIVE = 22;
    public static final int DT_BL_ANGLE = 23;
    public static final int DT_BR_DRIVE = 24;
    public static final int DT_BR_ANGLE = 25;
    public static final int DT_FR_DRIVE = 26;
    public static final int DT_FR_ANGLE = 27;
  }
  

  /* 
   * ========================================
   * SECTION 2: Subsystem Constants
   * ========================================
   */

  public static final class Vision {
    // limelight distance constants
        // how many degrees back is your limelight rotated from perfectly vertical?
      public static final double LL_MOUNT_ANGLE_DEG = 28.0;
        // distance from the center of the Limelight lens to the floor
      public static final double LL_LENS_HEIGHT_INCHES = 27.0;
        // distance from the target to the floor
      public static final double GOAL_HEIGHT_TO_FLOOR_INCHES = 104.0;
        // adjustment from edge to centre of target
      public static final double EDGE_TO_CENTER_INCHES = 24.0;
   }

  public static final class Sensors {
  public enum EncoderID {
    BackLeft, BackRight, FrontLeft, FrontRight
  }

  public enum GyroStatus {
    UsingNavx("Navx"),
    UsingPigeon("Pigeon");
    private String name;
    private GyroStatus(String name) {
      this.name = name;
    }
    public String toString() {
      return name;
    }
  }
  }

   public static final class DriveTrain {
    // NT
    public static final String NT_NAME_DT = "DT"; // expose data under DriveTrain table
    public static final String NT_NAME_DC = "DC";


    //PIDs
    public static final PIDFController drivePIDF = new PIDFController(0.09*FT_PER_M, 0.0, 0.0, 0.08076*FT_PER_M);  
    public static final PIDFController anglePIDF = new PIDFController(0.01, 0.0, 0.0, 0.0); //maybe 1.0,0.0,0.1 from SDS sample code?

    // Constraints on speeds enforeced in DriveTrain
    public static final double kMaxSpeed = 12.0*M_PER_FT; // [m/s]
    public static final double kMaxAngularSpeed = 2*Math.PI; // [rad/s] 

    public enum DriveModes {
      robotCentric("Robot Centric"),
      fieldCentric("Field Centric"),
      intakeCentric("Intake Centric");
      private String name;
      private DriveModes(String name) {
        this.name = name;
      }
      public String toString() {
        return name;
      }
    }
   }

   /* 
   * ========================================
   * SECTION 3: Game Constants
   * ========================================
   */

   public static final class DriverControls {

    public enum Id {
      Driver(0), Operator(1), SwitchBoard(2), Phantom(3);

      public final int value;

        Id(int value) {
        this.value = value;
      }
    }

    public enum DriverMode {
      Arcade(0), Tank(1), XYRot(2);
  
      public final int value;
  
      DriverMode(int value) {
        this.value = value;
      }
    }

     //Driver Preferences
    public static final class DriverPrefs {
      public static final double VelExpo = 0.3;        // non-dim [0.0 - 1.0]
      public static final double RotationExpo = 0.9;   // non-dim [0.0 - 1.0]
      public static final double StickDeadzone = 0.05; // non-dim [0.0 - 1.0]
    }

    public static final class Autonomous {

      // path coordinates are in meters - utility only works in meters
      // TODO: update to 2023
      public static final Pose2d startPose1 = new Pose2d(7.67, 1.82, new Rotation2d(-180)); // Bottom, furthest from
      public static final Pose2d startPose2 = new Pose2d(6.86, 2.63, new Rotation2d(-180)); // Middle
      public static final Pose2d startPose3 = new Pose2d(6.7, 5.47, Rotation2d.fromDegrees(-180)); // Top
      public static final Pose2d hubPose = new Pose2d(8.27, 4.12, new Rotation2d(0)); // Center of Hub
      public static final Pose2d testStartPose = new Pose2d(5, 5, new Rotation2d(-180));
    }
  }

  /* 
   * ========================================
   * SECTION 3: Robot Specs
   * ========================================
   */


   public static final class ChassisConfig{

    // Kinematics model - wheel offsets from center of robot (0, 0)
    // Left Front given below, symmetry used for others 
    public final double XwheelOffset;  //meters, half of X wheelbase
    public final double YwheelOffset; //meters, half of Y wheelbase

    public final double wheelCorrectionFactor; //percent
    public final double wheelDiameter; //meters
    public final double kSteeringGR;   // [mo-turns to 1 angle wheel turn]
    public final double kDriveGR;      // [mo-turn to 1 drive wheel turn] 

    public ChassisConfig(double XwheelOffset, double YwheelOffset, double wheelCorrectionFactor, double wheelDiameter, double kSteeringGR,
      double kDriveGR){
        this.XwheelOffset = XwheelOffset;
        this.YwheelOffset = YwheelOffset;
        this.wheelCorrectionFactor = wheelCorrectionFactor;
        this.wheelDiameter = wheelDiameter * wheelCorrectionFactor;
        this.kSteeringGR = kSteeringGR;
        this.kDriveGR = kDriveGR;
    }
}

// CANCoder offsets for absolure calibration - stored in the magnet offset of the CC. [degrees]  
public static final class WheelOffsets{
    public final double CC_FL_OFFSET;
    public final double CC_BL_OFFSET;
    public final double CC_FR_OFFSET;
    public final double CC_BR_OFFSET;

    public WheelOffsets(double FL, double BL, double FR, double BR){
      this.CC_FL_OFFSET = FL;
      this.CC_BL_OFFSET = BL;
      this.CC_FR_OFFSET = FR;
      this.CC_BR_OFFSET = BR;
    }
}

  //FOR SWERVEBOT
  static final WheelOffsets swerveBotOffsets = 
    new WheelOffsets(-98.942, 91.33, -177.035, -28.215);
  static final ChassisConfig swerveBotChassisConfig =
    new ChassisConfig(10.5 / 12, 10.5 / 12, 0.995, 99.5/1000.0, 12.8, 8.14);
  static final SubsystemConfig swerveBotSubsystemConfig = 
    new SubsystemConfig(
      false, // intake
      false, // competition bot
      true,  // drivetrain
      true); // limelight (can be true even without one)

  public static final class SubsystemConfig{

    public final boolean HAS_INTAKE;
    public final boolean IS_COMPETITION_BOT;
    public final boolean HAS_DRIVETRAIN;
    public final boolean HAS_LIMELIGHT;

    public SubsystemConfig(boolean HAS_INTAKE, boolean IS_COMPETITION_BOT, boolean HAS_DRIVETRAIN, boolean HAS_LIMELIGHT) {
        this.HAS_INTAKE = HAS_INTAKE;
        this.IS_COMPETITION_BOT = IS_COMPETITION_BOT;
        this.HAS_DRIVETRAIN = HAS_DRIVETRAIN;
        this.HAS_LIMELIGHT = HAS_LIMELIGHT;
      }
    }

    public enum RobotSpecs {
      SWERVE_BOT(swerveBotSubsystemConfig, swerveBotChassisConfig, swerveBotOffsets),
      ;
  
      public final SubsystemConfig subsysConfig;
      public final ChassisConfig chassisConfig;
      public final WheelOffsets wheelOffsets;
      private RobotSpecs(SubsystemConfig subsysConfig, ChassisConfig chassisConfig, WheelOffsets wheelOffsets) {
        this.subsysConfig = subsysConfig;
        this.chassisConfig = chassisConfig;
        this.wheelOffsets = wheelOffsets;
      }
    }

    public static final Map<String, RobotSpecs> keysAndBots = new HashMap<String, RobotSpecs>(Map.ofEntries(
      Map.entry("031b7511", RobotSpecs.SWERVE_BOT),
      Map.entry("sim", RobotSpecs.SWERVE_BOT)                     //simulation won't have a S/N 
    ));
}
