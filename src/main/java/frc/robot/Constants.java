/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.PIDFController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * 
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class SubsystemConfig {

    public final boolean HAS_INTAKE;
    public final boolean HAS_SHOOTER;
    public final boolean IS_COMPETITION_BOT;
    public final boolean HAS_MAGAZINE;
    public final boolean HAS_CLIMBER;
    public final boolean HAS_POSITIONER;
    public final boolean HAS_DRIVETRAIN;
    public final boolean HAS_LIMELIGHT;

    public SubsystemConfig(boolean HAS_INTAKE, boolean HAS_SHOOTER, boolean IS_COMPETITION_BOT, boolean HAS_MAGAZINE,
        boolean HAS_CLIMBER,
        boolean HAS_POSITIONER, boolean HAS_DRIVETRAIN, boolean HAS_LIMELIGHT) {
      this.HAS_INTAKE = HAS_INTAKE;
      this.HAS_SHOOTER = HAS_SHOOTER;
      this.IS_COMPETITION_BOT = IS_COMPETITION_BOT;
      this.HAS_MAGAZINE = HAS_MAGAZINE;
      this.HAS_CLIMBER = HAS_CLIMBER;
      this.HAS_POSITIONER = HAS_POSITIONER;
      this.HAS_DRIVETRAIN = HAS_DRIVETRAIN;
      this.HAS_LIMELIGHT = HAS_LIMELIGHT;
    }
  }

  public static final SubsystemConfig swerveBotSubsystemConfig = new SubsystemConfig(false,false, false, false, false,      false, true, true);
  public static final SubsystemConfig chadBotSubsystemConfig = new SubsystemConfig(true, true, false, true, true, true, true, true);
  //2023 competitionbot
  public static final SubsystemConfig compBotSubsystemConfig = new SubsystemConfig(true,
      false,
      true,
      false,
      false,
      false,
      true,
      true);

  // Handy feet to meters
  public static final double FTperM = 3.28084;
  public static final double MperFT = 1.0 / FTperM;

  public static final double DT = 0.02; // 20ms framerate 50Hz
  public static final double Tperiod = 0.02; // framerate period 20ms, 50Hz
  public static final int NEO_COUNTS_PER_REVOLUTION = 42;

  public static final class Autonomous {

    // path coordinates are in meters - utility only works in meters
    public static final Pose2d startPose1 = new Pose2d(7.67, 1.82, new Rotation2d(-180)); // Bottom, furthest from
                                                                                          // terminal
    public static final Pose2d startPose2 = new Pose2d(6.86, 2.63, new Rotation2d(-180)); // Middle
    public static final Pose2d startPose3 = new Pose2d(6.7, 5.47, Rotation2d.fromDegrees(-180)); // Top
    public static final Pose2d hubPose = new Pose2d(8.27, 4.12, new Rotation2d(0)); // Center of Hub
    public static final Pose2d testStartPose = new Pose2d(5, 5, new Rotation2d(-180));
  }

  public static class PowerOnPos {
    public static final double arm = 0.0;     //[cm]
    public static final double elbow = -10.0;   //[deg]
    public static final double wrist = -60.0; //[deg]
    public static final double rotate = 0.0;   //[deg]
  }

  public static class ConePickup {
    public static final double armLength = 20.0; //[cm] not the final number
    public static final double elbowAngle = 90.0;  //[degrees]
    public static final double wristAngle = 0.0; //[degrees] not final
  }

  /**
   * CAN bus IDs
   * 
   * Please keep in order ID order
   * 
   */
  public static final class CAN {
    // CAN ID for non-motor devices

    //DL PROBLEM TO WORK WITH OTHER BOTS
    public static final int PDP = 1; //for rev
    public static final int PCM1 = 2; //for rev

    // drive train CANCoders
    public static final int DT_BL_CANCODER = 28;
    public static final int DT_BR_CANCODER = 31;
    public static final int DT_FR_CANCODER = 30;
    public static final int DT_FL_CANCODER = 7;

    // Intake motor
    public static final int INTAKE_RIGHT_MTR = 19;
    public static final int INTAKE_LEFT_MTR = 18;
    public static final int CARWASH_RIGHT_MTR = 11;
    public static final int CARWASH_LEFT_MTR = 10;

    // drive train drive / angle motors - sparkmax neo
    public static final int DT_FL_DRIVE = 20;
    public static final int DT_FL_ANGLE = 21;
    public static final int DT_BL_DRIVE = 22;
    public static final int DT_BL_ANGLE = 23;
    public static final int DT_BR_DRIVE = 24;
    public static final int DT_BR_ANGLE = 25;
    public static final int DT_FR_DRIVE = 26;
    public static final int DT_FR_ANGLE = 27;

    // Climber Arms
    public static final int CMB_LEFT_Extend = 34;
    public static final int CMB_RIGHT_Extend = 35;
    public static final int CMB_LEFT_Rotate = 36;
    public static final int CMB_RIGHT_Rotate = 37;

    // Arms
    public static final int ARM_RIGHT_Motor = 12;
    public static final int ARM_LEFT_Motor = 13;
    public static final int ELBOW_Motor = 14;
    public static final int WRIST_Motor = 15;
    
    //Claw
    public static final int CLAW_WHEEL_MOTOR = 16;
    public static final int CLAW_ROTATE_MOTOR = 17;
    
    // IMU
    public static final int PIGEON_IMU_CAN = 60;

    // Whether to burn flash or not
    public static final boolean BURN_FLASH = false; // swerve-mk3
    
  }

  // PWM assignments on the Rio
  public static final class PWM {
    // dpl unused public static final int INTAKE = 0;
  }

  // Digital IO on the RIO
  public static final class DigitalIO {
    public static final int IntakeLightGate = 0;
    public static final int ClawLightgate = 1;
  }

  public static final class AnalogIn {
    // public static final int MAGAZINE_ANGLE = 0;
  }

  // PWM assignments on the Rio
  public static final class PCM1 {
    // Double Solenoid
    public static final int INTAKE_UP_SOLENOID_PCM = 1; // test value
    public static final int INTAKE_DOWN_SOLENOID_PCM = 0; // test value

    // claw double Solenoid
    public static final int CLAW_FWD = 2;
    public static final int CLAW_REV = 3;

  }

  // if we use a second PCM
  public static final class PCM2 {
  }


  // Intake Constants
  public static final class Intake_Constants {
    // PID values to get copied to the hardware
    public static PIDFController r_side_mtrPIDF = new PIDFController(1.0, 0.0, 0.0, 0.0);
    public static PIDFController l_side_mtrPIDF = new PIDFController(1.0, 0.0, 0.0, 0.0);
  }

  // Driver Preferences
  public static final class DriverPrefs {
    public static final double VelExpo = 0.7; // non-dim [0.0 - 1.0]
    public static final double RotationExpo = 0.9; // non-dim [0.0 - 1.0]
    public static final double StickDeadzone = 0.05; // non-dim [0.0 - 1.0]
  }

  public static final class ChassisConfig {

    // Kinematics model - wheel offsets from center of robot (0, 0)
    // Left Front given below, symmetry used for others
    public final double XwheelOffset; // meters, half of X wheelbase
    public final double YwheelOffset; // meters, half of Y wheelbase

    public final double wheelCorrectionFactor; // percent
    public final double wheelDiameter; // meters
    public final double kSteeringGR; // [mo-turns to 1 angle wheel turn]
    public final double kDriveGR; // [mo-turn to 1 drive wheel turn]

    public ChassisConfig(double XwheelOffset, double YwheelOffset, double wheelCorrectionFactor, double wheelDiameter,
        double kSteeringGR,
        double kDriveGR) {
      this.XwheelOffset = XwheelOffset;
      this.YwheelOffset = YwheelOffset;
      this.wheelCorrectionFactor = wheelCorrectionFactor;
      this.wheelDiameter = wheelDiameter * wheelCorrectionFactor;
      this.kSteeringGR = kSteeringGR;
      this.kDriveGR = kDriveGR;
    }
  }

  // CANCoder offsets for absolure calibration - stored in the magnet offset of
  // the CC. [degrees]
  public static final class WheelOffsets {
    public final double CC_FL_OFFSET;
    public final double CC_BL_OFFSET;
    public final double CC_FR_OFFSET;
    public final double CC_BR_OFFSET;

    public WheelOffsets(double FL, double BL, double FR, double BR) {
      this.CC_FL_OFFSET = FL;
      this.CC_BL_OFFSET = BL;
      this.CC_FR_OFFSET = FR;
      this.CC_BR_OFFSET = BR;
    }
  }

  public static final class DriveTrain {
    // motor constraints
    public static final double motorMaxRPM = 5600; // motor limit

    // Constraints on speeds enforeced in DriveTrain
    public static final double kMaxSpeed = 20.0 * MperFT; // [m/s]
    public static final double kMaxAngularSpeed = 2 * Math.PI; // [rad/s]

    /****
     * ### REMINDER - enable these once we have basics working
     * // Other constraints
     * public static final int smartCurrentMax = 60; //amps in SparkMax, max setting
     * public static final int smartCurrentLimit = 35; //amps in SparkMax, inital
     * setting
     */
    // Acceleration limits
    /// public static final double slewRateMax = 2; //sec limits adjusting slewrate
    // public static final boolean safetyEnabled = false;

    // SmartMax PID values [kp, ki, kd, kff] - these get sent to hardware controller
    // DEBUG - SET FF first for drive, then add KP

    // public static final PIDFController drivePIDF = new
    // PIDFController(0.09*MperFT, 0.0, 0.0, 0.08076*MperFT);
    public static final PIDFController drivePIDF = new PIDFController(0.09 * FTperM, 0.0, 0.0, 0.08076 * FTperM);
    public static final PIDFController anglePIDF = new PIDFController(0.01, 0.0, 0.0, 0.0); // maybe 1.0,0.0,0.1 from
                                                                                            // SDS sample code?

    // FOR SWERVEBOT, aka Tim 2.0
    public static final WheelOffsets swerveBotOffsets = new WheelOffsets(-98.942, 91.33, -177.035, -28.215);
    public static final ChassisConfig swerveBotChassisConfig = new ChassisConfig(10.5 / 12, 10.5 / 12, 0.995,
        99.5 / 1000.0, 12.8, 8.14);

    // FOR 2022 Chad Bot - degrees
    public static final WheelOffsets chadBotOffsets = new WheelOffsets(-175.60, -115.40, -162.15, 158.81);
    public static final ChassisConfig chadBotChassisConfig = new ChassisConfig(MperFT * (21.516 / 12.0) / 2.0,
        MperFT * (24.87 / 12) / 2, 0.995, 99.5 / 1000.0, 12.8, 8.14);

    // For 2023 CompetitionBot
    public static final WheelOffsets compBotOffsets = new WheelOffsets(129.03, -83.94, -57.83, 139.38);
    public static final ChassisConfig compBotChassisConfig = new ChassisConfig(
        MperFT * (23.5 / 12.0) / 2.0, //based on CAD in reference_links
        MperFT * (19.5 / 12.0) / 2.0, //based on CAD in reference_links
        0.999, // scale [] <= 1.0
        MperFT * (4.0/12.0), // wheel diameter[m] Comp bot is 4" wheels
        12.8, //confirmed with vince
        8.14); //confirmed with vince

  }

  public final static class NTStrings {
    public final static String NT_Name_Position = "Position";
  }

  public final static class FieldPoses {
    public final static Pose2d blueScorePose1 = new Pose2d(new Translation2d(2.0,0.50), Rotation2d.fromDegrees(180)); //lowest scoring position in Y
    public final static Pose2d blueScorePose2 = new Pose2d(new Translation2d(2.0,1.05), Rotation2d.fromDegrees(180));
    public final static Pose2d blueScorePose3 = new Pose2d(new Translation2d(2.0,1.60), Rotation2d.fromDegrees(180));
    public final static Pose2d blueScorePose4 = new Pose2d(new Translation2d(2.0,2.15), Rotation2d.fromDegrees(180));
    public final static Pose2d blueScorePose5 = new Pose2d(new Translation2d(2.0,2.70), Rotation2d.fromDegrees(180));
    public final static Pose2d blueScorePose6 = new Pose2d(new Translation2d(2.0,3.30), Rotation2d.fromDegrees(180));
    public final static Pose2d blueScorePose7 = new Pose2d(new Translation2d(2.0,3.80), Rotation2d.fromDegrees(180));
    public final static Pose2d blueScorePose8 = new Pose2d(new Translation2d(2.0,4.40), Rotation2d.fromDegrees(180));
    public final static Pose2d blueScorePose9 = new Pose2d(new Translation2d(2.0,5.00), Rotation2d.fromDegrees(180)); //highest scoring position in Y

    //These are fake red scoring positions for testing in black box
    // public final static Pose2d redScorePose1 = new Pose2d(new Translation2d(7.0,0.50), Rotation2d.fromDegrees(0)); //lowest scoring position in Y.  
    // public final static Pose2d redScorePose2 = new Pose2d(new Translation2d(7.0,1.05), Rotation2d.fromDegrees(0));
    // public final static Pose2d redScorePose3 = new Pose2d(new Translation2d(7.0,1.60), Rotation2d.fromDegrees(0));
    // public final static Pose2d redScorePose4 = new Pose2d(new Translation2d(7.0,2.15), Rotation2d.fromDegrees(0));
    // public final static Pose2d redScorePose5 = new Pose2d(new Translation2d(7.0,2.70), Rotation2d.fromDegrees(0));
    // public final static Pose2d redScorePose6 = new Pose2d(new Translation2d(7.0,3.30), Rotation2d.fromDegrees(0));
    // public final static Pose2d redScorePose7 = new Pose2d(new Translation2d(7.0,3.80), Rotation2d.fromDegrees(0));
    // public final static Pose2d redScorePose8 = new Pose2d(new Translation2d(7.0,4.40), Rotation2d.fromDegrees(0));
    // public final static Pose2d redScorePose9 = new Pose2d(new Translation2d(7.0,5.00), Rotation2d.fromDegrees(0)); //highest scoring position in Y

    //JR estimate of real red scoring positions
    public final static Pose2d redScorePose1 = new Pose2d(new Translation2d(14.7,0.50), Rotation2d.fromDegrees(0)); //lowest scoring position in Y.  
    public final static Pose2d redScorePose2 = new Pose2d(new Translation2d(14.7,1.05), Rotation2d.fromDegrees(0));
    public final static Pose2d redScorePose3 = new Pose2d(new Translation2d(14.7,1.60), Rotation2d.fromDegrees(0));
    public final static Pose2d redScorePose4 = new Pose2d(new Translation2d(14.7,2.15), Rotation2d.fromDegrees(0));
    public final static Pose2d redScorePose5 = new Pose2d(new Translation2d(14.7,2.70), Rotation2d.fromDegrees(0));
    public final static Pose2d redScorePose6 = new Pose2d(new Translation2d(14.7,3.30), Rotation2d.fromDegrees(0));
    public final static Pose2d redScorePose7 = new Pose2d(new Translation2d(14.7,3.80), Rotation2d.fromDegrees(0));
    public final static Pose2d redScorePose8 = new Pose2d(new Translation2d(14.7,4.40), Rotation2d.fromDegrees(0));
    public final static Pose2d redScorePose9 = new Pose2d(new Translation2d(14.7,5.00), Rotation2d.fromDegrees(0)); //highest scoring position in Y
  
    public final static Pose2d[][] blueScorePoses =  {{blueScorePose1,blueScorePose2,blueScorePose3},
                                                     {blueScorePose4,blueScorePose5,blueScorePose6},
                                                     {blueScorePose7,blueScorePose8,blueScorePose9}};

    public final static Pose2d[][] redScorePoses =   {{redScorePose1,redScorePose2,redScorePose3},
                                                     {redScorePose4,redScorePose5,redScorePose6},
                                                     {redScorePose7,redScorePose8,redScorePose9}};

    public final static Pose2d bluePickupPoseCenter = new Pose2d(new Translation2d(15.50, 6.75), Rotation2d.fromDegrees(0));
    public final static Pose2d bluePickupPoseRight = new Pose2d(new Translation2d(15.50, 7.35), Rotation2d.fromDegrees(0));
    public final static Pose2d bluePickupPoseLeft = new Pose2d(new Translation2d(15.50, 6.00), Rotation2d.fromDegrees(0));
    public final static Pose2d redPickupPoseCenter = new Pose2d(new Translation2d(1.00, 6.75), Rotation2d.fromDegrees(0));
    public final static Pose2d redPickupPoseLeft = new Pose2d(new Translation2d(1.00, 7.35), Rotation2d.fromDegrees(0));
    public final static Pose2d redPickupPoseRight = new Pose2d(new Translation2d(1.00, 6.00), Rotation2d.fromDegrees(0));

    public final static Pose2d[] bluePickupPoses = {bluePickupPoseLeft, bluePickupPoseCenter, bluePickupPoseRight};
    public final static Pose2d[] redPickupPoses = {redPickupPoseLeft, redPickupPoseCenter, redPickupPoseRight};

  }

  public final static class MagazineSettings {
    public final static double defaultFrontIntakeSpeed = 0.5;
    public final static double defaultSideIntakeSpeed = 0.3;
    public final static double defaultMagazineSpeed = 1.0;
  }

  public static final class Shooter {
    public static final double DefaultRPMTolerance = .05; // percent of RPM

    // Power Cell info
    // public static final double PowerCellMass = 3.0 / 16.0; // lbs
    public static final double PCNominalRadius = 10 / 2.0 / 12.0; // feet - power cell
    public static final double PCEffectiveRadius = 8 / 2.0 / 12.0; // feet - compressed radius

    public static final double shortVelocity = 40;
    public static final double shortMediumVelocity = 44;
    public static final double mediumVelocity = 50;
    public static final double longVelocity = 60;
    public static final double autoVelocity = 46;

    // limelight distance constants
    // how many degrees back is your limelight rotated from perfectly vertical?
    public static final double LL_MOUNT_ANGLE_DEG = 28.0;
    // distance from the center of the Limelight lens to the floor
    public static final double LL_LENS_HEIGHT_INCHES = 27.0;
    // distance from the target to the floor
    public static final double GOAL_HEIGHT_TO_FLOOR_INCHES = 104.0;
    // adjustment from edge to centre of target
    public static final double EDGE_TO_CENTER_INCHES = 24.0;
    // adjustment factor
    public static final double METERS_TO_INCHES = 39.37;

    public static final double limelight_default_p = 7; // was 4 // [deg/s / deg-err]
    public static final double limelight_default_i = 0.1;
    public static final double limelight_default_d = 0.1;

    // constraints
    public static final double kMaxFPS = 80; // max FPS
    public static final double maxLongRage = 8; // maximum range in long distance shooting mode
    public static final double minLongRange = 1.8; // minimum range in long distance shooting mode
    public static final double maxShortRange = 2; // maximum range in short distance shooting mode
    public static final double degPerPixel = 59.6 / 320; // limelight conversion
    public static final double angleErrorTolerance = 2.0; // [deg] allowed angle error to shoot in guided shooting modes
    public static final double angleVelErrorTolerance = 1.0; // [deg/s] allowed angle error to shoot in guided shooting
                                                             // modes
  }

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
  }

  //Enums for placement, used in Place.java command
  public enum HorizontalScoringLane {
    Left,
    Right,
    Center
  }
  public enum VerticalScoringLane{
    Middle,
    Top,
    Bottom
  }
  public enum ScoringBlock {
    Left(0), Center(1), Right(2);

    public final int value;

    ScoringBlock(int value) {
      this.value = value;
    }
  }    
}