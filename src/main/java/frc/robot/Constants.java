/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  /**
   * CAN bus IDs
   * 
   * Please keep in order ID order
   * 
   */
  public static final class CAN {
    // CAN ID for non-motor devices
    public static final int PDP = 0; // this must be 0
    public static final int PCM1 = 1; // default ID for PCM

    // drive train CANCoders
    public static final int DT_BL_CANCODER = 28;
    public static final int DT_BR_CANCODER = 31;
    public static final int DT_FR_CANCODER = 30;
    public static final int DT_FL_CANCODER = 7;

    // Intake motor
    public static final int INTAKE_RIGHT_MTR = 19;
    public static final int INTAKE_LEFT_MTR = 18;

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
    public static final int ARM_LEFT_Motor = 13;
    public static final int ARM_RIGHT_Motor = 12;
    public static final int RIGHT_ELBOW = 14;
    public static final int LEFT_ELBOW = 15;

    // IMU
    public static final int PIGEON_IMU_CAN = 30;

    // Whether to burn flash or not
    public static final boolean BURN_FLASH = false; // swerve-mk3
  }

  // PWM assignments on the Rio
  public static final class PWM {
    public static final int RIGHT_WRIST = 0;
    public static final int LEFT_WRIST = 1;

    // dpl unused public static final int INTAKE = 0;
  }

  // Digital IO on the RIO
  public static final class DigitalIO {

  }

  public static final class AnalogIn {
    // public static final int MAGAZINE_ANGLE = 0;
  }

  // PWM assignments on the Rio
  public static final class PCM1 {
    // Double Solenoid
    public static final int INTAKE_UP_SOLENOID_PCM = 2; // test value
    public static final int INTAKE_DOWN_SOLENOID_PCM = 3; // test value
    public static final int POSITIONER_UP_SOLENOID_PCM = 0; // test value
    public static final int POSITIONER_DOWN_SOLENOID_PCM = 1; // test value

    // claw double Solenoid
    public static final int CLAW_FWD = 4;
    public static final int CLAW_REV = 5;

  }

  // if we use a second PCM
  public static final class PCM2 {
  }

  public static final class RobotPhysical {
  }

  // Magazine constants
  public static final class Magazine {
  }

  public static final class ArmSettings {

    public static final double MAX_VELOCITY = 1.0; // TODO: update me

  }

  // Intake Constants
  public static final class Intake {
    // PID values to get copied to the hardware
    public static PIDFController r_side_mtrPIDF = new PIDFController(1.0, 0.0, 0.0, 0.0);
    public static PIDFController l_side_mtrPIDF = new PIDFController(1.0, 0.0, 0.0, 0.0);
  }

  // Driver Preferences
  public static final class DriverPrefs {
    public static final double VelExpo = 0.3; // non-dim [0.0 - 1.0]
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
    public static final double kMaxSpeed = 12.0 * MperFT; // [m/s]
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
    public static final ChassisConfig chadBotChassisConfig = new ChassisConfig(MperFT * (21.516 / 12) / 2,
        MperFT * (24.87 / 12) / 2, 0.995, 99.5 / 1000.0, 12.8, 8.14);

    // For 2023 CompetitionBot TODO FIX ME
    public static final WheelOffsets compBotOffsets = new WheelOffsets(-175.60, -115.40, -162.15, 158.81);
    public static final ChassisConfig compBotChassisConfig = new ChassisConfig(
        MperFT * (21.516 / 12) / 2,
        MperFT * (24.87 / 12) / 2,
        0.999, // scale [] <= 1.0
        (99.5 / 1000.0), // wheel diameter[m]
        12.8,
        8.14);

  }

  public final static class NTStrings {
    public final static String NT_Name_Position = "Position";
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

  // class is for us to figure out our position on field when using inclinator
  // uses geometry to find out our x, y, and z direction in a function in terms of
  // the angle between the bars
  public class ArmGeometry {
    // Passive -> Extend, Active <- Retreat
    // Length of string -> motion -> position servo + 2 spark max + spring
    private double w1;
    private double w2;
    private double w3;
    // geometry-derived length in all three directions based on comp bot
    public static final double geoX = 175.60; // wheel and chassis height in x-direction
    public static final double geoY = 115.40; // wheel and chassis height in y-direction
    public static final double geoZ = 162.15; // wheel and chassis height in z-direction

    // length of string
    private double stringLength;
    // angle between the lengths
    double angle;

    // getters and setters
    public double getAngle() {
      return angle;
    }

    public double getGeoX() {
      return geoX;
    }

    public double getGeoY() {
      return geoY;
    }

    public double getGeoZ() {
      return geoZ;
    }

    public double getWidth1() {
      return w1;
    }

    public double getWidth2() {
      return w2;
    }

    public double getWidth3() {
      return w3;
    }

    public double getStringLength() {
      return stringLength;
    }

    // constructor
    public ArmGeometry(double xW1, double xW2, double xW3, double xAngle, double xStringLength) {
      w1 = xW1;
      w2 = xW2;
      w3 = xW3;
      angle = xAngle;
      stringLength = xStringLength;
    }

    double width = w1 + w2 + w3;
    // array to find the 3 lengths
    double lengthX = width * Math.sin(angle) + stringLength * Math.sin(angle) + geoX;
    double lengthY = geoY;
    double armLength = width * Math.cos(angle) + stringLength * Math.cos(angle) + geoZ;
    double[] lengths = { lengthX, lengthY, armLength };

    public double getLengthX() {
      return lengthX;
    }

    public double getLengthY() {
      return lengthY;
    }

    public double getLengthZ() {
      return armLength;
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
