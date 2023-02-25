/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.NTStrings;
import frc.robot.util.ModMath;

public class Sensors_Subsystem extends SubsystemBase {
  public enum YawSensor {
    kNavX, kPigeon
  };

  /**
   * Creates a new Sensors_Subsystem.
   * 
   * This class will collect various robot sensors and ensure they are sampled and
   * filtered together.
   * 
   * Sensor sets include: NavX Chasis signals Lidar?
   * 
   * 
   */
  private NetworkTable table;
  private NetworkTable positionTable;
  
  private NetworkTableEntry nt_canUtilization;
  private NetworkTableEntry nt_canTxError;
  private NetworkTableEntry nt_canRxError;

  private NetworkTableEntry nt_cancoder_bl;
  private NetworkTableEntry nt_cancoder_br;
  private NetworkTableEntry nt_cancoder_fl;
  private NetworkTableEntry nt_cancoder_fr;
  private NetworkTableEntry nt_activeIMU;
  
  private NetworkTableEntry nt_yaw;
  private NetworkTableEntry nt_roll;
  private NetworkTableEntry nt_pitch;
  private NetworkTableEntry nt_rotation;


  // Sensors
  Pigeon2 m_pigeon;  
  double[] m_xyz_dps = new double[3]; // rotation rates [deg/s]

  public static class RotationPositions {
    public double back_left;
    public double back_right;
    public double front_left;
    public double front_right;
  }

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

  // CANCoders - monitor dt angles
  CANCoder rot_encoder_bl = init(new CANCoder(CAN.DT_BL_CANCODER));
  CANCoder rot_encoder_br = init(new CANCoder(CAN.DT_BR_CANCODER));
  CANCoder rot_encoder_fl = init(new CANCoder(CAN.DT_FL_CANCODER));
  CANCoder rot_encoder_fr = init(new CANCoder(CAN.DT_FR_CANCODER));

  // CAN monitoring
  CANStatus m_canStatus;

  // Simulation
  /// AHRS_GyroSim m_gyroSim;

  // measured values
  double m_roll;
  double m_pitch;
  double m_yaw;
  //offsets measured at power up
  final int BIAS_SAMPLES = 5; //[count]
  final double BIAS_DELAY = 0.2; //[s]
  double m_roll_bias;  //[deg]
  double m_pitch_bias; //[deg]
  double m_yaw_bias;   //[deg] measured, but not corrected for
  
  double m_yaw_d;
  final RotationPositions m_rot = new RotationPositions();

  // configurion setting
  YawSensor c_yaw_type = YawSensor.kPigeon;
  GyroStatus c_gryo_status = GyroStatus.UsingPigeon;

  final int NT_UPDATE_FRAME = 20;
  int log_counter = 0;

  //set this to true to default to pigeon
  public Pose2d autoStartPose;
  public Pose2d autoEndPose;

  public Sensors_Subsystem() {

    // alocate sensors
    m_canStatus = new CANStatus();
    m_pigeon = new Pigeon2(CAN.PIGEON_IMU_CAN);

    // set all the CanCoders to 100ms refresh rate to save the can bus
    rot_encoder_bl.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 100);
    rot_encoder_br.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 100);
    rot_encoder_fl.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 100);
    rot_encoder_fr.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 100);

    // setup network table
    table = NetworkTableInstance.getDefault().getTable("Sensors");
    positionTable = NetworkTableInstance.getDefault().getTable(NTStrings.NT_Name_Position);

    nt_canUtilization = table.getEntry("CanUtilization/value");
    nt_canRxError = table.getEntry("CanRxError");
    nt_canTxError = table.getEntry("CanTxError");

    // position angle encoders
    nt_cancoder_bl = table.getEntry("cc_bl");
    nt_cancoder_br = table.getEntry("cc_br");
    nt_cancoder_fl = table.getEntry("cc_fl");
    nt_cancoder_fr = table.getEntry("cc_fr");
    nt_activeIMU = table.getEntry("Active IMU");
    nt_yaw = table.getEntry("Active Yaw");
    nt_rotation = table.getEntry("Rotation");
    nt_pitch = positionTable.getEntry("Pitch");
    nt_roll = positionTable.getEntry("Roll");

    calibrate();
    log();
  }


  //@Override
  public void calibrate() {
    double roll_bias=0.0, pitch_bias=0.0, yaw_bias=0.0;
    for (int i=0; i< BIAS_SAMPLES; i++)
    {
      roll_bias +=  m_pigeon.getRoll();
      pitch_bias +=  m_pigeon.getPitch();
      yaw_bias += m_pigeon.getYaw();
      Timer.delay(BIAS_DELAY);
    }
    //save bias value to subtract from live measurements
    m_roll_bias =  roll_bias/(double)BIAS_SAMPLES;
    m_pitch_bias = pitch_bias/(double)BIAS_SAMPLES;
    m_yaw_bias = yaw_bias/(double)BIAS_SAMPLES;
  }

  @Override
  public void periodic() {  
    //CCW positive, inverting here to match all the NavX code previously written.
    m_yaw = ModMath.fmod360_2(-m_pigeon.getYaw()); 
    m_pitch = m_pigeon.getPitch() - m_pitch_bias;
    m_roll = m_pigeon.getRoll() - m_roll_bias;
    getRotationPositions(m_rot);
    m_pigeon.getRawGyro(m_xyz_dps);
    m_yaw_d = m_xyz_dps[2];

    log_counter++;
    log();
  }

  
  void setupSimulation() {
    // m_gyroSim_ahrs = new AHRS_GyroSim(m_ahrs);
    // m_gyroSim SimDevice
  }

  @Override
  public void simulationPeriodic() {
    // m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  public void log() {
    if ((log_counter % NT_UPDATE_FRAME)==0) {
     
      CANJNI.getCANStatus(m_canStatus);
      nt_canUtilization.setDouble(m_canStatus.percentBusUtilization);
      nt_canRxError.setNumber(m_canStatus.receiveErrorCount);
      nt_canTxError.setNumber(m_canStatus.transmitErrorCount);

      nt_cancoder_bl.setDouble(m_rot.back_left);
      nt_cancoder_br.setDouble(m_rot.back_right);
      nt_cancoder_fl.setDouble(m_rot.front_left);
      nt_cancoder_fr.setDouble(m_rot.front_right);
      nt_activeIMU.setString(c_gryo_status.toString());
     
      nt_yaw.setDouble(getYaw());
      nt_rotation.setDouble(getRotation2d().getDegrees());
      nt_roll.setDouble(getRoll());
      nt_pitch.setDouble(getPitch());
    }
  }


  // All accessors return values measured in the periodic()
  public double getRoll() {   
      return m_roll;
  }

  public double getPitch() {
      return m_pitch;
  }

  public double getPitchRate() {
    return m_xyz_dps[1];
  }

  public double getRollRate() {
    return m_xyz_dps[0];
  }

  public double getYawRate() {
    return m_xyz_dps[2];
  }

  /* not sure why these would be needed - Mr.L 2//22/2023
  public double getTotalTilt() {
    return Math.sqrt(Math.pow(getPitch(), 2) + Math.pow(getRoll(), 2));
  }

  public double getTotalTiltRate() {
    return Math.sqrt(Math.pow(getPitchRate(), 2) + Math.pow(getRollRate(), 2));
  }
  */

  /**
   * Return the heading of the robot in degrees.
   *
   * <p>
   * The angle is continuous, that is it will continue from 360 to 361 degrees.
   * This allows algorithms that wouldn't want to see a discontinuity in the gyro
   * output as it sweeps past from 360 to 0 on the second time around.
   *
   * <p>
   * The angle is expected to increase as the gyro turns clockwise when looked at
   * from the top. It needs to follow the NED axis convention.
   *
   * <p>
   * This heading is based on integration of the returned rate from the gyro.
   *
   * @return the current heading of the robot in degrees.
   */
    public double getYaw() {
      return m_yaw;
     }

  public void setYaw(double yawDegrees) {
    m_pigeon.setYaw(yawDegrees);
  }
  public void setYaw(Rotation2d rotation) {
    setYaw(rotation.getDegrees());
  }


/**
   * Return the heading of the robot as a {@link edu.wpi.first.math.geometry.Rotation2d}.
   *
   * <p>
   * The angle is continuous, that is it will continue from 360 to 361 degrees.
   * This allows
   * algorithms that wouldn't want to see a discontinuity in the gyro output as it
   * sweeps past from
   * 360 to 0 on the second time around.
   *
   * <p>
   * The angle is expected to increase as the gyro turns counterclockwise when
   * looked at from the
   * top. It needs to follow the NWU axis convention.
   *
   * <p>
   * This heading is based on integration of the returned rate from the gyro.
   *
   * @return the current heading of the robot as a {@link
   *         edu.wpi.first.math.geometry.Rotation2d}.
   */
  // @Override
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-m_yaw);  //note sign
  }

  /**
   * Return the rate of rotation of the gyro.
   *
   * <p>
   * The rate is based on the most recent reading of the gyro analog value
   *
   * <p>
   * The rate is expected to be positive as the gyro turns clockwise when looked
   * at from the top. It needs to follow the NED axis convention.
   *
   * @return the current rate in degrees per second
   */
  // @Override
  public double getRate() {
        return m_yaw_d;
  }

  public RotationPositions getRotationPositions(RotationPositions pos) {

    pos.back_left = rot_encoder_bl.getAbsolutePosition();
    pos.back_right = rot_encoder_br.getAbsolutePosition();
    pos.front_left = rot_encoder_fl.getAbsolutePosition();
    pos.front_right = rot_encoder_fr.getAbsolutePosition();

    return pos;
  }

  public CANCoder getCANCoder(EncoderID id) {
    switch (id) {
      case BackLeft:
        return rot_encoder_bl;
      case BackRight:
        return rot_encoder_br;
      case FrontLeft:
        return rot_encoder_fl;
      case FrontRight:
        return rot_encoder_fr;
      default:
        return null;
    }
  }

  /**
   * init() - setup cancoder the way we need them.
   * 
   * @param c
   * @return CANCoder just initialized
   */

  CANCoder init(CANCoder c) {
    c.configFactoryDefault(); // defaults to deg
    c.setPositionToAbsolute();
    c.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    c.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    c.clearStickyFaults();
    return c;
  }


  public void setAutoStartPose(Pose2d pose) {
    autoStartPose = new Pose2d(pose.getTranslation(), pose.getRotation());
    setYaw(pose.getRotation()); // set gyro to starting heading so it's in field coordinates.
    System.out.println("***Auto Start Pose set: " + pose);
  }

  public void setAutoEndPose(Pose2d pose) {
    autoEndPose = new Pose2d(pose.getTranslation(), pose.getRotation());

    // expected difference in heading from start of auto to end
    Rotation2d autoRot = autoStartPose.getRotation().minus(autoEndPose.getRotation());

    // gyro should power on at zero heading which would be our auto start position's
    // heading. So any angle off zero is the difference from start to end per the
    // gyro
    // not sure if this should be added or subtracted
    Rotation2d rotError = autoRot.minus(Rotation2d.fromDegrees(getYaw()));

    System.out.println("***Auto End Pose set: " + pose);
    System.out.println("***Rotation difference per Pose: " + autoRot.getDegrees());
    System.out.println("***Rotation difference per Gyro: " + getYaw());
    System.out.println("***Difference: " + rotError.getDegrees());

    /*
     * Idea below for correcting pose angle
     * Since before we run each path we set our pose to the starting position,
     * it's possible that our "true" heading (as determined by gryo) is not exactly
     * the starting heading of the new path.
     * The end of the prior path should be the start of the new path, but presumably
     * the rotation is not perfectly aligned (PID errors)
     * So with multiple paths this rotation error in pose may accumulate?
     */
    // RobotContainer.RC().drivetrain.resetAnglePose(pose.getRotation().minus(rotError));
    System.out.println("***Corrected End Pose: " + pose);

  }



}
