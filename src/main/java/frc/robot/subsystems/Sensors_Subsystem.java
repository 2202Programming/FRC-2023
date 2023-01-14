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

  final double Kgyro = -1.0; // ccw is positive, just like geometry class

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

  static final byte update_hz = 100;
  // Sensors
  Pigeon2 m_pigeon;
  
  double[] m_xyz_dps = new double[3];     //rotation rates [deg/s]

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
  double m_yaw_d;
  final RotationPositions m_rot = new RotationPositions();

  // configurion setting
  YawSensor c_yaw_type = YawSensor.kPigeon;
  GyroStatus c_gryo_status = GyroStatus.UsingPigeon;

  double log_counter = 0;
  //set this to true to default to pigeon
  public Pose2d autoStartPose;
  public Pose2d autoEndPose;

  public Sensors_Subsystem() {

    // alocate sensors
    m_canStatus = new CANStatus();

    // create devices and interface access, use interface where possible
    // m_gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    // m_gyro_ahrs = m_ahrs = new AHRS(SPI.Port.kMXP, update_hz);
    // m_ahrs.enableLogging(true);

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

    //allow Sensors to calibrate in it's own thread
    new Thread(()->{
      try {
        Thread.sleep(1000); // wait a second so gyro is done booting before calibrating.
        calibrate();
      } catch (Exception e) {
      }
    }).start();

    log(20);
  }


  //@Override
  public void calibrate() {

    //TODO: measure angles, average and remove offsets.

    reset();
  }

  @Override
  public void periodic() {  
    m_yaw = ModMath.fmod360_2(-m_pigeon.getYaw()); //CCW positive, inverting here to match all the NavX code previously written.
    m_pitch = m_pigeon.getPitch();
    m_roll = m_pigeon.getRoll();
    getRotationPositions(m_rot);
    m_pigeon.getRawGyro(m_xyz_dps);
    m_yaw_d = m_xyz_dps[2];

    log(20);
  }

  

  void setupSimulation() {
    // m_gyroSim_ahrs = new AHRS_GyroSim(m_ahrs);
    // m_gyroSim SimDevice
  }

  @Override
  public void simulationPeriodic() {
    // m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  public void log(double mod) {

    log_counter++;
    if ((log_counter % mod)==0) {
     
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

  public void reset() {

    // if (m_ahrs.isConnected()) {
    // m_ahrs.reset();
    // m_ahrs.resetDisplacement();
    // }

  }

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

  // @Override
  public void close() throws Exception {
    // //m_gyro.close();
    // m_gyro_ahrs.close();
  }

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
