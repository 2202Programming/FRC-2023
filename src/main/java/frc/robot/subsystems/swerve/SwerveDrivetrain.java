// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.NTStrings;
import frc.robot.subsystems.Sensors_Subsystem.EncoderID;
import frc.robot.util.PoseMath;

public class SwerveDrivetrain extends SubsystemBase {
  /**
   * Inversions account for rotations of the module relative to left or right side
   * of robot.
   * 
   * CANCoders are setup in Sensors and will have CCW= positve convention. Their
   * offsets are adjusted by their use in the drive train.
   */
  boolean kDriveMotorInvert_Right = true;
  boolean kAngleMotorInvert_Right = false;
  boolean kAngleCmdInvert_Right = false;
  boolean kDriveMotorInvert_Left = false;
  boolean kAngleMotorInvert_Left = false;
  boolean kAngleCmdInvert_Left = false;

  /**
   *
   * Modules are in the order of - Front Left Front Right Back Left Back Right
   * 
   * Positive x values represent moving toward the front of the robot Positive y
   * values represent moving toward the left of the robot All lengths in feet.
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(RobotContainer.RC().m_robotSpecs.chassisConfig.XwheelOffset, RobotContainer.RC().m_robotSpecs.chassisConfig.YwheelOffset), // Front Left
      new Translation2d(RobotContainer.RC().m_robotSpecs.chassisConfig.XwheelOffset, -RobotContainer.RC().m_robotSpecs.chassisConfig.YwheelOffset), // Front Right
      new Translation2d(-RobotContainer.RC().m_robotSpecs.chassisConfig.XwheelOffset, RobotContainer.RC().m_robotSpecs.chassisConfig.YwheelOffset), // Back Left
      new Translation2d(-RobotContainer.RC().m_robotSpecs.chassisConfig.XwheelOffset, -RobotContainer.RC().m_robotSpecs.chassisConfig.YwheelOffset) // Back Right
  );
  private SwerveDriveOdometry m_odometry;
  private Pose2d m_pose;
  private Pose2d old_pose;
  private SwerveModuleState[] cur_states;
  private SwerveModuleState[] meas_states;   //measured wheel speed & angle

  // sensors and our mk3 modules
  private final Sensors_Subsystem sensors;
  private final SwerveModuleMK3[] modules;

  private NetworkTable table;
  private NetworkTable postionTable;
  private NetworkTableEntry currentX;
  private NetworkTableEntry currentY;
  private NetworkTableEntry currentHeading;
  private NetworkTableEntry nt_currentBearing;

  private NetworkTableEntry velocityFL;
  private NetworkTableEntry velocityFR;
  private NetworkTableEntry velocityBL;
  private NetworkTableEntry velocityBR;
  private NetworkTableEntry posFL;
  private NetworkTableEntry posFR;
  private NetworkTableEntry posBL;
  private NetworkTableEntry posBR;
  private NetworkTableEntry robotVel;
  
  double drive_kP = DriveTrain.drivePIDF.getP();
  double drive_kI = DriveTrain.drivePIDF.getI();
  double drive_kD = DriveTrain.drivePIDF.getD();
  double drive_kFF = DriveTrain.drivePIDF.getF();

  double angle_kP = DriveTrain.anglePIDF.getP();
  double angle_kI = DriveTrain.anglePIDF.getI();
  double angle_kD = DriveTrain.anglePIDF.getD();
  double angle_kFF = DriveTrain.anglePIDF.getF();

  public final String NT_Name = "DT"; // expose data under DriveTrain table
  private int timer;
  private double currentBearing = 0;
  private double filteredBearing = 0;
  private double filteredVelocity = 0;

  // Creates a new Single-Pole IIR filter
  // Time constant is 0.1 seconds
  // Period is 0.02 seconds - this is the standard FRC main loop period
  private LinearFilter bearingFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private LinearFilter velocityFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

  public final Field2d m_field = new Field2d();

  public SwerveDrivetrain() {
    sensors = RobotContainer.RC().sensors;

    var MT = CANSparkMax.MotorType.kBrushless;
    modules = new SwerveModuleMK3[] {
        // Front Left
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_FL_DRIVE, MT), new CANSparkMax(CAN.DT_FL_ANGLE, MT),
            RobotContainer.RC().m_robotSpecs.wheelOffsets.CC_FL_OFFSET, sensors.getCANCoder(EncoderID.FrontLeft), kAngleMotorInvert_Left,
            kAngleCmdInvert_Left, kDriveMotorInvert_Left, "FL"),
        // Front Right
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_FR_DRIVE, MT), new CANSparkMax(CAN.DT_FR_ANGLE, MT),
            RobotContainer.RC().m_robotSpecs.wheelOffsets.CC_FR_OFFSET, sensors.getCANCoder(EncoderID.FrontRight), kAngleMotorInvert_Right,
            kAngleCmdInvert_Right, kDriveMotorInvert_Right, "FR"),
        // Back Left
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_BL_DRIVE, MT), new CANSparkMax(CAN.DT_BL_ANGLE, MT),
            RobotContainer.RC().m_robotSpecs.wheelOffsets.CC_BL_OFFSET, sensors.getCANCoder(EncoderID.BackLeft), kAngleMotorInvert_Left,
            kAngleCmdInvert_Left, kDriveMotorInvert_Left, "BL"),
        // Back Right
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_BR_DRIVE, MT), new CANSparkMax(CAN.DT_BR_ANGLE, MT),
            RobotContainer.RC().m_robotSpecs.wheelOffsets.CC_BR_OFFSET, sensors.getCANCoder(EncoderID.BackRight), kAngleMotorInvert_Right,
            kAngleCmdInvert_Right, kDriveMotorInvert_Right, "BR") };

    m_odometry = new SwerveDriveOdometry(kinematics, sensors.getRotation2d());
    cur_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
    meas_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    // for updating CAN status in periodic
    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    postionTable = NetworkTableInstance.getDefault().getTable(NTStrings.NT_Name_Position);
    currentX = postionTable.getEntry("/Current X");
    currentY = postionTable.getEntry("/Current Y");
    currentHeading = postionTable.getEntry("/Current Heading");
    nt_currentBearing = postionTable.getEntry("/Current Bearing");
    velocityFL = table.getEntry("/Velocity Front Left");
    velocityFR = table.getEntry("/Velocity Front Right");
    velocityBL = table.getEntry("/Velocity Back Left");
    velocityBR = table.getEntry("/Velocity Back Right");
    posFL = table.getEntry("/POS FL");
    posFR = table.getEntry("/POS FR");
    posBL = table.getEntry("/POS BL");
    posBR = table.getEntry("/POS BR");
    robotVel = postionTable.getEntry("/RobotVel");

    SmartDashboard.putData("Field", m_field);

    // display PID coefficients on SmartDashboard if tuning drivetrain
    /*
    SmartDashboard.putNumber("Drive P", drive_kP);
    SmartDashboard.putNumber("Drive I", drive_kI);
    SmartDashboard.putNumber("Drive D", drive_kD);
    SmartDashboard.putNumber("Drive Feed Forward", drive_kFF);

    SmartDashboard.putNumber("Angle P", angle_kP);
    SmartDashboard.putNumber("Angle I", angle_kI);
    SmartDashboard.putNumber("Angle D", angle_kD);
    SmartDashboard.putNumber("Angle Feed Forward", angle_kFF);
    */
    m_pose = m_odometry.update(sensors.getRotation2d(), cur_states);
    old_pose = m_pose;
  }

  public void drive(SwerveModuleState[] states) {
    this.cur_states = states; //keep copy of commanded states so we can stop() withs 

    //if any one wheel is above max obtainable speed, reduce them all in the same ratio to maintain control
    //SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrain.kMaxSpeed);

    // output the angle and velocity for each module
    for (int i = 0; i < states.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  // used for testing
  public void testDrive(double speed, double angle) {
    // output the angle and speed (meters per sec) for each module
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(new SwerveModuleState(speed, new Rotation2d(Math.toRadians(angle))));
    }
  }

  @Override
  public void periodic() {
    // update data from each of the swerve drive modules.
    for (int i = 0; i < modules.length; i++) {
      modules[i].periodic();
      meas_states[i].speedMetersPerSecond = modules[i].getVelocity();
      meas_states[i].angle = modules[i].getAngleRot2d();
    }

    // update pose
    old_pose = m_pose;
    m_pose = m_odometry.update(sensors.getRotation2d(), meas_states);

    // from -PI to +PI
    double temp = Math.atan2(m_pose.getY() - old_pose.getY(), m_pose.getX() - old_pose.getX());
    if(temp != 0){ //remove singularity when moving too slow - otherwise lots of jitter
      currentBearing = temp;
      // convert this to degrees in the range -180 to 180
      currentBearing = Math.toDegrees(currentBearing);
    }
    //run bearing through low pass filter
    filteredBearing = bearingFilter.calculate(currentBearing);

    // velocity assuming period is 0.02 seconds - this is the standard FRC main loop period
    filteredVelocity = velocityFilter.calculate(PoseMath.poseDistance(m_pose, old_pose)/0.02);

    // updates CAN status data every 4 cycles
    timer++;
    if (timer == 5) {
      currentX.setDouble(m_pose.getX());
      currentY.setDouble(m_pose.getY());
      currentHeading.setDouble(m_pose.getRotation().getDegrees());
      velocityFL.setDouble(modules[0].getVelocity());
      velocityFR.setDouble(modules[1].getVelocity());
      velocityBL.setDouble(modules[2].getVelocity());
      velocityBR.setDouble(modules[3].getVelocity());

      posFL.setDouble(modules[0].getPosition());
      posFR.setDouble(modules[1].getPosition());
      posBL.setDouble(modules[2].getPosition());
      posBR.setDouble(modules[3].getPosition());
      
      nt_currentBearing.setDouble(filteredBearing);
      robotVel.setDouble(filteredVelocity);
      timer = 0;

      m_field.setRobotPose(m_odometry.getPoseMeters());
      //if Drivetrain tuning
      //pidTuning();
    }
  }

  @Override
  public void simulationPeriodic() {
    // any sim work for each module
    for (int i = 0; i < modules.length; i++) {
      // modules[i].periodic();
    }
  }

  public SwerveModuleMK3 getMK3(int modID) {
    if ((modID < 0) || (modID > modules.length - 1))
      return null;
    return modules[modID];
  }

  // sets X,Y, and sets current angle (will apply sensors correction)
  public void setPose(Pose2d new_pose) {
    m_pose = new_pose;
    m_odometry.resetPosition(m_pose, sensors.getRotation2d());
  }

  // resets X,Y, and set current angle to be 0
  public void resetPose() {
    m_pose = new Pose2d(0, 0, new Rotation2d(0));
    m_odometry.resetPosition(m_pose, sensors.getRotation2d());
  }

  //reset angle to be zero, but retain X and Y; takes a Rotation2d object
  public void resetAnglePose(Rotation2d rot){
    m_pose = new Pose2d(getPose().getX(), getPose().getY(), rot);
    m_odometry.resetPosition(m_pose, sensors.getRotation2d());  //updates gryo offset
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public void printPose(){
    System.out.println("***POSE X:" + m_pose.getX() + ", Y:" + m_pose.getY() + ", Rot:" + m_pose.getRotation().getDegrees());
  }

  public double getBearing(){
    return filteredBearing;
  }

  public double getVelocity(){
    return filteredVelocity;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public ChassisSpeeds  getChassisSpeeds() {
      return  kinematics.toChassisSpeeds(meas_states);
  }

  public ChassisSpeeds getFieldRelativeSpeeds(){    
    return new ChassisSpeeds(
      getChassisSpeeds().vxMetersPerSecond * sensors.getRotation2d().getCos() - getChassisSpeeds().vyMetersPerSecond * sensors.getRotation2d().getSin(),
      getChassisSpeeds().vyMetersPerSecond * sensors.getRotation2d().getCos() + getChassisSpeeds().vxMetersPerSecond * sensors.getRotation2d().getSin(),
      getChassisSpeeds().omegaRadiansPerSecond);
  }

  /**
   * stop() - zero the current state's velocity component and leave angles as they are
   */
  public void stop() {
    SwerveModuleState state = new SwerveModuleState();
    state.speedMetersPerSecond =0.0;
    // output the angle and velocity for each module
    for (int i = 0; i < modules.length; i++) {
      state.angle = Rotation2d.fromDegrees(modules[i].getAngle());
      modules[i].setDesiredState(state);
    }
  }

  public void setBrakeMode(){
    for (int i = 0; i < modules.length; i++) {
      modules[i].setBrakeMode();
    }
    System.out.println("***BRAKES ENGAGED***");
  }

  public void setCoastMode(){
    for (int i = 0; i < modules.length; i++) {
      modules[i].setCoastMode();
    }
    System.out.println("***BRAKES RELEASED***");
  }

  //  TODO: Move to a TEST/Tuning command  - DPL 2/21/22
  // private void pidTuning() { //if drivetrain tuning

  //   // read PID coefficients from SmartDashboard if tuning drivetrain
  //   double drive_p = SmartDashboard.getNumber("Drive P Gain", DriveTrain.drivePIDF.getP());
  //   double drive_i = SmartDashboard.getNumber("Drive I Gain", DriveTrain.drivePIDF.getI());
  //   double drive_d = SmartDashboard.getNumber("Drive D Gain", DriveTrain.drivePIDF.getD());
  //   double drive_ff = SmartDashboard.getNumber("Drive Feed Forward", DriveTrain.drivePIDF.getF());
  //   double angle_p = SmartDashboard.getNumber("Angle P Gain", DriveTrain.anglePIDF.getP());
  //   double angle_i = SmartDashboard.getNumber("Angle I Gain", DriveTrain.anglePIDF.getI());
  //   double angle_d = SmartDashboard.getNumber("Angle D Gain", DriveTrain.anglePIDF.getD());
  //   double angle_ff = SmartDashboard.getNumber("Angle Feed Forward", DriveTrain.anglePIDF.getF());

  //   // if anything changes in drive PID, update all the modules with a new drive PID
  //   if ((drive_p != drive_kP) || (drive_i != drive_kI) || (drive_d != drive_kD) || (drive_ff != drive_kFF)) {
  //     drive_kP = drive_p;
  //     drive_kI = drive_i;
  //     drive_kD = drive_d;
  //     drive_kFF = drive_ff;
  //     for (SwerveModuleMK3 i : modules) {
  //       i.setDrivePID(new PIDFController(drive_kP, drive_kI, drive_kD, drive_kFF));
  //     }
  //   }

  //   // if anything changes in angle PID, update all the modules with a new angle PID
  //   if ((angle_p != angle_kP) || (angle_i != angle_kI) || (angle_d != angle_kD) || (angle_ff != angle_kFF)) {
  //     angle_kP = angle_p;
  //     angle_kI = angle_i;
  //     angle_kD = angle_d;
  //     angle_kFF = angle_ff;
  //     for (SwerveModuleMK3 i : modules) {
  //       i.setAnglePID(new PIDFController(angle_kP, angle_kI, angle_kD, angle_kFF));
  //     }
  //   }
  // }

}