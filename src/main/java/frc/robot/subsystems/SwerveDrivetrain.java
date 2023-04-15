// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.ChassisConfig;
import frc.robot.Constants.DriveTrain;
import frc.robot.Constants.NTStrings;
import frc.robot.Constants.WheelOffsets;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors_Subsystem.EncoderID;
import frc.robot.util.ModMath;
import frc.robot.util.PoseMath;
import frc.robot.util.VisionWatchdog;
import frc.robot.Constants.ChassisInversionSpecs;

public class SwerveDrivetrain extends SubsystemBase {

  // cc is the chassis config for all our pathing math
  private final ChassisConfig cc = RobotContainer.RC().robotSpecs.getChassisConfig(); // chassis config
  private final WheelOffsets wc = RobotContainer.RC().robotSpecs.getWheelOffset(); // wc = wheel config
  private final ChassisInversionSpecs is = RobotContainer.RC().robotSpecs.getChassisInversionSpecs(); // is = inversion specs

  /**
   *
   * Modules are in the order of - Front Left, Front Right, Back Left, Back Right
   * 
   * Positive x --> represent moving toward the front of the robot
   * Positive y --> represent moving toward the left of the robot
   * [m]
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(cc.XwheelOffset, cc.YwheelOffset), // Front Left
      new Translation2d(cc.XwheelOffset, -cc.YwheelOffset), // Front Right
      new Translation2d(-cc.XwheelOffset, cc.YwheelOffset), // Back Left
      new Translation2d(-cc.XwheelOffset, -cc.YwheelOffset) // Back Right
  );
  private SwerveDriveOdometry m_odometry;
  private Pose2d m_pose;
  private Pose2d old_pose;
  private Pose2d m_pose_integ; // incorporates vision

  private double maxImagingVelocity = 2.0; //m/s
  private VisionWatchdog watchdog;

  private SwerveModuleState[] meas_states; // measured wheel speed & angle
  private SwerveModulePosition[] meas_pos = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  // sensors and our mk3 modules
  private final Sensors_Subsystem sensors;
  private final SwerveModuleMK3[] modules;

  // used to update postion esimates
  double kTimeoffset = .1; // [s] measurement delay from photonvis TODO:measure this???
  private final PhotonVision photonVision;
  private final Limelight_Subsystem limelight;
  // Network tables
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
  private NetworkTableEntry est_pose_od_x;
  private NetworkTableEntry est_pose_od_y;
  private NetworkTableEntry est_pose_od_h;
  private NetworkTableEntry est_pose_integ_x;
  private NetworkTableEntry est_pose_integ_y;
  private NetworkTableEntry est_pose_integ_h;

  // ll pose updating
  private NetworkTableEntry nt_x_diff;
  private NetworkTableEntry nt_y_diff;
  private NetworkTableEntry nt_yaw_diff;
  private boolean visionPoseUsingRotation = true;
  private boolean visionPoseEnabled = true;

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
  private LinearFilter bearingFilter = LinearFilter.singlePoleIIR(0.1, Constants.DT);
  private LinearFilter velocityFilter = LinearFilter.singlePoleIIR(0.1, Constants.DT);

  public final SwerveDrivePoseEstimator m_poseEstimator_ll;
  public final SwerveDrivePoseEstimator m_poseEstimator_pv;
  private double x_diff; // [m]
  private double y_diff; // [m]
  private double yaw_diff; // [deg]

  private Pose2d llPose;
  private Pose2d pvPose;
  public final Field2d m_field = new Field2d();

  public SwerveDrivetrain() {
    sensors = RobotContainer.RC().sensors;
    photonVision = RobotContainer.RC().photonVision;
    limelight = RobotContainer.RC().limelight;
    watchdog = new VisionWatchdog(3.0);

    var MT = CANSparkMax.MotorType.kBrushless;
    modules = new SwerveModuleMK3[] {
        // Front Left
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_FL_DRIVE, MT), new CANSparkMax(CAN.DT_FL_ANGLE, MT),
            wc.CC_FL_OFFSET, sensors.getCANCoder(EncoderID.FrontLeft), is.FL.kAngleMotorInvert,
            is.FL.kAngleCmdInvert, is.FL.kDriveMotorInvert, "FL"),
        // Front Right
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_FR_DRIVE, MT), new CANSparkMax(CAN.DT_FR_ANGLE, MT),
            wc.CC_FR_OFFSET, sensors.getCANCoder(EncoderID.FrontRight), is.FR.kAngleMotorInvert,
            is.FR.kAngleCmdInvert, is.FR.kDriveMotorInvert, "FR"),
        // Back Left
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_BL_DRIVE, MT), new CANSparkMax(CAN.DT_BL_ANGLE, MT),
            wc.CC_BL_OFFSET, sensors.getCANCoder(EncoderID.BackLeft), is.BL.kAngleMotorInvert,
            is.BL.kAngleCmdInvert, is.BL.kDriveMotorInvert, "BL"),
        // Back Right
        new SwerveModuleMK3(new CANSparkMax(CAN.DT_BR_DRIVE, MT), new CANSparkMax(CAN.DT_BR_ANGLE, MT),
            wc.CC_BR_OFFSET, sensors.getCANCoder(EncoderID.BackRight), is.BR.kAngleMotorInvert,
            is.BR.kAngleCmdInvert, is.BR.kDriveMotorInvert, "BR") };

    /*
     * Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings.
     * The numbers used below are robot specific, and should be tuned.
     */
    m_poseEstimator_ll = new SwerveDrivePoseEstimator(
        kinematics,
        sensors.getRotation2d(),
        meas_pos,
        new Pose2d(), // initial pose ()
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // std x,y, heading from odmetry
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // std x, y heading from vision

    m_poseEstimator_pv = new SwerveDrivePoseEstimator(
      kinematics,
      sensors.getRotation2d(),
      meas_pos,
      new Pose2d(), // initial pose ()
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // std x,y, heading from odmetry
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // std x, y heading from vision

    m_odometry = new SwerveDriveOdometry(kinematics, sensors.getRotation2d(), meas_pos);
    // cur_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
    meas_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    m_pose = m_odometry.update(sensors.getRotation2d(), meas_pos);
    old_pose = m_pose;

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

    est_pose_od_x = table.getEntry("est_od_x");
    est_pose_od_y = table.getEntry("est_od_y");
    est_pose_od_h = table.getEntry("est_od_h");
    est_pose_integ_x = table.getEntry("est_int_x");
    est_pose_integ_y = table.getEntry("est_int_y");
    est_pose_integ_h = table.getEntry("est_int_h");

    // ll pose estimating
    nt_x_diff = table.getEntry("vision_x_diff");
    nt_y_diff = table.getEntry("vision_y_diff");
    nt_yaw_diff = table.getEntry("vision_yaw_diff");

    SmartDashboard.putData("Field", m_field);

    // display PID coefficients on SmartDashboard if tuning drivetrain
    /*
     * SmartDashboard.putNumber("Drive P", drive_kP);
     * SmartDashboard.putNumber("Drive I", drive_kI);
     * SmartDashboard.putNumber("Drive D", drive_kD);
     * SmartDashboard.putNumber("Drive Feed Forward", drive_kFF);
     * 
     * SmartDashboard.putNumber("Angle P", angle_kP);
     * SmartDashboard.putNumber("Angle I", angle_kI);
     * SmartDashboard.putNumber("Angle D", angle_kD);
     * SmartDashboard.putNumber("Angle Feed Forward", angle_kFF);
     */

    offsetDebug();
  }

  private void offsetDebug() {
    periodic(); // run to initialize module values
    double offsetFL = wc.CC_FL_OFFSET;
    double measuredFL = modules[0].m_internalAngle;

    double offsetFR = wc.CC_FR_OFFSET;
    double measuredFR = modules[1].m_internalAngle;

    double offsetBL = wc.CC_BL_OFFSET;
    double measuredBL = modules[2].m_internalAngle;

    double offsetBR = wc.CC_BR_OFFSET;
    double measuredBR = modules[3].m_internalAngle;

    System.out.println("================Offsets==================");
    System.out.println("FL: offset " + offsetFL + ", measured " + measuredFL + ", should be "
        + ModMath.fmod360_2(offsetFL - measuredFL));
    System.out.println("FR: offset " + offsetFR + ", measured " + measuredFR + ", should be "
        + ModMath.fmod360_2(offsetFR - measuredFR));
    System.out.println("BL: offset " + offsetBL + ", measured " + measuredBL + ", should be "
        + ModMath.fmod360_2(offsetBL - measuredBL));
    System.out.println("BR: offset " + offsetBR + ", measured " + measuredBR + ", should be "
        + ModMath.fmod360_2(offsetBR - measuredBR));
    System.out.println("============Offsets Done==============");
  }

  public void drive(SwerveModuleState[] states) {
    // this.cur_states = states; //keep copy of commanded states so we can stop()
    // withs

    // if any one wheel is above max obtainable speed, reduce them all in the same
    // ratio to maintain control
    // SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrain.kMaxSpeed);

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
      meas_states[i].angle = meas_pos[i].angle = modules[i].getAngleRot2d();
      meas_pos[i].distanceMeters = modules[i].getPosition();
    }

    updateOdometry(); // upates old_pose and m_pose

    double tol = Math.toRadians(0.5);
    // from -PI to +PI (radians)
    double temp = Math.atan2(m_pose.getY() - old_pose.getY(), m_pose.getX() - old_pose.getX()); 
    // Changed from !=0 to include tol variable
    if (Math.abs(temp) < tol) { // remove singularity when moving too slow - otherwise lots of jitter
      currentBearing = temp;
      // convert this to degrees in the range -180 to 180
      currentBearing = Math.toDegrees(currentBearing);
    }
    // run bearing through low pass filter
    filteredBearing = bearingFilter.calculate(currentBearing);

    // velocity assuming period is 0.02 seconds - this is the standard FRC main loop
    // period
    filteredVelocity = velocityFilter.calculate(PoseMath.poseDistance(m_pose, old_pose) / 0.02);

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

      est_pose_od_x.setDouble(m_pose.getX());
      est_pose_od_y.setDouble(m_pose.getY());
      est_pose_od_h.setDouble(m_pose.getRotation().getDegrees());
      m_field.setRobotPose(m_odometry.getPoseMeters());

      if (m_pose_integ == null)
        return;
      est_pose_integ_x.setDouble(m_pose_integ.getX());
      est_pose_integ_y.setDouble(m_pose_integ.getY());
      est_pose_integ_h.setDouble(m_pose_integ.getRotation().getDegrees());



      // if Drivetrain tuning
      // pidTuning();
    }
  }

  public void simulationInit() {
    // WIP placeholder
    // motor/inertia models

  }

  @Override
  public void simulationPeriodic() {
    // WIP
  }

  public SwerveModuleMK3 getMK3(int modID) {
    if ((modID < 0) || (modID > modules.length - 1))
      return null;
    return modules[modID];
  }

  // todo: eleminate this function, duplicate of resetpose
  // sets X,Y, and sets current angle (will apply sensors correction)
  public void setPose(Pose2d new_pose) {
    resetPose(new_pose);
  }

  // TODO: do we REALLY think this is where we need to go? field coords???
  // resets X,Y, and set current angle to be 0
  public void resetPose() {
    resetPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  }

  public void resetPose(Pose2d pose) {
    m_pose = pose;
    m_odometry.resetPosition(sensors.getRotation2d(), meas_pos, m_pose);

    // keep our vision pose estimators up to date
    if (photonVision != null) {
      // position and timestamp
      photonVision.setInitialPose(new Pair<Pose2d, Double>(pose, 0.0));
    }
    if (limelight != null) {
      limelight.setInitialPose(pose, 0.0);
    }
  }

  // reset angle to be zero, but retain X and Y; takes a Rotation2d object
  public void resetAnglePose(Rotation2d rot) {
    m_pose = new Pose2d(getPose().getX(), getPose().getY(), rot);
    m_odometry.resetPosition(sensors.getRotation2d(), meas_pos, m_pose); // updates gryo offset
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public void printPose() {
    System.out
        .println("***POSE X:" + m_pose.getX() + ", Y:" + m_pose.getY() + ", Rot:" + m_pose.getRotation().getDegrees());
  }

  // TODO: bearing should be to or from *what*? split out?
  public double getBearing() {
    return filteredBearing;
  }

  public double getVelocity() {
    return filteredVelocity;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(meas_states);
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return new ChassisSpeeds(
        getChassisSpeeds().vxMetersPerSecond * sensors.getRotation2d().getCos()
            - getChassisSpeeds().vyMetersPerSecond * sensors.getRotation2d().getSin(),
        getChassisSpeeds().vyMetersPerSecond * sensors.getRotation2d().getCos()
            + getChassisSpeeds().vxMetersPerSecond * sensors.getRotation2d().getSin(),
        getChassisSpeeds().omegaRadiansPerSecond);
  }

  /**
   * stop() - zero the current state's velocity component and leave angles as they
   * are
   */
  public void stop() {
    SwerveModuleState state = new SwerveModuleState();
    state.speedMetersPerSecond = 0.0;
    // output the angle and velocity for each module
    for (int i = 0; i < modules.length; i++) {
      state.angle = Rotation2d.fromDegrees(modules[i].getAngle());
      modules[i].setDesiredState(state);
    }
  }

  public void setBrakeMode() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setBrakeMode();
    }
    System.out.println("***BRAKES ENGAGED***");
  }

  public void setCoastMode() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setCoastMode();
    }
    System.out.println("***BRAKES RELEASED***");
  }

  public static Command pathFactoryAuto(SwerveDrivetrain dt, Sensors_Subsystem sensors, double maxVel, double maxAcc,
      String pathname, int pathNum) {
    SwerveDrivetrain m_robotDrive = dt;
    Sensors_Subsystem m_sensors = sensors;

    var path = PathPlanner.loadPath(pathname, maxVel, maxAcc);

    if (path == null) {
      return new InstantCommand(); // no path selected
    }

    dt.m_field.getObject("Path" + pathNum).setTrajectory(path);

    // get initial state from the trajectory
    PathPlannerState initialState = path.getInitialState();
    Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

    PIDController xController = new PIDController(4.0, 0.0, 0.0, Constants.DT); // [m]
    PIDController yController = new PIDController(4.0, 0.0, 0.0, Constants.DT); // [m]
    PIDController thetaController = new PIDController(4, 0, 0, Constants.DT); // [rad]
    // Units are radians for thetaController; PPSwerveController is using radians
    // internally.
    thetaController.enableContinuousInput(-Math.PI, Math.PI); // prevent piroutte paths over continuity

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        path,
        m_robotDrive::getPose, // Functional interface to feed supplier
        m_robotDrive.getKinematics(),
        // Position controllers
        xController,
        yController,
        thetaController,
        m_robotDrive::drive,
        m_robotDrive);

    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_robotDrive.setPose(startingPose);
          if (pathNum == 1)
            m_sensors.setAutoStartPose(startingPose);
        }),
        new PrintCommand("***Factory: Running Path " + pathname),
        swerveControllerCommand,
        new InstantCommand(m_robotDrive::stop),
        new PrintCommand("***Done Running Path " + pathname));
  }

  public PathPlannerTrajectory pathFactoryTele(Pose2d finalPoint) {
    return PathPlanner.generatePath(new PathConstraints(1, 1),
        new PathPoint(m_pose.getTranslation(), finalPoint.getRotation(), m_pose.getRotation()),
        new PathPoint(finalPoint.getTranslation(), finalPoint.getRotation(), finalPoint.getRotation()));
  }

  /** Updates the field relative position of the robot. */
  void updateOdometry() {
    // update states
    old_pose = m_pose;
    m_pose = m_odometry.update(sensors.getRotation2d(), meas_pos);

    // vision  from here down
    if (limelight != null) {
      m_poseEstimator_ll.update(sensors.getRotation2d(), meas_pos); //this should happen every robot cycle, regardless of vision targets.
      llPoseEstimatorUpdate();
    }

    if (photonVision != null){
      m_poseEstimator_pv.update(sensors.getRotation2d(), meas_pos); //this should happen every robot cycle, regardless of vision targets.
      pvPoseEstimatorUpdate();
    }

    //only update pose from imaging if max velocity is low enough
    //get center of robot velocity from sqrt of vX2 + vY2
    if(Math.sqrt(
              Math.pow(kinematics.toChassisSpeeds(meas_states).vxMetersPerSecond,2) +
              Math.pow(kinematics.toChassisSpeeds(meas_states).vyMetersPerSecond,2)) > maxImagingVelocity){
      return;
    }

    if ((limelight != null) && (llPose != null) && (limelight.getNumApriltags() > 0)) { //just use LL for now
      Pose2d prev_m_Pose = m_pose;
      if(visionPoseEnabled) {
        watchdog.update(prev_m_Pose, llPose);
        if (visionPoseUsingRotation) {
          setPose(llPose); //update robot pose from swervedriveposeestimator, include vision-based rotation
        }
        else{
          setPose(new Pose2d(llPose.getTranslation(), prev_m_Pose.getRotation())); //update robot translation from swervedriveposeestimator, do not update rotation
        }
      }
      x_diff = Math.abs(prev_m_Pose.getX() -  m_pose.getX());
      y_diff = Math.abs(prev_m_Pose.getY() - m_pose.getY());
      yaw_diff = Math.abs(prev_m_Pose.getRotation().getDegrees() - m_pose.getRotation().getDegrees());
      // vision pose updating NTs
      nt_x_diff.setDouble(x_diff);
      nt_y_diff.setDouble(y_diff);
      nt_yaw_diff.setDouble(yaw_diff);
    }
    

    // WIP use other poseEstimator
    if (limelight != null && photonVision != null)
    {
      /*
      if (photonVision.hasAprilTarget() && pvPose != null){
        Pose2d prev_m_Pose = m_pose;
        setPose(pvPose); // update main pose with vision integrated pose if we have a target
        double x_dif = Math.abs(prev_m_Pose.getX() -  m_pose.getX()) / 16.54099;
        double y_dif = Math.abs(prev_m_Pose.getY() - m_pose.getY()) / 8.002778;
        System.out.println("***UPDATED BY PV \n -----> X Fixed: " + x_dif + "%   Y Fixed: " + y_dif
           + "%");
      }*/
      // if (limelight.hasAprilTarget() && llPose != null){ // else if with photonvision
      //   Pose2d prev_m_Pose = m_pose;
      //   setPose(llPose);
      //   double x_dif = Math.abs(prev_m_Pose.getX() -  m_pose.getX()) / 16.54099;
      //   double y_dif = Math.abs(prev_m_Pose.getY() - m_pose.getY()) / 8.002778;
      //   System.out.println("***UPDATED BY LL \n -----> X: " + x_dif + "%   Y: " + y_dif+ "%   Y Fixed: " + y_dif
      //       + "%");
      // }
    }

  }

  void pvPoseEstimatorUpdate(){
    
    if (photonVision.hasAprilTarget() ) {
      // this should happen only if we have a tag in view
      // OK if it is run only intermittanly.  Uses latency of vision pose.
      m_poseEstimator_pv.addVisionMeasurement(photonVision.getPoseEstimate().getFirst(), photonVision.getVisionTimestamp());

      pvPose = m_poseEstimator_pv.getEstimatedPosition();
    }
  }

  void llPoseEstimatorUpdate(){
    
    if (limelight.getNumApriltags() > 0) {
      // this should happen only if we have a tag in view
      // OK if it is run only intermittanly.  Uses latency of vision pose.
      m_poseEstimator_ll.addVisionMeasurement(limelight.getBluePose(), limelight.getVisionTimestamp());

      llPose = m_poseEstimator_ll.getEstimatedPosition();
    }
  }

public Pose2d getLLEstimate() {
  return new Pose2d(llPose.getTranslation(), llPose.getRotation());
}

public Pose2d getPVEstimate() {
  return new Pose2d(pvPose.getTranslation(), pvPose.getRotation());
}

public void disableVisionPoseRotation() {
  visionPoseUsingRotation = false;
  System.out.println("*** Vision pose updating rotation disabled***");
}

public void enableVisionPoseRotation() {
  visionPoseUsingRotation = true;
  System.out.println("*** Vision pose updating rotation enabled***");
}

public void enableVisionPose() {
  visionPoseEnabled = true;
  System.out.println("*** Vision updating pose enabled***");
}

public void disableVisionPose(){
  visionPoseEnabled = false;
  System.out.println("*** Vision updating pose disabled***");
}

}
// TODO: Move to a TEST/Tuning command - DPL 2/21/22
// private void pidTuning() { //if drivetrain tuning

// // read PID coefficients from SmartDashboard if tuning drivetrain
// double drive_p = SmartDashboard.getNumber("Drive P Gain",
// DriveTrain.drivePIDF.getP());
// double drive_i = SmartDashboard.getNumber("Drive I Gain",
// DriveTrain.drivePIDF.getI());
// double drive_d = SmartDashboard.getNumber("Drive D Gain",
// DriveTrain.drivePIDF.getD());
// double drive_ff = SmartDashboard.getNumber("Drive Feed Forward",
// DriveTrain.drivePIDF.getF());
// double angle_p = SmartDashboard.getNumber("Angle P Gain",
// DriveTrain.anglePIDF.getP());
// double angle_i = SmartDashboard.getNumber("Angle I Gain",
// DriveTrain.anglePIDF.getI());
// double angle_d = SmartDashboard.getNumber("Angle D Gain",
// DriveTrain.anglePIDF.getD());
// double angle_ff = SmartDashboard.getNumber("Angle Feed Forward",
// DriveTrain.anglePIDF.getF());

// // if anything changes in drive PID, update all the modules with a new drive
// PID
// if ((drive_p != drive_kP) || (drive_i != drive_kI) || (drive_d != drive_kD)
// || (drive_ff != drive_kFF)) {
// drive_kP = drive_p;
// drive_kI = drive_i;
// drive_kD = drive_d;
// drive_kFF = drive_ff;
// for (SwerveModuleMK3 i : modules) {
// i.setDrivePID(new PIDFController(drive_kP, drive_kI, drive_kD, drive_kFF));
// }
// }

// // if anything changes in angle PID, update all the modules with a new angle
// PID
// if ((angle_p != angle_kP) || (angle_i != angle_kI) || (angle_d != angle_kD)
// || (angle_ff != angle_kFF)) {
// angle_kP = angle_p;
// angle_kI = angle_i;
// angle_kD = angle_d;
// angle_kFF = angle_ff;
// for (SwerveModuleMK3 i : modules) {
// i.setAnglePID(new PIDFController(angle_kP, angle_kI, angle_kD, angle_kFF));
// }
// }
// }
