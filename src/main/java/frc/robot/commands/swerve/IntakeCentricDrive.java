package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;

/* Current driving behavior:
  Starts in field centric
  B will toggle between field centric and intake centric
  Holding right trigger will switch to hub centric until you let go, then it will go back to original mode
          (either field or intake centric, depending what you started in)
  If in intake centric and you try to rotate with left joystick, will drop back to field centric mode.
*/


public class IntakeCentricDrive extends DriveCmdClass {

  final SwerveDrivetrain drivetrain;
  final DriverControls dc;
  final SwerveDriveKinematics kinematics;

  boolean lastShootMode = false;
  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  Rotation2d currrentHeading;
  SwerveModuleState[] output_states;

  // PID for heading to a target
  private PIDController intakeAnglePid;
  private double angle_kp = 0.075;
  private double angle_ki = 0.004;
  private double angle_kd = 0.005;

  private Rotation2d lastBearing; //stores the last significant bearing angle

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  NetworkTable table;
  private NetworkTableEntry NTangleError;
  private NetworkTableEntry NTTargetAngle;
  public final String NT_Name = "Shooter"; 

  double log_counter = 0;

  Rotation2d m_targetAngle;
  Rotation2d m_angleError;

  // Creates a new Single-Pole IIR filter
  // Time constant is 0.1 seconds
  // Period is 0.02 seconds - this is the standard FRC main loop period
  private LinearFilter bearingFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private double filteredBearing = 0;

  public IntakeCentricDrive(SwerveDrivetrain drivetrain, DriverControls dc) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc;
    this.kinematics = drivetrain.getKinematics();

    intakeAnglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    intakeAnglePid.enableContinuousInput(-180, 180);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    NTangleError = table.getEntry("/IntakeCentric/angleError");
    NTTargetAngle = table.getEntry("/IntakeCentric/TargetAngle");

    lastBearing = new Rotation2d(0);
  }

  @Override
  public void initialize() {
    updateNT();
  }

  void calculate() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;
    rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed;

    filteredBearing = bearingFilter.calculate(getJoystickBearing().getRadians());

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    rot = MathUtil.clamp(rot, -Constants.DriveTrain.kMaxAngularSpeed, Constants.DriveTrain.kMaxAngularSpeed);

    // set goal of angle PID to be commanded bearing (in degrees) from joysticks
    m_targetAngle = new Rotation2d(filteredBearing);
    Rotation2d m_currentAngle = drivetrain.getPose().getRotation(); // from -Pi to Pi
    m_angleError = m_targetAngle;
    m_angleError.minus(m_currentAngle);
    intakeAnglePid.setSetpoint(m_targetAngle.getDegrees()); //PID already tuned in degrees
    rot = intakeAnglePid.calculate(m_currentAngle.getDegrees());
    
    //convert field centric speeds to robot centric
    ChassisSpeeds tempChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_currentAngle);

    //tip correction is in robot centric
    tempChassisSpeed.vxMetersPerSecond += pitch_correction;
    tempChassisSpeed.vyMetersPerSecond += roll_correction;

    output_states = kinematics
        .toSwerveModuleStates(tempChassisSpeed);

  }

  @Override
  public void execute() {
    calculate();
    drivetrain.drive(output_states);
    updateNT();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
    // update network tables
      NTTargetAngle.setDouble(m_targetAngle.getDegrees());
      NTangleError.setDouble(m_angleError.getDegrees());
    }
  }

  private Rotation2d getJoystickBearing(){
    //take joystick X and Y inputs (field centric space) and return an expected direction of travel (-Pi to Pi Rad)
    Rotation2d joystickBearing = new Rotation2d(Math.atan2(ySpeed, xSpeed));
    
    if(Math.abs(xSpeed) >= 0.05 || Math.abs(ySpeed) >= 0.05){ //if the joysticks aren't moved out of deadzone, just return last bearing.
      lastBearing = joystickBearing;
    } else {
      joystickBearing = lastBearing;
    }
    return joystickBearing;
  }

}
