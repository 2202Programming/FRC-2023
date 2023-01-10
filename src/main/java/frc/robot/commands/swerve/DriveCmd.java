package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
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

public class DriveCmd extends CommandBase {

  public enum DriveModeTypes {
    robotCentric("Robot Centric"),
    fieldCentric("Field Centric"),
    hubCentric("Hub Centric"),
    intakeCentric("Intake Centric");

    private String name;

    private DriveModeTypes(String name) {
      this.name = name;
    }

    public String toString() {
      return name;
    }
  }

  final SwerveDrivetrain drivetrain;
  final DriverControls dc;
  final SwerveDriveKinematics kinematics;
  // command behaviors
  DriveModeTypes driveMode = DriveModeTypes.fieldCentric;
  DriveModeTypes lastDriveMode = DriveModeTypes.fieldCentric;
  boolean fieldRelativeMode = false;

  boolean lastShootMode = false;
  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  Rotation2d currrentHeading;
  SwerveModuleState[] output_states;

  // PID for heading to a target
  private PIDController anglePid;
  private PIDController intakeAnglePid;
  private double angle_kp = 0.075;
  private double angle_ki = 0.004;
  private double angle_kd = 0.005;

  private double lastBearing; //stores the last significant bearing angle

  private Pose2d centerField = new Pose2d(27, 13.5, new Rotation2d()); //actual
  // hub location?
  //private Pose2d centerField = new Pose2d(10, 0, new Rotation2d()); // close point for testing to max rotation obvious

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  NetworkTable table;
  private NetworkTableEntry driveCmd;
  private NetworkTableEntry fieldMode;
  private NetworkTableEntry hubCentricTarget;
  private NetworkTableEntry xVelTarget;
  private NetworkTableEntry yVelTarget;
  private NetworkTableEntry rotVelTarget;
  private NetworkTableEntry NTangleError;
  private NetworkTableEntry xJoystick;
  private NetworkTableEntry yJoystick;
  
  public final String NT_Name = "DT"; // expose data under DriveTrain table

  double log_counter = 0;

  // Creates a new Single-Pole IIR filter
  // Time constant is 0.1 seconds
  // Period is 0.02 seconds - this is the standard FRC main loop period
  private LinearFilter bearingFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
  private double filteredBearing = 0;

  public DriveCmd(SwerveDrivetrain drivetrain, DriverControls dc2) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.dc = dc2;
    this.kinematics = drivetrain.getKinematics();

    anglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    intakeAnglePid = new PIDController(angle_kp, angle_ki, angle_kd);
    intakeAnglePid.enableContinuousInput(-180, 180);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    hubCentricTarget = table.getEntry("/hubCentricTarget");;
    fieldMode = table.getEntry("/FieldMode");

    xVelTarget = table.getEntry("/xVelTarget");
    yVelTarget = table.getEntry("/yVelTarget");
    rotVelTarget = table.getEntry("/rotVelTarget");
    NTangleError = table.getEntry("/angleError");
    xJoystick = table.getEntry("/xJoystick");
    yJoystick = table.getEntry("/yJoystick");
    driveCmd = table.getEntry("/driveCmd");
    //NTLastDriveMode = table.getEntry("/LastDriveMode");
  }

  public DriveCmd(SwerveDrivetrain drivetrain, DriverControls dc, boolean fieldRelativeMode) {
    this(drivetrain, dc);
    this.fieldRelativeMode = fieldRelativeMode;
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

    filteredBearing = bearingFilter.calculate(getJoystickBearing());

    if ((Math.abs(rot)>0.1) && (driveMode==DriveModeTypes.intakeCentric)){
      //driver is trying to rotate, drop out of intake mode
      driveMode=DriveModeTypes.fieldCentric;
      lastDriveMode=driveMode;
    }

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    rot = MathUtil.clamp(rot, -Constants.DriveTrain.kMaxAngularSpeed, Constants.DriveTrain.kMaxAngularSpeed);

    currrentHeading = drivetrain.getPose().getRotation();

    // Now workout drive mode behavior
    switch (driveMode) {
      case robotCentric:
        output_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        break;

      case fieldCentric:
        output_states = kinematics
            .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));
        break;

      case hubCentric:
        rot = 0;
        // set goal of angle PID to be heading (in degrees) from current position to
        // centerfield
        double targetAngle = getHeading2Target(drivetrain.getPose(), centerField);
        targetAngle = targetAngle + 180; // flip since shooter is on "back" of robot
        if(targetAngle > 180){
          targetAngle = targetAngle - 360;
        }
        double currentAngle = drivetrain.getPose().getRotation().getDegrees(); // from -180 to 180
        double angleError = targetAngle - currentAngle;
        // feed both PIDs even if not being used.
        anglePid.setSetpoint(targetAngle);
        rot = anglePid.calculate(currentAngle);
        
        // deal with continuity issue across 0
        if (angleError < -180) {
          targetAngle += 360;
        }
        if (angleError > 180) {
          targetAngle -= 360;
        }
        hubCentricTarget.setValue(targetAngle);
        NTangleError.setDouble(angleError);

        output_states = kinematics
            .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));
        break;

      case intakeCentric:
        // set goal of angle PID to be commanded bearing (in degrees) from joysticks
        double m_targetAngle2 = filteredBearing;
        double m_currentAngle2 = drivetrain.getPose().getRotation().getDegrees(); // from -180 to 180
        double m_angleError2 = m_targetAngle2 - m_currentAngle2;
        // feed both PIDs even if not being used.
        intakeAnglePid.setSetpoint(m_targetAngle2);
        rot = intakeAnglePid.calculate(m_currentAngle2);
        
        // deal with continuity issue across 0
        if (m_angleError2 < -180) {
          m_targetAngle2 += 360;
        }
        if (m_angleError2 > 180) {
          m_targetAngle2 -= 360;
        }
        hubCentricTarget.setValue(m_targetAngle2);
        NTangleError.setDouble(m_angleError2);

        output_states = kinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));

      break;
    }
  }

  @Override
  public void execute() {
    //checkShooter();
    calculate();
    drivetrain.drive(output_states);
    updateNT();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // takes 2 positions, gives heading from current point to target (in degrees)
  double getHeading2Target(Pose2d current, Pose2d target) {
    // from -PI to +PI
    double theta = Math.atan2(target.getY() - current.getY(), target.getX() - current.getX());

    // convert this to degrees in the range -180 to 180
    theta = Math.toDegrees(theta);
    return theta;
  }

  public void toggleFieldRealitiveMode() {
    if (fieldRelativeMode)
      fieldRelativeMode = false;
    else
      fieldRelativeMode = true;
    fieldMode.setBoolean(fieldRelativeMode);
    return;
  }

  public void cycleDriveMode() {
    //Current use case is only to allow toggling between field and intake centric
    //Make sure if in hubcentric (trigger held) that toggling doesn't do anything
    switch (driveMode) {
      //case robotCentric:
      //  driveMode = DriveModeTypes.fieldCentric;
      //  drivetrain.setDriveModeString("Robot Centric Drive");
      //  break;
      case fieldCentric:
        lastDriveMode = driveMode;
        driveMode = DriveModeTypes.intakeCentric;
        break;
      //case hubCentric:
      //  driveMode = DriveModeTypes.intakeCentric;
      //  drivetrain.setDriveModeString("Hub Centric Drive");
      //  break;
      case intakeCentric:
        lastDriveMode = driveMode;
        driveMode = DriveModeTypes.fieldCentric;
        break;
        default:
          break;    // DPL 2/21/22 - for some reason we don't need other modes - can this get moved to Commands?
    }
  }

  public DriveModeTypes getDriveMode() {
    return driveMode;
  }

  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
    // update network tables
    xJoystick.setDouble(dc.getVelocityX());
    yJoystick.setDouble(dc.getVelocityY());
    xVelTarget.setValue(xSpeed);
    yVelTarget.setValue(ySpeed);
    rotVelTarget.setValue(rot);
    //fieldMode.setBoolean(fieldRelativeMode);
    driveCmd.setString("DriveCmd");
    //NTLastDriveMode.setString(lastDriveMode.toString());
    }
  }

  // public void checkShooter(){
  //   boolean shootingModeOn = drivetrain.getShootingMode();
  //     if (lastShootMode != shootingModeOn) {//shoot mode has changed
  //     if(shootingModeOn){ //switched to shooting mode; hub centric mode while in shooting mode
  //       lastDriveMode = driveMode; //save current drive mode to restore later
  //       driveMode = DriveModeTypes.hubCentric;
  //       drivetrain.setDriveModeString("Shooting mode");
  //     } else { //switched out of shooting mode
  //       driveMode = lastDriveMode; //revert to pre-shooting drive mode
  //       drivetrain.setDriveModeString(driveMode.toString());
  //     }
  //   }   
  //   lastShootMode = shootingModeOn;
  // }

  private double getJoystickBearing(){
    //take joystick X and Y inputs (field centric space) and return an expected direction of travel (-180 to 180 degrees)
    double joystickBearing = 0;
    joystickBearing = Math.atan2(ySpeed, xSpeed);
    
    if(Math.abs(xSpeed) >= 0.05 || Math.abs(ySpeed) >= 0.05){
      lastBearing = joystickBearing;
    } else {
      joystickBearing = lastBearing;
    }

    // if the joystick has an x or y value above an index (doing something), save the bearing
    // as the last bearing. If not, use last bearing.
    
    // if (xSpeed > 0) { //0 to 180
    //   if (ySpeed > 0) { //0 to 90
    //     joystickBearing = Math.atan(ySpeed / xSpeed);
    //   } else { //90 to 180
    //     joystickBearing = Math.atan(-ySpeed / xSpeed) + 90;
    //   }
    // } else { //0 to -180
    //   if (ySpeed > 0) { //0 to -90
    //     joystickBearing = -Math.atan(ySpeed / -xSpeed);
    //   } else { //-90 to -180
    //     joystickBearing = -(Math.atan(-ySpeed / -xSpeed) + 90);
    //   }
    // }
    return Math.toDegrees(joystickBearing);
  }
}
