package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrain;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

/*
  Driver controls the robot using field coordinates.
    X,Y, Rotation
*/
public class FieldCentricDrive extends CommandBase {

  final SwerveDrivetrain drivetrain;
  final SwerveDriveKinematics kinematics;
  final HID_Xbox_Subsystem dc;
  final AntiTip antiTip = RobotContainer.RC().antiTip;

  // output to Swerve Drivetrain
  double xSpeed, ySpeed, rot;
  Rotation2d currrentHeading;
  SwerveModuleState[] output_states;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  
  public FieldCentricDrive(SwerveDrivetrain drivetrain) {
    this.dc = RobotContainer.RC().dc;       //driverControls
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    this.kinematics = drivetrain.getKinematics();
  }

  @Override
  public void initialize() {
  }

  void calculate() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
    ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;
    rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed;

    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    rot = MathUtil.clamp(rot, -Constants.DriveTrain.kMaxAngularSpeed, Constants.DriveTrain.kMaxAngularSpeed);

    
    currrentHeading = drivetrain.getPose().getRotation();
    //convert field centric speeds to robot centric
    ChassisSpeeds tempChassisSpeed = (DriverStation.getAlliance().equals(Alliance.Blue)) 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed + antiTip.getFieldCentricTipCorrection().vxMetersPerSecond,
                                                ySpeed + antiTip.getFieldCentricTipCorrection().vyMetersPerSecond,
                                                rot, currrentHeading) 
        : ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed + antiTip.getFieldCentricTipCorrection().vxMetersPerSecond,
                                                -ySpeed + antiTip.getFieldCentricTipCorrection().vyMetersPerSecond,
                                                rot, currrentHeading); // if on red alliance you're looking at robot from opposite. Pose is in blue coordinates so flip if red

    output_states = kinematics
        .toSwerveModuleStates(tempChassisSpeed);
  }

  @Override
  public void execute() {
    antiTip.tipCalculate();
    calculate();
    drivetrain.drive(output_states);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

}