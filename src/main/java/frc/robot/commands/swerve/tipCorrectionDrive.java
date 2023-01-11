package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;

/* Current driving behavior:
  Starts in field centric
  B will toggle between field centric and intake centric
  Holding right trigger will switch to hub centric until you let go, then it will go back to original mode
          (either field or intake centric, depending what you started in)
  If in intake centric and you try to rotate with left joystick, will drop back to field centric mode.
*/


public class tipCorrectionDrive extends FieldCentricDrive {

  double log_counter = 0;

  PIDController tipRollPid;
  double roll_kP = 0.05;
  double roll_kI = 0.0;
  double roll_kD = 0.0;
  double tipRollPidOutput = 0.0;

  PIDController tipPitchPid;
  double pitch_kP = 0.05;
  double pitch_kI = 0.0;
  double pitch_kD = 0.0;
  double tipPitchPidOutput = 0.0;

  double requested_pitch_P = pitch_kP;
  double requested_pitch_I = 0.0;
  double requested_pitch_D = 0.0;

  double requested_roll_P = roll_kP;
  double requested_roll_I = 0.0;
  double requested_roll_D = 0.0;

  NetworkTable table;
  NetworkTableEntry nt_roll_factor;
  NetworkTableEntry nt_pitch_factor;
  public final String NT_Name = "DC"; 

  public tipCorrectionDrive(SwerveDrivetrain drivetrain, DriverControls dc) {
    super(drivetrain, dc);
    addRequirements(drivetrain);
    tipRollPid = new PIDController(roll_kP, roll_kI, roll_kD);
    tipPitchPid = new PIDController(pitch_kP, pitch_kI, pitch_kD);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    nt_roll_factor = table.getEntry("/DriveController/RollFactor");
    nt_pitch_factor = table.getEntry("/DriveController/PitchFactor");

    SmartDashboard.putNumber("Requested Pitch P", requested_pitch_P);
    SmartDashboard.putNumber("Requested Pitch I", requested_pitch_I);
    SmartDashboard.putNumber("Requested Pitch D", requested_pitch_D);
    SmartDashboard.putNumber("Requested Roll P", requested_roll_P);
    SmartDashboard.putNumber("Requested Roll I", requested_roll_I);
    SmartDashboard.putNumber("Requested Roll D", requested_roll_D);
    
  }

  @Override
  public void initialize() {
    System.out.println("***STARTING TIP CORRECTION MODE");
  }

  void calculate() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    //xSpeed = xspeedLimiter.calculate(dc.getVelocityX()) * DriveTrain.kMaxSpeed;
    //ySpeed = yspeedLimiter.calculate(dc.getVelocityY()) * DriveTrain.kMaxSpeed;
    //rot = rotLimiter.calculate(dc.getXYRotation()) * DriveTrain.kMaxAngularSpeed;


    //For Pitch/Roll PID Tuning, comment out when done.
    requested_pitch_P = SmartDashboard.getNumber("Requested Pitch P", requested_pitch_P);
    requested_pitch_I = SmartDashboard.getNumber("Requested Pitch I", requested_pitch_I);
    requested_pitch_D = SmartDashboard.getNumber("Requested Pitch D", requested_pitch_D);
    requested_roll_P = SmartDashboard.getNumber("Requested Roll P", requested_roll_P);
    requested_roll_I = SmartDashboard.getNumber("Requested Roll I", requested_roll_I);
    requested_roll_D = SmartDashboard.getNumber("Requested Roll D", requested_roll_D);

    if (requested_pitch_P != tipPitchPid.getP()){
      tipPitchPid.setP(requested_pitch_P);
      System.out.println("****Tip Pitch P adjusted to: " + tipPitchPid.getP());
    }
    if (requested_pitch_I != tipPitchPid.getI()){
      tipPitchPid.setI(requested_pitch_I);
      System.out.println("****Tip Pitch I adjusted to: " + tipPitchPid.getI());
    }
    if (requested_pitch_D != tipPitchPid.getD()){
      tipPitchPid.setD(requested_pitch_D);
      System.out.println("****Tip Pitch D adjusted to: " + tipPitchPid.getD());
    }

    if (requested_roll_P != tipRollPid.getP()){
      tipRollPid.setP(requested_roll_P);
      System.out.println("****Tip Roll P adjusted to: " + tipRollPid.getP());
    }
    if (requested_roll_I != tipRollPid.getI()){
      tipRollPid.setI(requested_roll_I);
      System.out.println("****Tip Roll I adjusted to: " + tipRollPid.getI());
    }
    if (requested_roll_D != tipRollPid.getD()){
      tipRollPid.setD(requested_roll_D);
      System.out.println("****Tip Roll D adjusted to: " + tipRollPid.getD());
    }

    //NOTE: PITCH IS FRONT/BACK OF ROBOT, Positive towards intake
    //NOTE: ROLL IS POSITIVE WITH CLOCKWISE ROTATION (LOOKING FROM BACK TOWARDS INTAKE)
    //Y direction is left/right, positive towards left when facing intake from back

    double pitchAngleDegrees = RobotContainer.RC().sensors.getPitch();
    double rollAngleDegrees = RobotContainer.RC().sensors.getRoll();
    SmartDashboard.putNumber("Actual Pitch", pitchAngleDegrees);
    SmartDashboard.putNumber("Actual Roll", rollAngleDegrees);

    //Simulation mode, comment out when wanting real gryo data
    //pitchAngleDegrees = SmartDashboard.getNumber("Simulated Pitch", 0);
    //rollAngleDegrees = SmartDashboard.getNumber("Simulated Roll", 0);

    SmartDashboard.putNumber("Current Pitch", pitchAngleDegrees);
    SmartDashboard.putNumber("Current Roll", rollAngleDegrees);

    tipRollPid.setSetpoint(0); 
    tipRollPidOutput = tipRollPid.calculate(rollAngleDegrees);
    nt_roll_factor.setDouble(tipRollPidOutput);

    tipPitchPid.setSetpoint(0); 
    tipPitchPidOutput = tipPitchPid.calculate(-pitchAngleDegrees);
    nt_pitch_factor.setDouble(tipPitchPidOutput);

    xSpeed = 0.0; //tipPitchPidOutput;
    ySpeed = tipRollPidOutput;
    rot = 0.0;


    // Clamp speeds/rot from the Joysticks
    xSpeed = MathUtil.clamp(xSpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    ySpeed = MathUtil.clamp(ySpeed, -Constants.DriveTrain.kMaxSpeed, Constants.DriveTrain.kMaxSpeed);
    rot = MathUtil.clamp(rot, -Constants.DriveTrain.kMaxAngularSpeed, Constants.DriveTrain.kMaxAngularSpeed);

    output_states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
  }

  @Override
  public void execute() {
    calculate();
    drivetrain.drive(output_states);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("***STOPPING TIP CORRECTION MODE");
    drivetrain.stop();
  }

}
