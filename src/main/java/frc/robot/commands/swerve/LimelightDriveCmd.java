package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;

public class LimelightDriveCmd extends DriveCmd {
  PIDController limelightPid;
  double limelight_kP = 0.05;
  double limelight_kI = 0.0;
  double limelight_kD = 0.0;

  double limelightPidOutput = 0.0;

  Limelight_Subsystem limelight;
  SlewRateLimiter llLimiter = new SlewRateLimiter(3);

  public LimelightDriveCmd(SwerveDrivetrain drivetrain, DriverControls dc, Limelight_Subsystem limelight) {
    super(drivetrain, dc);
    this.limelight = limelight;
    limelightPid = new PIDController(limelight_kP, limelight_kI, limelight_kD);

    // display PID coefficients on SmartDashboard
    // SmartDashboard.putNumber("Limelight P Gain", limelight_kP);
    // SmartDashboard.putNumber("Limelight I Gain", limelight_kI);
    // SmartDashboard.putNumber("Limelight D Gain", limelight_kD);
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    //checkShooter();
    calculate(); // parent does most the work, sets output_states
    updateLimelightPID();
    
    if (((driveMode == DriveModeTypes.hubCentric)) &&
        limelight.getTarget() &&
        limelight.getLEDStatus()) {
      // we only use limelight in hubCentric mode
      limelightPid.setSetpoint(0); // always go towards the light.
      limelightPidOutput = limelightPid.calculate(limelight.getFilteredX());
      // update rotation and calulate new output-states
      rot = llLimiter.calculate(limelightPidOutput);
      //rot = limelightPidOutput;
      output_states = kinematics
          .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading));
    }
    //set the outputs and update the network tables.
    drivetrain.drive(output_states);
    updateNT();
  }

  void updateLimelightPID() {
    double limelight_p = SmartDashboard.getNumber("Limelight P Gain", limelight_kP);
    double limelight_i = SmartDashboard.getNumber("Limelight I Gain", limelight_kI);
    double limelight_d = SmartDashboard.getNumber("Limelight D Gain", limelight_kD);

    // if anything changes in limeLight PID, update it
    if ((limelight_p != limelight_kP) || (limelight_i != limelight_kI) || (limelight_d != limelight_kD)) {
      limelight_kP = limelight_p;
      limelight_kI = limelight_i;
      limelight_kD = limelight_d;
      limelightPid.setP(limelight_kP);
      limelightPid.setI(limelight_kI);
      limelightPid.setD(limelight_kD);
    }
  }

}
