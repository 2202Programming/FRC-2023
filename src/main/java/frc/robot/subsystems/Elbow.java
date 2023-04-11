package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PowerOnPos;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;
import frc.robot.util.VelocityControlled;

public class Elbow extends SubsystemBase implements VelocityControlled {
  final int STALL_CURRENT = 40;
  final int FREE_CURRENT = 20;
  final double ELBOW_MIN_DEG = -20.0;
  final double ELBOW_MAX_DEG = 110.0;
 
  // mechanical gearing motor rotations to degrees with gear ratio
  final double conversionFactor = (360.0 / 350.0); //orig. 20% bump (5:7:10)

  // positionPID at position tolerances
  double posTol = 3.0; // [deg]
  double velTol = 2.0; // [deg/s]

  // motion speed limits
  double velLimit = 120.0; // [deg/s]
  double accelLimit = 5.0; // [deg/s^2] - only in future smartmode

  // ArbFeedforward to compensate for static torque
  double maxArbFF = 0.09; // [%power] -1.0 to 1.0  Tested with SMax Client %pwr mode

  //trim stuff
  double trim_decrement = 0.5; // degrees
  double trim_increment = 1.0; // degrees
  double Ktrim = 14.0; //6,7 orig

  // NeoServo - TODO (It's what arm values are rn, will need to change) - verify fix is complete
  final NeoServo elbow_servo;
  PIDController positionPID = new PIDController(6.0, 0.083, 0.0);  
  PIDFController hwVelPID = new PIDFController(0.0042, 0.0000052, 0.00, .003);

  public Elbow() {
    elbow_servo = new NeoServo(CAN.ELBOW_Motor, positionPID, hwVelPID, true);
    // finish the config
    elbow_servo.setName(getName()+"/servo")
        .setConversionFactor(conversionFactor)
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(velLimit, accelLimit)
        .setMaxVelocity(velLimit)
        .setTolerance(posTol, velTol)
        .burnFlash();
    elbow_servo.setBrakeMode(IdleMode.kCoast);// TEMP FOR TESTING
    elbow_servo.setClamp(ELBOW_MIN_DEG, ELBOW_MAX_DEG);

    // Starting point
    elbow_servo.setPosition(PowerOnPos.elbow);

  }

  // @Override
  public void periodic() {
    // use our desired postion to set a min %pwr for gravity - sin(pos),
    // where 0 deg is hanging vertical. +-180 is vertical up.
    double arbFF = maxArbFF * Math.sin(Math.toRadians(elbow_servo.getSetpoint()));
    //don't use arbff if using velocity mode
    arbFF = (elbow_servo.isVelocityMode()) ? 0.0 : arbFF;

    double trim = Ktrim*Math.sin(Math.toRadians(elbow_servo.getPosition()));
    elbow_servo.setTrim( trim );

    elbow_servo.setArbFeedforward(arbFF);
    elbow_servo.periodic();
  }
  public void incrementTrim(){
    Ktrim += trim_increment;
  }
  public void decrementTrim(){
    Ktrim -= trim_decrement;
  }

  public double getTrim() {
    return Ktrim;
  }

  public void setTrim(double trim){
    Ktrim = trim;
  }

  public boolean atSetpoint() {
    return elbow_servo.atSetpoint();
  }

  public void setClamp(double min_pos, double max_pos){
    elbow_servo.setClamp(min_pos, max_pos);
  }

  public void setSetpoint(double degrees) {
    elbow_servo.setSetpoint(degrees);
  }

  // sets encoder to a default positon, doesn't move anything
  public void setPosition(double degrees) {
    elbow_servo.setPosition(degrees);
  }

  public double getSetpoint() {
    return elbow_servo.getSetpoint();
  }

  public double getPosition() {
    return elbow_servo.getPosition();
  }

  public void setVelocityCmd(double vel) {
    elbow_servo.setVelocityCmd(vel);
  }

  public double getVelocity() {
    return elbow_servo.getVelocity();
  }

  public double getVelocityCmd() {
    return getVelocityCmd();
  }

  public double getMaxVel() {
    return elbow_servo.getMaxVel();
  }

  public void setMaxVel(double vel) {
    elbow_servo.setMaxVel(vel);
  }

  public void hold() {
    elbow_servo.hold();
  }

  public Command getWatcher() {
    var cmd = new Watcher();
    cmd.schedule();
    return cmd;
  }

  class Watcher extends CommandBase {
    NetworkTable table;
    NetworkTableEntry nt_arbFF;

    Watcher() {
      // create and start the servo's watcher cmd
      elbow_servo.getWatcher();
      this.runsWhenDisabled();
      ntcreate();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      ntupdates();
    }

    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }

    public void ntcreate() {
      table = NetworkTableInstance.getDefault().getTable(Elbow.this.getName());
      nt_arbFF = table.getEntry("maxArbFF");

      //put the initial values for mutables
      nt_arbFF.setDouble(maxArbFF);
    }

    public void ntupdates() {
      //get mutable values
      maxArbFF = nt_arbFF.getDouble(maxArbFF);
    }
  }

}
