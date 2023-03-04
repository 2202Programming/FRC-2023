package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;
import frc.robot.util.VelocityControlled;

public class Elbow extends SubsystemBase implements VelocityControlled {
  final int STALL_CURRENT = 40;
  final int FREE_CURRENT = 20;

  // mechanical gearing motor rotations to degrees with gear ratio
  final double conversionFactor = (360.0 / 210.0);

  // positionPID at position tolerances
  double posTol = 3.0; // [deg]
  double velTol = 2.0; // [deg/s]

  // motion speed limits
  double velLimit = 10.0; // [deg/s]
  double accelLimit = 5.0; // [deg/s^2] - only in future smartmode

  // ArbFeedforward to compensate for static torque
  double maxArbFF = 0.09; // [%power] -1.0 to 1.0  Tested with SMax Client %pwr mode

  // NeoServo - TODO (It's what arm values are rn, will need to change)
  final NeoServo elbow_servo;
  PIDController positionPID = new PIDController(1.0, 0.0, 0.0);  //(5.0, 0.150, 0.250);
  PIDFController hwVelPID = new PIDFController(0.005, 0.00005, 0.00, .003);   //0.003125);

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
    
    // Starting point
    elbow_servo.setPosition(0.0);
    
    // elbow rotates, define the range at +- 180 degrees
    positionPID.enableContinuousInput(-180.0, 180.0);
  }

  // @Override
  public void periodic() {
    // use our desired postion to set a min %pwr for gravity - sin(pos),
    // where 0 deg is hanging vertical. +-180 is vertical up.
    double arbFF = maxArbFF * Math.sin(Math.toRadians(elbow_servo.getSetpoint()));
    elbow_servo.setArbFeedforward(arbFF);
    elbow_servo.periodic();
  }

  public boolean atSetpoint() {
    return elbow_servo.atSetpoint();
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
