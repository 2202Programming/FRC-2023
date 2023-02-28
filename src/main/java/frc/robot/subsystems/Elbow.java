package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;
import frc.robot.util.VelocityControlled;

public class Elbow extends SubsystemBase implements VelocityControlled {
  int STALL_CURRENT = 30;
  int FREE_CURRENT = 30;

  double maxVel = 5.0; // [deg/s]
  double maxAccel = 5.0; // [deg/s^2]
  final double conversionFactor = (360.0 / 175.0);

  // positionPID at position limits
  static double posLimit = 3.0; // [deg]
  static double velLimit = 5.0; // [deg/s]

  // NeoServo
  final NeoServo elbow_servo;
  // TODO (It's what arm values are rn, will need to change)
  PIDController positionPID = new PIDController(5.0, 0.150, 0.250);
  PIDFController hwVelPID = new PIDFController(0.002141, 0.00005, 0.15, 0.05017);

  public Elbow() {
    elbow_servo = new NeoServo(CAN.ELBOW_Motor, positionPID, true);
    // finish the config
    elbow_servo
        .setConversionFactor(conversionFactor)
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(hwVelPID, maxVel, maxAccel)
        .setMaxVel(maxVel);
    elbow_servo.positionPID.setTolerance(posLimit, velLimit);
    ntcreate();
  }

  // @Override
  public void periodic() {
    elbow_servo.periodic();
    ntupdates();
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

  NetworkTable table = NetworkTableInstance.getDefault().getTable("elbow");
  NetworkTableEntry nt_currentPos;
  NetworkTableEntry nt_desiredPos;
  NetworkTableEntry nt_desiredVel;
  NetworkTableEntry nt_currentVel;

  public void ntcreate() {
    nt_currentPos = table.getEntry("Position");
    nt_currentVel = table.getEntry("Velocity");
    nt_desiredPos = table.getEntry("PositionCmd");
    nt_desiredVel = table.getEntry("VelocityCmd");

  }

  public void ntupdates() {
    nt_currentPos.setDouble(elbow_servo.getPosition());
    nt_currentVel.setDouble(elbow_servo.getVelocity());
    nt_desiredPos.setDouble(elbow_servo.getSetpoint());
    nt_desiredVel.setDouble(elbow_servo.getVelocityCmd());
  }

}
