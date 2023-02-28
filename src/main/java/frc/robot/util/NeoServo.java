// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class NeoServo implements VelocityControlled {

    // command
    double velCmd; // computed
    double maxVelocity;

    // measured values
    double currentPos;
    double currentVel;

    // state vars
    final public PIDController positionPID;
    public PIDFController hwVelPID;
    final int hwVelSlot;

    // Testing Mode
    boolean velocity_mode = false;
    double external_vel_cmd = 0.0;

    // hardware
    final CANSparkMax ctrl;
    final SparkMaxPIDController pid;
    final RelativeEncoder encoder;

    public NeoServo(int canID, PIDController positionPID ,boolean inverted) {
        this(canID, positionPID, inverted, 0);
    }
    public NeoServo(int canID, PIDController positionPID ,boolean inverted, int hwVelSlot) {
        // use canID to get controller and supporting objects
        ctrl = new CANSparkMax(canID, MotorType.kBrushless);
        ctrl.clearFaults();
        ctrl.restoreFactoryDefaults();
        ctrl.setInverted(inverted);
        ctrl.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pid = ctrl.getPIDController();
        encoder = ctrl.getEncoder();
        this.positionPID = positionPID;
        this.hwVelSlot = hwVelSlot;
    }

    // methods to tune the servo
    public NeoServo setConversionFactor(double conversionFactor) {
        encoder.setPositionConversionFactor(conversionFactor);
        encoder.setVelocityConversionFactor(conversionFactor / 60);
        return this;
    }

    public NeoServo setTolerance(double posTol, double velTol) {
        positionPID.setTolerance(posTol, velTol);
        return this;
    }

    public NeoServo setSmartCurrentLimit(int stallLimit, int freeLimit) {
        ctrl.setSmartCurrentLimit(45, 20);
        return this;
    }

    public NeoServo setVelocityHW_PID(PIDFController hwpid, double smVelMax, double smAccelMax ) {
        // write the hwVelPID constants to the sparkmax
        hwVelPID.copyTo(pid, hwVelSlot, smVelMax, smAccelMax);
        ctrl.burnFlash();
        Timer.delay(.2); // this holds up the current thread
        return this;
    }

    // control Servo's setpoint
    public void setSetpoint(double pos) {
        positionPID.setSetpoint(pos);
        velocity_mode = false;
        external_vel_cmd = 0.0;
    }

    public double getSetpoint() {
        return positionPID.getSetpoint();
    }

    public boolean atSetpoint() {
        return positionPID.atSetpoint();
    }

    // Sets the encoder position (Doesn't move anything)
    public void setPosition(double pos) {
        encoder.setPosition(pos);
        positionPID.reset();
        setSetpoint(pos);
    }

    public double getPosition() {
        return currentPos;
    }

    public void setMaxVel(double v) {
        maxVelocity = Math.abs(v);
    }

    public double getMaxVel() {
        return maxVelocity;
    }

    public void setVelocityCmd(double vel) {
        velocity_mode = true;
        external_vel_cmd = vel;
    }

    public double getVelocity() {
        return currentVel;
    }

    public double getVelocityCmd(){
        return velCmd;
    }

    public void hold() {
        pid.setReference(0.0, ControlType.kVelocity);
        currentPos = encoder.getPosition();
        setSetpoint(currentPos);
        positionPID.reset();
        positionPID.calculate(currentPos);
    }

    public void periodic() {
        periodic(0.0);
    }

    public void periodic(double compAdjustment) {
        // meaure -read encoder for current position
        currentPos = encoder.getPosition();
        currentVel = encoder.getVelocity();

        // calculate - run position pid to get velocity
        velCmd = MathUtil.clamp(positionPID.calculate(currentPos) + compAdjustment, -maxVelocity, maxVelocity);
        // command hard 0.0 if POS is at tollerence
        velCmd = positionPID.atSetpoint() ? 0.0 : velCmd;

        // if velocity mode, use the maxVel to control it, otherwise use positionPID
        velCmd = velocity_mode ? external_vel_cmd + compAdjustment : velCmd;

        // output - send our vel to the controller
        pid.setReference(velCmd, ControlType.kVelocity);
    }
} // End of Arm Class
