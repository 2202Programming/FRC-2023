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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NeoServo implements VelocityControlled {
    String name = "no-name";

    // commands
    double velocity_cmd; // computed from pid, or external_vel_cmd
    double maxVelocity; // limits
    double arbFeedforward = 0.0; // for specialized control cases
    double external_vel_cmd = 0.0; // for velocity_mode == true
    boolean velocity_mode = false;

    // measured values
    double currentPos;
    double currentVel;

    // state vars
    final public PIDController positionPID;
    final public PIDFController hwVelPIDcfg; // matches hardware setting
    final PIDFController prevVelPIDcfg;      // soft copy to edit /w NT and compare with hwVelPIDcfg
    final int hwVelSlot;

    // hardware
    final CANSparkMax ctrl;
    final SparkMaxPIDController pid;
    final RelativeEncoder encoder;

    public NeoServo(int canID, PIDController positionPID, PIDFController hwVelPIDcfg, boolean inverted) {
        this(canID, positionPID, hwVelPIDcfg, inverted, 0);
    }

    public NeoServo(int canID, PIDController positionPID, PIDFController hwVelPIDcfg, boolean inverted, int hwVelSlot) {
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
        this.hwVelPIDcfg = hwVelPIDcfg;
        this.prevVelPIDcfg = new PIDFController(hwVelPIDcfg);
    }

    public NeoServo setName(String name) {
        this.name  =name;
        return this;
    }

    // methods to tune the servo very SmartMax Neo specific
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
        // sets curren limits, but RPMLimit is not enabled, 10K is default
        return setSmartCurrentLimit(stallLimit, freeLimit, 10000);
    }

    public NeoServo setSmartCurrentLimit(int stallLimit, int freeLimit, int rpmLimit) {
        ctrl.setSmartCurrentLimit(stallLimit, freeLimit, rpmLimit);
        return this;
    }

    public NeoServo setVelocityHW_PID(double smVelMax, double smAccelMax) {
        // write the hwVelPIDcfgcfg constants to the sparkmax
        hwVelPIDcfg.copyTo(pid, hwVelSlot, smVelMax, smAccelMax);
        return this;
    }

    public NeoServo burnFlash() {
        ctrl.burnFlash();
        Timer.delay(.2); // this holds up the current thread
        return this;
    }

    // defers to setMaxVel(), but returns this for config chaining
    public NeoServo setMaxVelocity(double maxVelocity) {
        // defer to the VelocityControlled API
        setMaxVel(maxVelocity);
        return this;
    }

    public NeoServo setBrakeMode(CANSparkMax.IdleMode mode) {
        ctrl.setIdleMode(mode);
        return this;
    }

    /*
     * VelocityControlled API
     * 
     */
    // Servo's position setpoint
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
        encoder.setPosition(pos); // tell our encoder we are at pos
        positionPID.reset(); // clear any history in the pid
        positionPID.calculate(pos, pos); // tell our pid we want that position; measured, setpoint same
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

    public double getVelocityCmd() {
        return velocity_cmd;
    }

    void setArbFeedforward(double aff) {
        arbFeedforward = aff;
    }

    public void hold() {
        external_vel_cmd = 0.0;
        currentPos = encoder.getPosition();

        // set our setpoint, but stay in whatever control mode is being used
        positionPID.calculate(currentPos, currentPos);
        positionPID.reset();
    }

    public void periodic() {
        periodic(0.0);
    }

    public void periodic(double compAdjustment) {
        // measure -read encoder for current position and velocity
        currentPos = encoder.getPosition();
        currentVel = encoder.getVelocity();

        // velocity_mode, update position setpoint so we don't jump back on mode switch
        if (velocity_mode) {
            positionPID.setSetpoint(currentPos);
        }

        // calculate - run position pid to get velocity
        velocity_cmd = MathUtil.clamp(positionPID.calculate(currentPos) + compAdjustment, -maxVelocity, maxVelocity);

        // if velocity mode, use external_vel_cmd, otherwise use positionPID
        velocity_cmd = velocity_mode ? external_vel_cmd + compAdjustment : velocity_cmd;

        // command hard 0.0 if POS is at tollerence
        /// velocity_cmd = positionPID.atSetpoint() ? 0.0 : velocity_cmd;
        // output - send our vel to the controller

        // potential use of feedforward
        pid.setReference(velocity_cmd, ControlType.kVelocity, hwVelSlot, arbFeedforward);
    }

    public Command getWatcher() {
        var cmd = new NeoWatcher();
        cmd.schedule();
        return cmd;
    }

    class NeoWatcher extends CommandBase {
        NetworkTable table;
        NetworkTableEntry nt_currentPos;
        NetworkTableEntry nt_desiredPos;
        NetworkTableEntry nt_desiredVel;
        NetworkTableEntry nt_currentVel;

        NeoWatcher() {
            table = NetworkTableInstance.getDefault().getTable(name); 
            // keep updates even when disabled, self-decoration 
            this.runsWhenDisabled(); 
            ntcreate();
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            ntUpdates();
        }

        public void ntcreate() {
            nt_currentPos = table.getEntry("Position");
            nt_currentVel = table.getEntry("Velocity");
            nt_desiredPos = table.getEntry("PositionCmd");
            nt_desiredVel = table.getEntry("VelocityCmd");

            //put the a copy on dashboard to edit
            SmartDashboard.putData(name+"/hwVelPIDcfg", prevVelPIDcfg);
        }

        public void ntUpdates() {
            nt_currentPos.setDouble(getPosition());
            nt_currentVel.setDouble(getVelocity());
            nt_desiredPos.setDouble(getSetpoint());
            nt_desiredVel.setDouble(getVelocityCmd());

            //look for PIDF config changes
            hwVelPIDcfg.copyChangesTo(pid, hwVelSlot, prevVelPIDcfg);

        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }

}