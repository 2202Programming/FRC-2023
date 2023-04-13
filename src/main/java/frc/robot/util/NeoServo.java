// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.utility.WatcherCmd;

public class NeoServo implements VelocityControlled {
    String name = "no-name";

    // commands
    double velocity_cmd; // computed from pid, or external_vel_cmd
    double maxVelocity; // limits
    double initialMaxVelocity = 0.0; // keep the first non-zero as the hard max
    double arbFeedforward = 0.0; // for specialized control cases
    double external_vel_cmd = 0.0; // for velocity_mode == true
    boolean velocity_mode = false;
    double trim = 0.0; // offset from the commanded position (not seen in measured)
    double MIN_POS = -500.0, MAX_POS = 500.0; // PLEASE SET YOUR CLAMP VALUES

    // safety checks
    int NO_MOTION_FRAMES = 10;

    // measured values
    double currentPos;
    double currentVel;

    // safety checks on servo movement
    int safety_frame_count = 0;
    final int WARNING_MSG_FRAMES = 100;
    int warning_count = 0; // limits warning msg to once every N frames

    // state vars
    final PIDController positionPID;
    final public PIDFController hwVelPIDcfg; // matches hardware setting
    final PIDFController prevVelPIDcfg; // soft copy to edit /w NT and compare with hwVelPIDcfg
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

    // not really a NEO, but a sparkmax controller on brushed motor and alt encoder
    public NeoServo(int canID, PIDController positionPID, PIDFController hwVelPIDcfg, boolean inverted, int hwVelSlot,
            Type extEncoderType, int kCPR) {
        ctrl = new CANSparkMax(canID, MotorType.kBrushed);
        ctrl.clearFaults();
        ctrl.restoreFactoryDefaults();
        ctrl.setInverted(inverted);
        ctrl.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pid = ctrl.getPIDController();
        encoder = ctrl.getAlternateEncoder(extEncoderType, kCPR);

        this.positionPID = positionPID;
        this.hwVelSlot = hwVelSlot;
        this.hwVelPIDcfg = hwVelPIDcfg;
        this.prevVelPIDcfg = new PIDFController(hwVelPIDcfg);
    }

    public NeoServo setName(String name) {
        this.name = name;
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
        pos = MathUtil.clamp(pos, MIN_POS, MAX_POS);
        positionPID.setSetpoint(pos);
        velocity_mode = false;
        external_vel_cmd = 0.0;
    }

    public void setClamp(double min_pos, double max_pos) {
        MIN_POS = min_pos;
        MAX_POS = max_pos;
    }

    public boolean isVelocityMode() {
        return velocity_mode;
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
        positionPID.calculate(pos - trim, pos); // tell our pid we want that position; measured, setpoint same
    }

    public double getPosition() {
        return currentPos;
    }

    public void setMaxVel(double v) {
        v = Math.abs(v);
        if (initialMaxVelocity == 0.0)
            initialMaxVelocity = v; // saving the initial as hard max
        maxVelocity = (v <= initialMaxVelocity) ? v : initialMaxVelocity;
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

    public void setArbFeedforward(double aff) {
        // uses percent power, on range of -1. to 1.0
        if (Math.abs(aff) > 1.0) {
            DriverStation.reportError("|ArbFF| > 1, check your math. Using ZERO.", true);
            arbFeedforward = 0.0;
        } else
            arbFeedforward = aff;
    }

    public void setTrim(double trim) {
        this.trim = trim;
    }

    public double getTrim() {
        return trim;
    }

    public void hold() {
        external_vel_cmd = 0.0;
        currentPos = encoder.getPosition();

        // set our setpoint, but stay in whatever control mode is being used
        positionPID.calculate(currentPos, currentPos);
    }

    public void clearHwPID() {
        ctrl.getPIDController().setIAccum(0.0);
    }

    /*
     * isStalled()
     * 
     * Looks for motion on the servo to ensure we are not stalled.
     * 
     * return -
     * false => servo is moving correctly
     * true => servo is stalled for N frames or more, cut the motor in periodic()
     */
    boolean isStalled() {
        boolean not_moving = (Math.abs(velocity_cmd) > positionPID.getVelocityTolerance()) && // motion requested
                (Math.abs(currentVel) < positionPID.getVelocityTolerance()) && // motion not seen
                (!positionPID.atSetpoint()) &&
                (DriverStation.isEnabled()); // is enabled

        // count frames we aren't moving
        safety_frame_count = not_moving ? ++safety_frame_count : 0;

        return safety_frame_count > NO_MOTION_FRAMES;
    }

    public void periodic() {
        periodic(0.0);
    }

    public void periodic(double compAdjustment) {
        // measure -read encoder for current position and velocity
        currentPos = encoder.getPosition() - trim;
        currentVel = encoder.getVelocity();

        // velocity_mode, update position setpoint so we don't jump back on mode switch
        if (velocity_mode) {
            // 4/10/2023 dpl positionPID.reset();
            positionPID.setSetpoint(currentPos);
        }

        // calculate - run position pid to get velocity
        velocity_cmd = MathUtil.clamp(positionPID.calculate(currentPos) + compAdjustment, -maxVelocity, maxVelocity);

        // if velocity mode, use external_vel_cmd, otherwise use positionPID
        velocity_cmd = velocity_mode ? external_vel_cmd + compAdjustment : velocity_cmd;
        
        //local copy of arbFeedforward incase stall has to zero it and it isn't set every frame by class user
        double arbFF = arbFeedforward;   

        // confirm we are moving and not stalled
        if (isStalled()) {
            // issue stall warning, but not every frame
            if ((warning_count++ % WARNING_MSG_FRAMES) == 0) {
                DriverStation.reportError(name + " servo stalled at pos=" + currentPos +
                        " set point=" + positionPID.getSetpoint() +
                        " velocity_cmd=" + velocity_cmd +
                        " measured_vel=" + currentVel, false);
                // stalled for NO_MOTION_FRAMES frames, stop trying to move
                setSetpoint(currentPos); // stay where we are 
                velocity_cmd = 0.0;
                arbFF = 0.0;
            } else {
                // moving, clear the warning counter
                warning_count = 0;
            }
        }
        // potential use of feedforward
        pid.setReference(velocity_cmd, ControlType.kVelocity, hwVelSlot, arbFF, ArbFFUnits.kPercentOut);
    }


    public void simulationPeriodic() {
        // nothing to do if we are not enabled
        if (!DriverStation.isEnabled()) 
            return;
        //simple model - encoder vel is set in sim when a velocity mode is used
        //so move the position based on velocity being commanded
        // no dynamics are modeled 
        double pos = encoder.getPosition() + encoder.getVelocity() * Constants.DT; 
        encoder.setPosition(pos);
    }

    public Command getWatcher() {
        return new NeoWatcher();
    }

    class NeoWatcher extends WatcherCmd {
        NetworkTableEntry nt_arbFF;
        NetworkTableEntry nt_currentPos;
        NetworkTableEntry nt_desiredPos;
        NetworkTableEntry nt_desiredVel;
        NetworkTableEntry nt_currentVel;
        NetworkTableEntry nt_trim;

        @Override
        public String getTableName() {
            return name; // from NeoServo
        }

        @Override
        public void ntcreate() {
            NetworkTable table = getTable();
            nt_arbFF = table.getEntry("ArbFF");
            nt_currentPos = table.getEntry("Position");
            nt_currentVel = table.getEntry("Velocity");
            nt_desiredPos = table.getEntry("PositionCmd");
            nt_desiredVel = table.getEntry("VelocityCmd");
            nt_trim = table.getEntry("Trim");

            // put the a copy on dashboard to edit
            SmartDashboard.putData(name + "/hwVelPIDcfg", prevVelPIDcfg);
        }

        @Override
        public void ntupdate() {
            nt_arbFF.setDouble(arbFeedforward);
            nt_currentPos.setDouble(getPosition());
            nt_currentVel.setDouble(getVelocity());
            nt_desiredPos.setDouble(getSetpoint());
            nt_desiredVel.setDouble(getVelocityCmd());
            nt_trim.setDouble(trim);

            // look for PIDF config changes
            hwVelPIDcfg.copyChangesTo(pid, hwVelSlot, prevVelPIDcfg);
        }
    }

}