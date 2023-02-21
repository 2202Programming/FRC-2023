package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.util.PIDFController;

/*
 * 1/27/23 notes from MrL
 * 
 * Motor controllers should be in velocity mode and will need PID values for the SparkMax
 *  Use a outer position loop PID in the sub-system.  Also we could look at Position control in HW or smartmotion.
 * 
 *  Consider an Arm class that does one arm and an ArmSS that instantiates left/right arm
 *    The arm class would have all the details in it, no left/right. Sub-sys has left/right arm.
 *    I started the code going down this path, should make the SS more readable.
 *
 *  Desired positon wouldn't be a SubSystem Argument. It would be part of api.
 */

public class ArmSS extends SubsystemBase {

    /**
     * Arm - does one arm
     */
    class Arm {
        // commands
        double velCmd; // [cm/s] computed
        final double gearRadius = 2.63398 * 2 * Math.PI; // [cm] .22 and .0037
        final double gearRatio = (1.0 / 75.0);

        // measured values
        double currentPos;

        // state vars
        PIDController positionPID = new PIDController(7.0, 0.150, 0.0); // outer position loop
        PIDFController hwVelPID = new PIDFController(0.002141, 0.00005, 0.15, 0.05017); // holds values for hardware
        final int hwVelSlot = 0;

        // Testing Mode
        boolean velocity_mode = false;
        double external_vel_cmd = 0.0;

        // hardware
        final CANSparkMax ctrl;
        final SparkMaxPIDController pid;
        final RelativeEncoder encoder;

        Arm(int canID) {
            // use canID to get controller and supporting objects
            ctrl = new CANSparkMax(canID, MotorType.kBrushless);
            ctrl.clearFaults();
            ctrl.restoreFactoryDefaults();
            ctrl.setIdleMode(CANSparkMax.IdleMode.kBrake);
            pid = ctrl.getPIDController();
            encoder = ctrl.getEncoder();
            positionPID.setTolerance(posTol, velTol);

            ctrl.setSmartCurrentLimit(30, 15);
            encoder.setPositionConversionFactor(gearRatio * gearRadius);
            encoder.setVelocityConversionFactor(gearRatio * gearRadius / 60); // rpm to rps

            // write the hwVelPID constants to the sparkmax
            hwVelPID.copyTo(pid, hwVelSlot, 50, 5);

            ctrl.burnFlash();
            Timer.delay(.2); // this holds up the current thread
        }

        Arm(int canID, CANSparkMax boss, boolean inverted) {
            this(canID);
        }

        // control the arm's postion [cm]
        void setSetpoint(double x_cm) {
            positionPID.setSetpoint(x_cm);
            velocity_mode = false;
            external_vel_cmd = 0.0;
        }

        // where we want to be is tracked by positionPID setpoint
        double getSetpoint() {
            return positionPID.getSetpoint();
        }

        boolean atSetpoint() {
            return positionPID.atSetpoint();
        }

        // Sets the position of the physical position (Doesn't move anything)
        void setPosition(double x_cm) {
            encoder.setPosition(x_cm);
            positionPID.reset();
            setSetpoint(x_cm);
        }

        double getPosition() {
            return currentPos;
        }

        void setMaxVel(double v) {
            maxVel = Math.abs(v);
        }

        double getMaxVel() {
            return maxVel;
        }

        void setVelocityCmd(double v_cm) {
            velocity_mode = true;
            external_vel_cmd = v_cm;
        }

        void hold() {
            pid.setReference(0.0, ControlType.kVelocity);
            currentPos = encoder.getPosition();
            setSetpoint(currentPos);
            positionPID.reset();
            positionPID.calculate(currentPos);
        }

        void periodic(double compAdjustment) {
            // read encoder for current position
            currentPos = encoder.getPosition();
            // run position pid to get velocity
            velCmd = MathUtil.clamp(positionPID.calculate(currentPos) + compAdjustment, -maxVel, maxVel);
            // command hard 0.0 if POS is at tollerence
            velCmd = positionPID.atSetpoint() ? 0.0 : velCmd;

            // if velocity mode, use the maxVel to control it, otherwise use positionPID
            velCmd = velocity_mode ? external_vel_cmd : velCmd;

            // send our vel to the controller
            pid.setReference(velCmd, ControlType.kVelocity);
        }
    } // End of Arm Class

    final boolean invert_left = true;

    // instance variables
    // State vars
    final Arm leftArm;
    final Arm rightArm;
    boolean follow_mode;

    // constants
    double maxVel = 15.0; // [cm/s]
    double posTol = 0.30; // [cm]
    double velTol = 0.25; // [cm/s]

    // sync instance vars
    boolean sync = true; // should usually be true, option to change to false for testing purposes
    double syncCompensation; // amount of compensation [m/s]

    // controllers
    PIDController syncPID = new PIDController(0.25, 0.0, 0.0); // arm synchronization pid. syncs left --> right

    public ArmSS() {
        rightArm = new Arm(CAN.ARM_RIGHT_Motor);
        // left may follow right
        leftArm = new Arm(CAN.ARM_LEFT_Motor, rightArm.ctrl, invert_left);
        sync = false;
        // zero our encoders at power up
        setPositions(0.0);
        ntcreate();
    }

    public void setFollowMode(boolean follow) {
        if (follow) {
            leftArm.ctrl.follow(rightArm.ctrl, invert_left);
            follow_mode = true;
            sync = false; // no software pid sync
        } else {
            leftArm.ctrl.follow(leftArm.ctrl);
            leftArm.ctrl.setInverted(invert_left);
            follow_mode = false;
        }
    }

    // At Position flags for use in the commands
    public boolean armsAtPosition() {
        if (follow_mode)
            return rightArm.atSetpoint();
        else
            return (rightArm.atSetpoint() && leftArm.atSetpoint());
    }

    public void setVelocityLimit(double vel_limit) {
        leftArm.setMaxVel(vel_limit);
        rightArm.setMaxVel(vel_limit);
    }

    public double getVelocityLimit() {
        // arms should have same vel_limit
        return leftArm.getMaxVel();
    }

    public void hold() {
        leftArm.hold();
        rightArm.hold();
    }

    /*
     * Looks at various pids and desired positions to see if we are there
     */

    @Override
    public void periodic() {
        // Synchronization
        syncCompensation = sync ? syncPID.calculate(leftArm.currentPos, rightArm.currentPos) / 2.0 : 0;
        leftArm.periodic(syncCompensation);
        rightArm.periodic(-syncCompensation);

        ntUpdates();
    }

    public void setSetpoints(double extension) {
        leftArm.setSetpoint(extension);
        rightArm.setSetpoint(extension);
    }

    // initializes position, doesn't move anything. Defines zero or whereever you.
    public void setPositions(double extension) {
        leftArm.setPosition(extension);
        rightArm.setPosition(extension);
    }

    // Testing mode, use with care since there are no limit switches
    public void setVelocityCmd(double vel_cm) {
        double v = MathUtil.clamp(vel_cm, -maxVel, maxVel);
        leftArm.setVelocityCmd(v);
        rightArm.setVelocityCmd(v);
    }

    //rarely used, testing only
    public void setVelocityLeft(double speed) {
        leftArm.setVelocityCmd(speed);
    }
    public void setVelocityRight(double speed) {
        rightArm.setVelocityCmd(speed);
    }

    /******************
     * Network Table Stuff.
     * 
     * Should most of this be in the arm class? Probably.
     * Is it easier to look at and fix if it's in the ArmSS class? Probably.
     *************/
    NetworkTable table = NetworkTableInstance.getDefault().getTable("arm");

    // PID controllers
    NetworkTableEntry nt_left_kP;
    NetworkTableEntry nt_left_kI;
    NetworkTableEntry nt_left_kD;

    NetworkTableEntry nt_right_kP;
    NetworkTableEntry nt_right_kI;
    NetworkTableEntry nt_right_kD;

    NetworkTableEntry nt_sync_kP;
    NetworkTableEntry nt_sync_kI;
    NetworkTableEntry nt_sync_kD;

    // positions/vels
    NetworkTableEntry nt_left_desiredPos;
    NetworkTableEntry nt_left_currentPos;
    NetworkTableEntry nt_left_desiredVel;
    NetworkTableEntry nt_left_currentVel;

    NetworkTableEntry nt_right_desiredPos;
    NetworkTableEntry nt_right_currentPos;
    NetworkTableEntry nt_right_desiredVel;
    NetworkTableEntry nt_right_currentVel;

    // maxs, mins, tols
    NetworkTableEntry nt_maxVel;
    NetworkTableEntry nt_posTol;
    NetworkTableEntry nt_velTol;

    NetworkTableEntry nt_syncCompensation;

    public void ntcreate() {
        // PIDs
        nt_left_kP = table.getEntry("Left kP");
        nt_left_kI = table.getEntry("Left kI");
        nt_left_kD = table.getEntry("Left kD");

        nt_right_kP = table.getEntry("Right kP");
        nt_right_kI = table.getEntry("Right kI");
        nt_right_kD = table.getEntry("Right kD");

        nt_sync_kP = table.getEntry("Sync kP");
        nt_sync_kI = table.getEntry("Sync kI");
        nt_sync_kD = table.getEntry("Sync kD");

        // des/cur pos/vel
        nt_left_desiredPos = table.getEntry("Left Desired Position");
        nt_left_currentPos = table.getEntry("Left Current Position");
        nt_left_desiredVel = table.getEntry("Left Desired Velocity");
        nt_left_currentVel = table.getEntry("Left Current Velocity");

        nt_right_desiredPos = table.getEntry("right Desired Position");
        nt_right_currentPos = table.getEntry("right Current Position");
        nt_right_desiredVel = table.getEntry("right Desired Velocity");
        nt_right_currentVel = table.getEntry("right Current Velocity");

        nt_syncCompensation = table.getEntry("Sync Compensation");

        // maxs, mins, tols
        nt_maxVel = table.getEntry("Max Velocity (cm/s)");
        nt_posTol = table.getEntry("Position Tolerance (cm)");
        nt_velTol = table.getEntry("Velocity Tolerance (cm/s)");

        // set doubles for values that we will update based on what is in NT, so they
        // appear
        nt_left_kP.setDouble(leftArm.positionPID.getP());
        nt_left_kI.setDouble(leftArm.positionPID.getI());
        nt_left_kD.setDouble(leftArm.positionPID.getD());

        nt_right_kP.setDouble(rightArm.positionPID.getP());
        nt_right_kI.setDouble(rightArm.positionPID.getI());
        nt_right_kD.setDouble(rightArm.positionPID.getD());

        nt_sync_kP.setDouble(syncPID.getP());
        nt_sync_kI.setDouble(syncPID.getI());
        nt_sync_kD.setDouble(syncPID.getD());

        nt_maxVel.setDouble(maxVel);
        nt_posTol.setDouble(posTol);
        nt_velTol.setDouble(velTol);
    }

    private void ntUpdates() {
        // info (get)
        nt_left_desiredPos.setDouble(leftArm.getSetpoint());
        nt_left_currentPos.setDouble(leftArm.currentPos);
        nt_left_desiredVel.setDouble(leftArm.velCmd);
        nt_left_currentVel.setDouble(leftArm.encoder.getVelocity());

        nt_right_desiredPos.setDouble(rightArm.getSetpoint());
        nt_right_currentPos.setDouble(rightArm.currentPos);
        nt_right_desiredVel.setDouble(rightArm.velCmd);
        nt_right_currentVel.setDouble(rightArm.encoder.getVelocity());

        nt_syncCompensation.setDouble(syncCompensation);

        // PID setters
        leftArm.positionPID.setP(nt_left_kP.getDouble(0.0));
        leftArm.positionPID.setI(nt_left_kI.getDouble(0.0));
        leftArm.positionPID.setD(nt_left_kD.getDouble(0.0));

        rightArm.positionPID.setP(nt_right_kP.getDouble(0.0));
        rightArm.positionPID.setI(nt_right_kI.getDouble(0.0));
        rightArm.positionPID.setD(nt_right_kD.getDouble(0.0));

        syncPID.setP(nt_sync_kP.getDouble(0.0));
        syncPID.setI(nt_sync_kI.getDouble(0.0));
        syncPID.setD(nt_sync_kD.getDouble(0.0));

        leftArm.positionPID.setTolerance(nt_posTol.getDouble(0.5), nt_velTol.getDouble(0.5));
        rightArm.positionPID.setTolerance(nt_posTol.getDouble(0.5), nt_velTol.getDouble(0.5));
    }

}
