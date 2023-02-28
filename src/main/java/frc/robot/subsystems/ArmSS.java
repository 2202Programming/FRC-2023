package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;
import frc.robot.util.VelocityControlled;

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

public class ArmSS extends SubsystemBase implements VelocityControlled {
    int STALL_CURRENT = 40;
    int FREE_CURRENT = 40;

    final double gearRadius = 2.63398 * 2 * Math.PI; //2.633[cm] is drive gear radius
    final double gearRatio = (1.0 / 75.0);
    final double conversionFactor = gearRadius*gearRatio;
 
    // state vars
    PIDController positionPID_rt = new PIDController(5.0, 0.150, 0.250); // outer position loop
    PIDController positionPID_lt = new PIDController(5.0, 0.150, 0.250); // outer position loop
    PIDFController hwVelPID = new PIDFController(0.002141, 0.00005, 0.15, 0.05017); // holds values for hardware
    final int hwVelSlot = 0;

    // positive extension moves arm out
    final boolean invert_left = false;
    final boolean invert_right = true;

    // instance variables
    // State vars
    final NeoServo leftArm;
    final NeoServo rightArm;
    boolean follow_mode;

    // PID and speed constants
    double maxVel = 15.0; // [cm/s]
    double maxAccel = 20.0; // [cm/s^2] Not enforce until smartmode used
    double posTol = 0.30; // [cm]
    double velTol = 0.25; // [cm/s]

    // sync instance vars
    boolean sync = false; // don't think we need this, +- 3mm without.
    double syncCompensation; // amount of compensation [m/s]
    
    // computed values from position pid
    double error;

    // controllers
    PIDController syncPID = new PIDController(0.0, 0.0, 0.0); // arm synchronization pid. syncs left --> right

    public ArmSS() {
        rightArm = new NeoServo(CAN.ARM_RIGHT_Motor, positionPID_rt, invert_right);
        leftArm = new NeoServo(CAN.ARM_LEFT_Motor, positionPID_lt, invert_left);
        configure(rightArm);
        configure(leftArm);

        // zero our encoders at power up
        setPosition(0.0);
        
        ntcreate();
    }
    // finish the arm servo configurations
    void configure(NeoServo arm) {
        arm.setConversionFactor(conversionFactor)
        .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
        .setVelocityHW_PID(hwVelPID, maxVel, maxAccel);
        arm.setMaxVel(maxVel);
        arm.positionPID.setTolerance(posTol, velTol);
    }

    /*
     * Looks at various pids and desired positions to see if we are there
     */

    @Override
    public void periodic() {
        // Synchronization, drive left to follow right
        error = rightArm.getPosition() - leftArm.getPosition();
        syncCompensation = sync ? syncPID.calculate(leftArm.getPosition(), rightArm.getPosition()) : 0.0;
        leftArm.periodic(syncCompensation);
        rightArm.periodic(0.0);
        ntUpdates();
    }

    // At Position flags for use in the commands
    public boolean atSetpoint() {
        return (rightArm.atSetpoint() && leftArm.atSetpoint());
    }

    public void setMaxVel(double vel_limit) {
        leftArm.setMaxVel(vel_limit);
        rightArm.setMaxVel(vel_limit);
    }

    public double getMaxVel() {
        // arms should have same vel_limit
        return rightArm.getMaxVel();
    }

    public void hold() {
        leftArm.hold();
        rightArm.hold();
    }

    public void setSetpoint(double extension) {
        leftArm.setSetpoint(extension);
        rightArm.setSetpoint(extension);
        sync = true;
    }

    public double getSetpoint() {
        // arms track, so just use right one
        return rightArm.getSetpoint();
    }

    public double getPosition() {
        return (leftArm.getPosition() + rightArm.getPosition()) / 2.0;
    }

    // initializes position, doesn't move anything. Defines zero or whereever you.
    public void setPosition(double extension) {
        leftArm.setPosition(extension);
        rightArm.setPosition(extension);
    }

    // Testing mode, use with care since there are no limit switches
    public void setVelocityCmd(double vel_cm) {
        double v = MathUtil.clamp(vel_cm, -maxVel, maxVel);
        leftArm.setVelocityCmd(v);
        rightArm.setVelocityCmd(v);
    }

    public double getVelocityCmd() {
        return (leftArm.getVelocityCmd() + rightArm.getVelocityCmd())/2.0;
    }

    public double getVelocity() {
        return (leftArm.getVelocity() + rightArm.getVelocity()) / 2.0;
    }

    // rarely used, testing only since arms are bolted together
    public void setVelocityLeft(double speed) {
        leftArm.setVelocityCmd(speed);
        sync = false;
    }

    public void setVelocityRight(double speed) {
        rightArm.setVelocityCmd(speed);
        sync = false;
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

    NetworkTableEntry nt_error;
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
        nt_error = table.getEntry("Pos Error");

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
        nt_error.setDouble(error);
    }

    private void ntUpdates() {
        // info (get)
        nt_left_desiredPos.setDouble(leftArm.getSetpoint());
        nt_left_currentPos.setDouble(leftArm.getPosition());
        nt_left_desiredVel.setDouble(leftArm.getVelocityCmd());
        nt_left_currentVel.setDouble(leftArm.getVelocity());

        nt_right_desiredPos.setDouble(rightArm.getSetpoint());
        nt_right_currentPos.setDouble(rightArm.getPosition());
        nt_right_desiredVel.setDouble(rightArm.getVelocityCmd());
        nt_right_currentVel.setDouble(rightArm.getVelocity());
        nt_error.setDouble(error);
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

        /// leftArm.positionPID.setTolerance(nt_posTol.getDouble(0.5),
        /// nt_velTol.getDouble(0.5));
        /// rightArm.positionPID.setTolerance(nt_posTol.getDouble(0.5),
        /// nt_velTol.getDouble(0.5));
    }

}
