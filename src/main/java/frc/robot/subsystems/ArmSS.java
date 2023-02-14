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
        final double gearRadius = 2.63398 * 2 * Math.PI; //[cm]
        final double gearRatio = (1/5) * 3.35; //3.35 fudge factor
        // measured values
        double currentPos;

        // state vars
        PIDController positionPID = new PIDController(5.0, 0.0, 0.0); // outer position loop
        PIDFController hwVelPID = new PIDFController(0.001, 0.0, 0.0, 0.0); // holds values for hwVelpid vel
        final int hwVelSlot = 0;

        // hardware
        final CANSparkMax ctrl;
        final SparkMaxPIDController pid;
        final RelativeEncoder encoder;

        Arm(int canID) {
            // use canID to get controller and supporting objects
            //TODO: Set current limit
            ctrl = new CANSparkMax(canID, MotorType.kBrushless);
            ctrl.restoreFactoryDefaults();
            ctrl.setIdleMode(CANSparkMax.IdleMode.kBrake);
            pid = ctrl.getPIDController();
            encoder = ctrl.getEncoder();
            positionPID.setTolerance(posTol, velTol);

            encoder.setPositionConversionFactor(gearRatio * gearRadius); 
            encoder.setVelocityConversionFactor(gearRatio * gearRadius / 60); //rpm to rps 

            // write the hwVelPID constants to the sparkmax
            hwVelPID.copyTo(pid, hwVelSlot);

        }

        // control the arm's postion [cm]
        void setSetpoint(double x) {
            positionPID.setSetpoint(x);
        }

        // where we want to be is tracked by positionPID setpoint
        double getSetpoint() {
            return positionPID.getSetpoint();
        }

        boolean atSetpoint() {
            return positionPID.atSetpoint();
        }
        void setPosition(double x){
            encoder.setPosition(x); //Sets the position of the physical position (Doesn't move anything)
            positionPID.reset();
            setSetpoint(x);
        }


        // we can change the max vel if we need to.
        // TODO: update with proper value in constants, use the constants value, remove
        // setter here
        void setMaxVel(double v) {
            maxVel = Math.abs(v);
        }
        double getPosition(){
            return currentPos;
        }
        void off() {
            ctrl.set(0);
           
        }

        void periodic(double compAdjustment) {
            // read encoder for current position
            currentPos = encoder.getPosition();
            // run position pid to get velocity
            velCmd = MathUtil.clamp(positionPID.calculate(currentPos) + compAdjustment, -maxVel, maxVel);
            // command hard 0.0 if POS is at tollerence
            velCmd = positionPID.atSetpoint() ? 0.0 : velCmd;
            // send our vel to the controller
            pid.setReference(velCmd, ControlType.kVelocity); // TODO can we use position or SmartMotion modes?

        }
        
        /**
         * NTs
         */

    } //End of Arm Class

    // instance variables
    // State vars
    final Arm leftArm;
    final Arm rightArm;

    // constants
    double maxVel = 1.0; // [cm/s]
    double posTol = .2; // [cm]
    double velTol = .05; // [cm/s]

    // sync instance vars
    boolean sync = true; // should usually be true, option to change to false for testing purposes
    double syncCompensation; // amount of compensation [m/s]

    // controllers
    PIDController syncPID = new PIDController(0.25, 0.0, 0.0); // arm synchronization pid. syncs left --> right

    public ArmSS() {
        leftArm = new Arm(CAN.ARM_LEFT_Motor);
        rightArm = new Arm(CAN.ARM_RIGHT_Motor);
        leftArm.setPosition(0.0); //Change if we end up starting somewhere else
        rightArm.setPosition(0.0); //Change if we end up starting somewhere else
        ntcreate();
    }
    // At Position flags for use in the commands
    public boolean armsAtPosition() {
        return ((leftArm.atSetpoint()) && (rightArm.atSetpoint()));
    }

    public void off() {
        leftArm.off();
        rightArm.off();
    }
    /*
     * Looks at various pids and desired positions to see if we are there
     */

    @Override
    public void periodic() {
        // Synchronization
        syncCompensation = sync ? syncPID.calculate(leftArm.currentPos, rightArm.currentPos) / 2.0 : 0;
        syncCompensation = 0.0;
        leftArm.periodic(syncCompensation);
        rightArm.periodic(-syncCompensation);

        ntUpdates();
    }

    public void setPositions(double extension) {
        leftArm.setSetpoint(extension);
        rightArm.setSetpoint(extension);
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
