package frc.robot.subsystems;

import frc.robot.Constants.CAN;
import frc.robot.util.PIDFController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

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
    // constants
    double MaxVel = 20.0; // [cm/s] //todo Networktable?
    double PosTol = .2; // [cm]
    double VelTol = .5; // [cm/s]

    /**
     * Arm - does one arm
     */
    class Arm {
        // commands
        double maxVel = MaxVel; // [cm/s] extension rate limit
        double velCmd;          // [cm/s] computed

        // measured values
        double currentPos;

        // state vars
        PIDController positionPID = new PIDController(0.0, 0.0, 0.0); // outer position loop
        PIDFController hwPID = new PIDFController(0.0, 0.0, 0.0, 0.0); // holds values for hwpid vel

        // hardware
        final CANSparkMax ctrl;
        final SparkMaxPIDController pid;
        final RelativeEncoder encoder;

        Arm(int canID) {
            // use canID to get controller and supporting objects
            ctrl = new CANSparkMax(canID, MotorType.kBrushless);
            ctrl.restoreFactoryDefaults();
            ctrl.setIdleMode(CANSparkMax.IdleMode.kBrake);
            pid = ctrl.getPIDController();
            encoder = ctrl.getEncoder();

            // todo: set scaling on endoder to use cm
            encoder.setPositionConversionFactor(TBD);
            encoder.setVelocityConversionFactor(TBD);

            // write the hwPID constants to the sparkmax
            hwPID.copyTo(pid, 0);

            // finish the position pid for outer loop
            positionPID.setTolerance(PosTol, VelTol);
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

        // we can change the max vel if we need to.
        void setMaxVel(double v) {
            maxVel = Math.abs(v);
        }

        void periodic() {
            // read encoder for current position
            currentPos = encoder.getPosition();
            // run position pid to get velocity
            velCmd = MathUtil.clamp(positionPID.calculate(currentPos), -maxVel, maxVel);
            // command hard 0.0 if POS is at tollerence
            velCmd = positionPID.atSetpoint() ? 0.0 : velCmd;
            // send our vel to the controller
            pid.setReference(velCmd, ControlType.kSmartVelocity); // todo can we use position or SmartMotion modes?
        }
    }


    //FEEDBACK from Mr.L  - rework this, most of it in now in Arm.
    // think about how to control the two arms?
    
    // TODO use the Controllrt.calculate and Controller.setpoint
    // instance variables
    // private PIDController left_arm_controller = new PIDController(0.0, 0.0, 0.0);
    // private PIDController right_arm_controller = new PIDController(0.0, 0.0,
    // 0.0);
    // private PIDController arm_sync_controller = new PIDController(0.0, 0.0, 0.0);
    private double currentPos; // current extension of arm in mm
    private double desiredPos; // desired extension of arm in mm
    private CANSparkMax leftMotor = new CANSparkMax(CAN.ARM_LEFT_Motor, MotorType.kBrushless); // TODO motor type
    private CANSparkMax rightMotor = new CANSparkMax(CAN.ARM_RIGHT_Motor, MotorType.kBrushless);
    private double motorSpeed;
    private double current_vel; // current velocity of arm
    private double desired_vel; // desired velocity of arm
    private double tolerance; // tolerance

    // State vars
    final Arm leftArm;
    final Arm rightArm;

    public ArmSS() {
        leftArm = new Arm(CAN.ARM_LEFT_Motor);
        rightArm = new Arm(CAN.ARM_RIGHT_Motor);

    }

    // Next think about what to do with the two arm objects

    public double getCurrentVel() {
        return current_vel;
    }

    public double getDesiredVel() {
        return desired_vel;
    }

    public double getTolerance() {
        return tolerance;
    }

    public void setDesiredVel(double Desired_vel) {
        this.desired_vel = Desired_vel;
    }

    public double getCurrentPos() {
        return currentPos;
    }

    public void setDesiredPos(double desiredPos) {
        this.desiredPos = desiredPos;
    }

    public double getdesiredPos() {
        return desiredPos;
    }

    /*
     * Looks at various pids and desired positions to see if we are there
     */
    public boolean isAtPosition() {
        return (Math.abs(currentPos - desiredPos) >= tolerance);
    }

    public boolean motorDirectionForwards() {
        return (currentPos < desiredPos);
    }

    @Override
    public void periodic() {
        leftArm.periodic();
        rightArm.periodic();

        // TODO sync compenstation

        boolean armCheck = isAtPosition();
        if (armCheck = true) {
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }

        //feedback from MrL  - we have better controllers, 
        if (armCheck = false) {
            boolean forward = motorDirectionForwards();
            if (forward = true) {
                leftMotor.set(motorSpeed);
                rightMotor.set(motorSpeed);
            }
            if (forward = false) {
                leftMotor.set(0 - motorSpeed);
                rightMotor.set(0 - motorSpeed);
            }
        }
    }

    /******************
     * Network Table Stuff
     *************/
    NetworkTable table = NetworkTableInstance.getDefault().getTable("arm");

    NetworkTableEntry nt_left_kP;
    NetworkTableEntry nt_left_kI;
    NetworkTableEntry nt_left_kD;
    NetworkTableEntry nt_right_kP;
    NetworkTableEntry nt_right_kI;
    NetworkTableEntry nt_right_kD;
    NetworkTableEntry nt_sync_kP;
    NetworkTableEntry nt_sync_kI;
    NetworkTableEntry nt_sync_kD;
    NetworkTableEntry nt_desiredPos;
    NetworkTableEntry nt_currentPos;
    NetworkTableEntry nt_desiredVel;
    NetworkTableEntry nt_currentVel;
    NetworkTableEntry nt_tolerance;

    public void ntcreate() {
        nt_left_kP = table.getEntry("Left kP");
        nt_left_kI = table.getEntry("Left kI");
        nt_left_kD = table.getEntry("Left kD");
        nt_right_kP = table.getEntry("Right kP");
        nt_right_kI = table.getEntry("Right kI");
        nt_right_kD = table.getEntry("Right kD");
        nt_sync_kP = table.getEntry("Sync kP");
        nt_sync_kI = table.getEntry("Sync kI");
        nt_sync_kD = table.getEntry("Sync kD");
        nt_desiredPos = table.getEntry("Desired Position");
        nt_currentPos = table.getEntry("Current Position");
        nt_desiredVel = table.getEntry("Desired Velocity");
        nt_currentVel = table.getEntry("Current Velocity");
        nt_tolerance = table.getEntry("Tolerance");
        nt_left_kP.setDouble(0.0);
        nt_left_kI.setDouble(0.0);
        nt_left_kD.setDouble(0.0);
        nt_right_kP.setDouble(0.0);
        nt_right_kI.setDouble(0.0);
        nt_right_kD.setDouble(0.0);
        nt_sync_kP.setDouble(0.0);
        nt_sync_kI.setDouble(0.0);
        nt_sync_kD.setDouble(0.0);
    }

    private void ntUpdates() {
        // info (set)
        nt_desiredPos.setDouble(desiredPos);
        nt_currentPos.setDouble(currentPos);
        nt_desiredVel.setDouble(desired_vel);
        nt_currentVel.setDouble(current_vel);
        nt_tolerance.setDouble(tolerance);

        // PID setters
        /**
         * left_arm_controller.setP(nt_left_kP.getDouble(0.0));
         * left_arm_controller.setI(nt_left_kI.getDouble(0.0));
         * left_arm_controller.setD(nt_left_kD.getDouble(0.0));
         * right_arm_controller.setP(nt_right_kP.getDouble(0.0));
         * right_arm_controller.setI(nt_right_kI.getDouble(0.0));
         * right_arm_controller.setD(nt_right_kD.getDouble(0.0));
         * sync_arms_controller.setP(nt_sync_kP.getDouble(0.0));
         * sync_arms_controller.setI(nt_sync_kI.getDouble(0.0));
         * sync_arms_controller.setD(nt_sync_kD.getDouble(0.0));
         */

    }

}
