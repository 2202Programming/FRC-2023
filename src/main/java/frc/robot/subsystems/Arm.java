package frc.robot.subsystems;
import frc.robot.Constants.CAN;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;


public class Arm extends SubsystemBase {
    //TODO use the Controllrt.calculate and Controller.setpoint
    //instance variables
    //private PIDController left_arm_controller = new PIDController(0.0, 0.0, 0.0);
    //private PIDController right_arm_controller = new PIDController(0.0, 0.0, 0.0);
    //private PIDController arm_sync_controller = new PIDController(0.0, 0.0, 0.0);
    private double currentPos; //current extension of arm in mm
    private double desiredPos; // desired extension of arm in mm
    private CANSparkMax leftMotor = new CANSparkMax(CAN.ARM_LEFT_Motor, MotorType.kBrushless); //TODO motor type
    private CANSparkMax rightMotor = new CANSparkMax(CAN.ARM_RIGHT_Motor, MotorType.kBrushless); 
    private double motorSpeed;
    private double current_vel; //current velocity of arm
    private double desired_vel; //desired velocity of arm
    private double tolerance; //tolerance
    public Arm(double currentPos, double desiredPos, CANSparkMax leftMotor, CANSparkMax rightMotor, double tolerance, double motorSpeed){
        //TODO networktable?
        this.motorSpeed = motorSpeed;
        this.tolerance = tolerance;
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.currentPos = currentPos;
        this.desiredPos = desiredPos;
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    // DPL - TODO: reads the motor's postion to indicate where the arm is at

    public double getCurrentVel(){
        return current_vel;
    }
    public double getDesiredVel(){
        return desired_vel;
    }
    public double getTolerance(){
        return tolerance;
    }
    public void setDesiredVel(double Desired_vel){
        this.desired_vel = Desired_vel;
    }
    public double getCurrentPos(){
        return currentPos;
    }

    public void setDesiredPos(double desiredPos){
        this.desiredPos = desiredPos;
    }
    
    public double getdesiredPos(){
        return desiredPos;
    }

    /*
     * Looks at various pids and desired positions to see if we are there
     */
    public boolean isAtPosition(){
        return (Math.abs(currentPos - desiredPos) >= tolerance);
    }

    public boolean motorDirectionForwards(){
        return (currentPos < desiredPos);
    }

    @Override
    public void periodic(){
        boolean armCheck = isAtPosition();
        if(armCheck = true){
            leftMotor.stopMotor();
            rightMotor.stopMotor();
        }
        if (armCheck = false){
            boolean forward = motorDirectionForwards();
            if (forward = true){
                leftMotor.set(motorSpeed);
                rightMotor.set(motorSpeed);
            }
            if (forward = false){
                leftMotor.set(0 - motorSpeed);
                rightMotor.set(0 - motorSpeed);
            }
        }
    }


    /******************
     *Network Table Stuff 
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


     public void ntcreate(){
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

     private void ntUpdates(){
        //info (set)
        nt_desiredPos.setDouble(desiredPos);
        nt_currentPos.setDouble(currentPos);
        nt_desiredVel.setDouble(desired_vel);
        nt_currentVel.setDouble(current_vel);
        nt_tolerance.setDouble(tolerance);

        //PID setters
        /**
        left_arm_controller.setP(nt_left_kP.getDouble(0.0));
        left_arm_controller.setI(nt_left_kI.getDouble(0.0));
        left_arm_controller.setD(nt_left_kD.getDouble(0.0));
        right_arm_controller.setP(nt_right_kP.getDouble(0.0));
        right_arm_controller.setI(nt_right_kI.getDouble(0.0));
        right_arm_controller.setD(nt_right_kD.getDouble(0.0));   
        sync_arms_controller.setP(nt_sync_kP.getDouble(0.0));
        sync_arms_controller.setI(nt_sync_kI.getDouble(0.0));
        sync_arms_controller.setD(nt_sync_kD.getDouble(0.0));
         */


     }
        
     }


