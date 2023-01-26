package frc.robot.subsystems;
import frc.robot.Constants.CAN;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Arm extends SubsystemBase {
    //TODO use the Controllrt.calculate and Controller.setpoint
    //instance variables
    private PIDController left_arm_controller = new PIDController(0.0, 0.0, 0.0);
    private PIDController right_arm_controller = new PIDController(0.0, 0.0, 0.0);
    private PIDController sync_arms_controller = new PIDController(0.0, 0.0, 0.0);
    //TODO values of velocity PID, do we need different PID controllers because it is spring loaded?
    private double current_pos; //current extension of arm in mm
    private double desired_pos; // desired extension of arm in mm
    private double current_vel; //current velocity of arm
    private double desired_vel; //desired velocity of arm
    private double tolerance; //tolerance
    private CANSparkMax left_motor = new CANSparkMax(CAN.ARM_LEFT_Motor, MotorType.kBrushless); //TODO motor type
    private CANSparkMax right_motor = new CANSparkMax(CAN.ARM_RIGHT_Motor, MotorType.kBrushless); 
    public Arm(){
        
        //TODO networktable?



    }

    // DPL - TODO: reads the motor's postion to indicate where the arm is at
    public double getCurrentPos(){
        return current_pos;
    }

    public void setDesiredPos(double Desired_pos){
        this.desired_pos = Desired_pos;
    }
    
    public double getDesiredPos(){
        return desired_pos;
    }

    /*
     * Looks at various pids and desired positions to see if we are there
     */
    public boolean isAtPosition()
    {
        return false;  //TODO fix me
    }

    @Override
    public void periodic(){

    }


    /******************
     *Network Table Stuff 
     *************/
//TODO updates
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
        nt_desiredPos.setDouble(desired_pos);
        nt_currentPos.setDouble(current_pos);
        nt_desiredVel.setDouble(desired_vel);
        nt_currentVel.setDouble(current_vel);
        nt_tolerance.setDouble(tolerance);

        //PID setters
        left_arm_controller.setP(nt_left_kP.getDouble(0.0));
        left_arm_controller.setI(nt_left_kI.getDouble(0.0));
        left_arm_controller.setD(nt_left_kD.getDouble(0.0));
        right_arm_controller.setP(nt_right_kP.getDouble(0.0));
        right_arm_controller.setI(nt_right_kI.getDouble(0.0));
        right_arm_controller.setD(nt_right_kD.getDouble(0.0));   
        sync_arms_controller.setP(nt_sync_kP.getDouble(0.0));
        sync_arms_controller.setI(nt_sync_kI.getDouble(0.0));
        sync_arms_controller.setD(nt_sync_kD.getDouble(0.0));


     }
        
     }


