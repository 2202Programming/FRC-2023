package frc.robot.subsystems;
import frc.robot.Constants.CAN;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

public class Arm extends SubsystemBase {
    //TODO use the Controllrt.calculate and Controller.setpoint
    //instance variables
    private PIDController left_arm_controller = new PIDController(0.0, 0.0, 0.0);
    private PIDController right_arm_controller = new PIDController(0.0, 0.0, 0.0);
    private PIDController arm_sync_controller = new PIDController(0.0, 0.0, 0.0);
    //TODO values of velocity PID, do we need different PID controllers because it is spring loaded?
    private double Current_pos; //current extension of arm in mm
    private double Desired_pos; // desired extension of arm in mm
    private CANSparkMax left_motor = new CANSparkMax(CAN.ARM_LEFT_Motor, MotorType.kBrushless); //TODO motor type
    private CANSparkMax right_motor = new CANSparkMax(CAN.ARM_RIGHT_Motor, MotorType.kBrushless); 

    public Arm(){
        //TODO networktable?

    }

    // DPL - TODO: reads the motor's postion to indicate where the arm is at
    public double getCurrent_pos(){
        return Current_pos;
    }

    public void setDesired_pos(double Desired_pos){
        this.Desired_pos = Desired_pos;
    }
    
    public double getDesired_pos(){
        return Desired_pos;
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
}
