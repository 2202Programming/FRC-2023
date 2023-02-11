package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;


public class Limelight_Helpers_util extends SubsystemBase{
    LimelightResults llresults; 
    private double x;
    private double y;
    private double area;
    /*3 total LED Status 
        0: pipeline control
        1: off
        2: blink
        3: on
    */
    private double ledStatus; 
    final private String LL_NAME = "";//"limelight" for if left blank

    public Limelight_Helpers_util(){
        //
        llresults = LimelightHelpers.getLatestResults(LL_NAME);
    }

    public void peridic() {
        x = LimelightHelpers.getTX(LL_NAME);
        y = LimelightHelpers.getTY(LL_NAME);
        area = LimelightHelpers.getTA(LL_NAME);
        ledStatus = LimelightHelpers.getLEDMode(LL_NAME);
    }


/*Get mehods below*/
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getArea(){
        return area;
    }
    public double getLEDStatus(){
        return ledStatus;
    }


/*Set methods below*/
    /**
     * @param id :
     *       0: pipeline control
     *       1: off
     *       2: blink
     *       3: on 
     * Change the state of limeLight
     */
    public void setLEDMode(double id){
        if(id == 0.0){
            LimelightHelpers.setLEDMode_PipelineControl(LL_NAME);
        }
        else if(id == 1.0){
            LimelightHelpers.setLEDMode_ForceOff(LL_NAME);
        }
        else if(id == 2.0){
            LimelightHelpers.setLEDMode_ForceBlink(LL_NAME);
        }
        else if(id == 3.0){
            LimelightHelpers.setLEDMode_ForceOn(LL_NAME);
        }
        else{
        }
    }
}
