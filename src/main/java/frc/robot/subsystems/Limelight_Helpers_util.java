package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;


public class Limelight_Helpers_util extends SubsystemBase{
    LimelightResults llresults;
    DriverStation.Alliance color;
    private double[] megaPose;
    private double[] teamPose;
    private boolean isBlue;
    final private String LL_NAME = "";//"limelight" for if left blank

    public Limelight_Helpers_util(){       
        
        //getting team Color
        color = DriverStation.getAlliance();
        if(color == DriverStation.Alliance.Blue){
            isBlue = true;
        }

    }

    public void peridic() {
        // in meters and degrees
        megaPose = LimelightHelpers.getBotPose(LL_NAME);

        //TeamPose
        if(isBlue){
            teamPose = LimelightHelpers.getBotpose_wpiBlue(LL_NAME);
        }
        else{
            teamPose = LimelightHelpers.getBotPose_wpiRed(LL_NAME);
        }

    }


/*Get mehods below*/

    public double getX(){
        return megaPose[0];
    }
    public double getY(){
        return  megaPose[1];
    }
    public double getZ(){
        return  megaPose[2];
    }
    public double getRoll(){
        return megaPose[3];
    }
    public double getPitch(){
        return megaPose[4];
    }
    public double getYaw(){
        return megaPose[5];
    }



    public double get_teamX(){
        return teamPose[0];
    }
    public double get_teamY(){
        return  teamPose[1];
    }
    public double get_teamZ(){
        return  teamPose[2];
    }
    public double get_teamRoll(){
        return teamPose[3];
    }
    public double get_teamPitch(){
        return teamPose[4];
    }
    public double get_teamYaw(){
        return teamPose[5];
    }

}
