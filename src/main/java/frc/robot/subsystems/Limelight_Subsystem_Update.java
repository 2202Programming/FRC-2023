package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight_Subsystem_Update extends SubsystemBase{
    private double x;
    private double filteredX;
    private double y;
    private double area; // area is between 0 and 100. Calculated as a percentage of image
    private boolean target;
    private boolean ledStatus; // true = ON
    private double filteredArea;

    public Limelight_Subsystem_Update(){
    }

    public void peridic() {

    }
}
