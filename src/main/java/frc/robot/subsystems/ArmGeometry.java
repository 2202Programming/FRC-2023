package frc.robot.subsystems;
import java.lang.Math;
public class ArmGeometry {
    //Passive -> Extend, Active <- Retreat
    //Length of string -> motion -> position servo + 2 spark max + spring
    private double w1;
    private double w2;
    private double w3; 
    //geometry-derived length in all three directions
    private double geoX; //wheel and chassis height in x-direction
    private double geoY; //wheel and chassis height in y-direction
    private double geoZ; //wheel and chassis height in z-direction
    //length of string
    private double stringLength;
    //angle between the lengths
    double angle;
    double width = w1 + w2 + w3;
    // array that holds the three lengths
    public double[] lengthDimensions() {
        double lengthX = width*Math.sin(angle) + stringLength*Math.sin(angle) + geoX;
        double lengthZ = width*Math.cos(angle) + stringLength*Math.cos(angle) + geoZ;
        double[] lengths = {lengthX,lengthY,lengthZ};
        return lengths;
    } 
    
    
}

