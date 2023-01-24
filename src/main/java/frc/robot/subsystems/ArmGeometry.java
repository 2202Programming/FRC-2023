package frc.robot.subsystems;
import java.lang.Math;

// class is for us to figure out our position on field when using inclinator
// uses geometry to find out our x, y, and z direction in a function in terms of the angle between the bars
public class ArmGeometry {
    //Passive -> Extend, Active <- Retreat
    //Length of string -> motion -> position servo + 2 spark max + spring
    private double w1;
    private double w2;
    private double w3; 
    //geometry-derived length in all three directions based on comp bot
    public static final double geoX = 175.60; //wheel and chassis height in x-direction
    public static final double geoY = 115.40; //wheel and chassis height in y-direction
    public static final double geoZ = 162.15; //wheel and chassis height in z-direction

    //length of string
    private double stringLength;
    //angle between the lengths
    double angle;
    //getters and setters
    public double getAngle() {
        return angle;
    } 
    public double getGeoX() {
        return geoX;
    }
    public double getGeoY() {
        return geoY;
    }
    public double getGeoZ() {
        return geoZ;
    }

    public double getWidth1() {
        return w1;
    }
    public double getWidth2() {
        return w2;
    }
    public double getWidth3() {
        return w3;
    }
    public double getStringLength() {
        return stringLength;
    }
    //constructor
    public ArmGeometry (double xW1, double xW2, double xW3, double xAngle, double xStringLength, double xGeoX, double xGeoY,double xGeoZ) {
        w1 = xW1;
        w2 = xW2;
        w3 = xW3;
        angle = xAngle;
        stringLength = xStringLength;
    }
    double width = w1 + w2 + w3;
    //array to find the 3 lengths
    double lengthX = width*Math.sin(angle) + stringLength*Math.sin(angle) + geoX;
    double lengthY = geoY;
    double armLength = width*Math.cos(angle) + stringLength*Math.cos(angle) + geoZ;
    double[] lengths = {lengthX,lengthY,armLength};
    public double getLengthX() {
        return lengthX;
    }
    public double getLengthY() {
        return lengthY;
    }
    public double getLengthZ() {
        return armLength;
    }
}
