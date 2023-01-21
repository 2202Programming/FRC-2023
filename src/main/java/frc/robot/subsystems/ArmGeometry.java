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
    //getters and setters
    public double getAngle() {
        return angle;
    } 
    public void setAngle(double angle) {
        this.angle = angle;
    }
    public double getGeoX() {
        return geoX;
    }
    public void setGeoX(double geoX) {
        this.geoX = geoX;
    }
    public double getGeoY() {
        return geoY;
    }
    public void setGeoY(double geoY) {
        this.geoY = geoY;
    }
    public double getGeoZ() {
        return geoZ;
    }
    public void setGeoZ(double geoZ) {
        this.geoZ = geoZ;
    }
    public double getWidth1() {
        return w1;
    }
    public void setWidth1(double w1) {
        this.w1 = w1;
    }
    public double getWidth2() {
        return w2;
    }
    public void setWidth2(double w2) {
        this.w2 = w2;
    }
    public double getWidth3() {
        return w3;
    }
    public void setWidth3(double w3) {
        this.w3 = w3;
    }
    public double getStringLength() {
        return stringLength;
    }
    public void setStringLength(double stringLength) {
        this.stringLength = stringLength;
    }
    //constructor
    public ArmGeometry (double xW1, double xW2, double xW3, double xAngle, double xStringLength, double xGeoX, double xGeoY,double xGeoZ) {
        w1 = xW1;
        w2 = xW2;
        w3 = xW3;
        geoX = xGeoX;
        geoY = xGeoY;
        geoZ = xGeoZ;
        angle = xAngle;
        stringLength = xStringLength;
    }
    double width = w1 + w2 + w3;
    //method to find the 3 lengths
    public double[] lengthDimensions() {
        double lengthX = width*Math.sin(angle) + stringLength*Math.sin(angle) + geoX;
        double lengthY = geoY;
        double lengthZ = width*Math.cos(angle) + stringLength*Math.cos(angle) + geoZ;
        double[] lengths = {lengthX,lengthY,lengthZ};
        return lengths;
    } 
}

