package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PowerOnPos;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.NeoServo;
import frc.robot.util.PIDFController;
import frc.robot.util.VelocityControlled;

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

public class ArmSS extends SubsystemBase implements VelocityControlled {
    final int STALL_CURRENT = 50;
    final int FREE_CURRENT = 20;
    final double ARM_MIN_EXT = 0.0;
    final double ARM_MAX_EXT = 37.0;

    final double gearRadius = 2.63398 * 2 * Math.PI; // 2.633[cm] is drive gear radius
    final double gearRatio = (1.0 / 75.0);
    final double conversionFactor = gearRadius * gearRatio;

    // state vars
    PIDController positionPID_rt = new PIDController(5.0, 0.150, 0.250); // outer position loop
    PIDController positionPID_lt = new PIDController(5.0, 0.150, 0.250); // outer position loop
    PIDFController hwVelPID_rt = new PIDFController(0.002141, 0.000055, 0.15, 0.0503);
    PIDFController hwVelPID_lt = new PIDFController(0.002141, 0.000055, 0.15, 0.0503);
    final int hwVelSlot = 0;

    // positive extension moves arm out
    final boolean invert_left = false;
    final boolean invert_right = true;

    // instance variables
    // State vars
    final NeoServo leftArm;
    final NeoServo rightArm;
    boolean follow_mode;

    // PID and speed constants
    double maxVel = 5.0; // [cm/s] 
    double maxAccel = 20.0; // [cm/s^2] Not enforce until smartmode used
    double posTol = 0.30; // [cm]
    double velTol = 0.25; // [cm/s]

    // sync instance vars
    boolean sync = false; // don't think we need this, +- 3mm without.
    double syncCompensation = 0.0; // amount of compensation [m/s]

    // computed values from position pid
    double pos_error;

    // controllers
    PIDController syncPID = new PIDController(0.0, 0.0, 0.0); // arm synchronization pid. syncs left --> right

    public ArmSS() {
        rightArm = new NeoServo(CAN.ARM_RIGHT_Motor, positionPID_rt, hwVelPID_rt, invert_right);
        leftArm = new NeoServo(CAN.ARM_LEFT_Motor, positionPID_lt, hwVelPID_lt, invert_left);
        rightArm.setName(getName() + "/ArmServo_right");
        leftArm.setName(getName() + "/ArmServo_left");
        configure(rightArm);
        configure(leftArm);

        setClamp(ARM_MIN_EXT, ARM_MAX_EXT);
        // zero our encoders at power up
        setPosition(PowerOnPos.arm);
    }

    // finish the arm servo configurations
    void configure(NeoServo arm) {
        // rest of Neo and servo PID stuff
        arm.setConversionFactor(conversionFactor)
                .setSmartCurrentLimit(STALL_CURRENT, FREE_CURRENT)
                .setVelocityHW_PID(maxVel, maxAccel)
                .setTolerance(posTol, velTol)
                .setMaxVelocity(maxVel)
                .burnFlash();
    }

    @Override
    public void periodic() {
        // Synchronization, drive left to follow right
        pos_error = rightArm.getPosition() - leftArm.getPosition();
        syncCompensation = sync ? syncPID.calculate(leftArm.getPosition(), rightArm.getPosition()) : 0.0;
        leftArm.periodic(syncCompensation);
        rightArm.periodic(0.0);
    }

    // At Position flags for use in the commands
    public boolean atSetpoint() {
        return (rightArm.atSetpoint() && leftArm.atSetpoint());
    }

    public void setMaxVel(double vel_limit) {
        leftArm.setMaxVel(vel_limit);
        rightArm.setMaxVel(vel_limit);
    }

    public double getMaxVel() {
        // arms should have same vel_limit
        return rightArm.getMaxVel();
    }

    public void hold() {
        leftArm.hold();
        rightArm.hold();
    }

    public void setSetpoint(double extension) {
        leftArm.setSetpoint(extension);
        rightArm.setSetpoint(extension);
        sync = true;
    }

    public void setClamp(double min_pos, double max_pos){
      leftArm.setClamp(min_pos, max_pos);
      rightArm.setClamp(min_pos, max_pos);
    }

    public double getSetpoint() {
        // arms track, so just use right one
        return rightArm.getSetpoint();
    }

    public double getPosition() {
        return (leftArm.getPosition() + rightArm.getPosition()) / 2.0;
    }

    // initializes position, doesn't move anything. Defines zero or wherever you.
    public void setPosition(double extension) {
        leftArm.setPosition(extension);
        rightArm.setPosition(extension);
    }

    // Testing mode, use with care since there are no limit switches
    public void setVelocityCmd(double vel_cm) {
        double v = MathUtil.clamp(vel_cm, -maxVel, maxVel);
        leftArm.setVelocityCmd(v);
        rightArm.setVelocityCmd(v);
    }

    public double getVelocityCmd() {
        return (leftArm.getVelocityCmd() + rightArm.getVelocityCmd()) / 2.0;
    }

    public double getVelocity() {
        return (leftArm.getVelocity() + rightArm.getVelocity()) / 2.0;
    }

    public double getPositionError() {
        return pos_error;
    }

    // rarely used, testing only since arms are bolted together
    public void setVelocityLeft(double speed) {
        leftArm.setVelocityCmd(speed);
        sync = false;
    }

    public void setVelocityRight(double speed) {
        rightArm.setVelocityCmd(speed);
        sync = false;
    }

    public double getTrim() {
      // avoid the complexity of independent trim if we can.
      return leftArm.getTrim();
    }
  
    public void setTrim(double trim){
      leftArm.setTrim(trim);
      rightArm.setTrim(trim);
    }
  

    public Command getWatcher() {
        return new ArmSSWatcher();
    }

    public boolean isArmCmdExtended() {
      return (leftArm.getPosition() >= 37.0) && (rightArm.getPosition() >= 37.0);
    }

    class ArmSSWatcher extends WatcherCmd {

        /******************
         * Network Table Stuff.
         * 
         * Should most of this be in the arm class? Probably.
         * Is it easier to look at and fix if it's in the ArmSS class? Probably.
         *************/
        NetworkTable table = NetworkTableInstance.getDefault().getTable(ArmSS.this.getName());

        // maxs, mins, tols
        NetworkTableEntry nt_maxVel;
        NetworkTableEntry nt_posTol;
        NetworkTableEntry nt_velTol;
        NetworkTableEntry nt_error;
        NetworkTableEntry nt_syncCompensation;

        ArmSSWatcher() {
            super();
            rightArm.getWatcher();
            leftArm.getWatcher();
        }

        @Override
        public String getTableName() {
            return ArmSS.this.getName();
        }

        @Override
        public void ntcreate() {
            // PIDs
            NetworkTable table = getTable();
            var tname = getTableName();
            SmartDashboard.putData(tname + "/positionPID_lt", positionPID_lt);
            SmartDashboard.putData(tname + "/positionPID_rt", positionPID_rt);
            SmartDashboard.putData(tname + "/syncPID", syncPID);

            nt_syncCompensation = table.getEntry("/SyncComp");

            // maxs, mins, tols
            nt_maxVel = table.getEntry("MaxVelocity");
            nt_posTol = table.getEntry("Position Tolerance");
            nt_velTol = table.getEntry("Velocity Tolerance");
            nt_error = table.getEntry("Position Error");

            // init with correct values
            nt_maxVel.setDouble(maxVel);
            nt_posTol.setDouble(posTol);
            nt_velTol.setDouble(velTol);
            nt_error.setDouble(pos_error);
        }

        @Override
        public void ntupdate() {
            // info (get)
            nt_error.setDouble(pos_error);
            nt_syncCompensation.setDouble(syncCompensation);

            // look for updates from NT for editable values
            maxVel = nt_maxVel.getDouble(maxVel);
            double _posTol = nt_posTol.getDouble(posTol);
            double _velTol = nt_velTol.getDouble(velTol);

            // update with new values
            setMaxVel(maxVel);
            positionPID_lt.setTolerance(_posTol, _velTol);
            positionPID_rt.setTolerance(_posTol, _velTol);

            nt_posTol.setDouble(posTol);
            nt_velTol.setDouble(velTol);
        }
    }

    /// Moved from constants, not used as of 3/17/23
    // class is for us to figure out our position on field when using inclinator
    // uses geometry to find out our x, y, and z direction in a function in terms of
    // the angle between the bars
  public class ArmGeometry {
    // Passive -> Extend, Active <- Retreat
    // Length of string -> motion -> position servo + 2 spark max + spring
    private double w1;
    private double w2;
    private double w3;
    // geometry-derived length in all three directions based on comp bot
    public static final double geoX = 175.60; // wheel and chassis height in x-direction
    public static final double geoY = 115.40; // wheel and chassis height in y-direction
    public static final double geoZ = 162.15; // wheel and chassis height in z-direction
  
    // length of string
    private double stringLength;
    // angle between the lengths
    double angle;

    // getters and setters
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

    // constructor
    ArmGeometry(double xW1, double xW2, double xW3, double xAngle, double xStringLength) {
        w1 = xW1;
        w2 = xW2;
        w3 = xW3;
        angle = xAngle;
        stringLength = xStringLength;
      }
  
      double width = w1 + w2 + w3;
      // array to find the 3 lengths
      double lengthX = width * Math.sin(angle) + stringLength * Math.sin(angle) + geoX;
      double lengthY = geoY;
      double armLength = width * Math.cos(angle) + stringLength * Math.cos(angle) + geoZ;
      double[] lengths = { lengthX, lengthY, armLength };
  
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
}
