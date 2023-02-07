package frc.robot.util;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.hal.HAL;

public class CustomServo extends PWM {
    private final double MaxServoAngle;
    private final double MinServoAngle;
    private final double ServoRange;

    private final double DefaultMaxServoPWM;
    private final double DefaultMinServoPWM;
    private double position; // save the setting and use it to return a value if asked

    /**
     * Constructor.<br>
     *
     * <p>
     * By default {@value #kDefaultMaxServoPWM} ms is used as the maxPWM value<br>
     * By default {@value #kDefaultMinServoPWM} ms is used as the minPWM value<br>
     *
     * @param channel    The PWM channel to which the servo is attached. 0-9 are
     *                   on-board, 10-19 are on the MXP port
     * 
     * @param minDegrees smallest angle of rotation in degrees
     * 
     * @param maxDegrees largest angle of rotation in degrees
     * 
     * @param minPWMuS   servo timing min value .6 typical
     * @param maxPWMuS   Servo timing max, 2.5 uS typical
     * 
     */
    public CustomServo(final int channel, final double minDegrees, final double maxDegrees, final double minPWMuS,
        final double maxPWMuS) {
      super(channel);
      MaxServoAngle = maxDegrees;
      MinServoAngle = minDegrees;
      DefaultMaxServoPWM = maxPWMuS;
      DefaultMinServoPWM = minPWMuS;
      // compute range once
      ServoRange = MaxServoAngle - MinServoAngle;
      setBounds(DefaultMaxServoPWM, 0.0, 0.0, 0.0, DefaultMinServoPWM);
      setPeriodMultiplier(PeriodMultiplier.k4X);

      HAL.report(tResourceType.kResourceType_Servo, getChannel());
    //  setName("CustomServo", getChannel());
    }

    public double getMinServoAngle(){
        return MinServoAngle;
    }
    public double getMaxServoAngle(){
        return MaxServoAngle;
    }
    public double getServoRange(){
        return ServoRange;
    }

    

    /**
     * Set the servo angle.
     *
     * <p>
     * Assume that the servo angle is linear with respect to the PWM value (big
     * assumption, need to test).
     *
     * <p>
     * Servo angles that are out of the supported range of the servo simply
     * "saturate" in that direction In other words, if the servo has a range of (X
     * degrees to Y degrees) than angles of less than X result in an angle of X
     * being set and angles of more than Y degrees result in an angle of Y being
     * set.
     *
     * @param degrees The angle in degrees to set the servo.
     */
    public void setAngle(double degrees) {
      if (degrees < MinServoAngle) {
        degrees = MinServoAngle;
      } else if (degrees > MaxServoAngle) {
        degrees = MaxServoAngle;
      }

      position = (degrees - MinServoAngle) / ServoRange;
      setPosition(position);
    }

    /**
     * Get the servo angle.
     *
     * <p>
     * Assume that the servo angle is linear with respect to the PWM value (big
     * assumption, need to test).
     * 
     * Derek L - getPosition() returns zero. Fake a value with saved position.
     * 2/24/2019
     *
     * @return The angle in degrees to which the servo is set.
     */
    public double getAngle() {
      double pos = getPosition() * ServoRange + MinServoAngle;
      return pos;
    }

    // DPL - use only the get/set angle for CustomServo
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("CustomServo");
      builder.addDoubleProperty("Value", this::getAngle, this::setAngle);
    }

    public void setName(String subsystem, String string) {
    }
  }