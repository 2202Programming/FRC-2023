// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PCM1;

/*
 * Notes from Mr.L  1/27/2023
 * 
 * Some of your variables would make for a good API
 * 
 *      is_open the state can be read off the Pnumatics 
 *      pieceheld -->  enum  HasPiece()  returns NONE, CUBE, CONE
 *     
 *      Is angle is wrist angle??, that may be a PWM Server (see vacuum bot)
 *      it won't need a PID, but will need a pwm port. 
 *      
 */
// 2 wrist servos, 1 left/right each have a 0 position, have one mirror the other, (switch signs & mapping)
// call 180 angle 0 and work from one way to the other - 0 wrist won't be 0 servo 

/*
 * notes from Mr.L 2/7/23
 * 
 * Format imports, variable section and code in general 
 *    (shift-alt-O) organizes imports
 *    (shift-ctrl-P format document)
 * 
 *  custom servo commented out for now, so can compile
 * 
 * Is the wrist here or in the Arm??? 
 * 
 */

//Eventually will need 2 solenoids 
public class Claw_Substyem extends SubsystemBase {

   public enum GamePieceHeld{  //TODO how do we know???
        Cube,Cone,Empty
      }
    

  // constants/statics
  static final double KServoSpd = 5.0;    // [deg/s] time to wait for a move of the wrist
  static final Value OPEN = Value.kForward;
  static final Value CLOSE = Value.kReverse;
  static final double WristMinDegrees = -90.0; // TODO: Find actual value
  static final double WristMaxDegrees = 90.0; // TODO: Find actual value
  static final double kServoMinPWM = 0.1; // TODO: Find actual value
  static final double kServoMaxPWM = 0.5; // TODO: Find actual value

  // Hardware
  private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PCM1.CLAW_FWD, PCM1.CLAW_REV);
  private Servo rightServo = new Servo(frc.robot.Constants.PWM.RIGHT_WRIST);
  private Servo leftServo = new Servo(frc.robot.Constants.PWM.LEFT_WRIST);
  // protected CustomServo wristServo; //tbd if needed

  // state vars
  private boolean is_open;
  private GamePieceHeld piece_held;
  private double wrist_cmd;  //[deg]

  /** Creates a new Claw. */
  public Claw_Substyem() {
    // Creating the custom servo if needed
    // TBD wristServo = new CustomServo(Claw.CLAW_WRIST_SERVO_PWM, WristMinDegrees,
    // WristMaxDegrees, kServoMinPWM, kServoMaxPWM);

    piece_held = GamePieceHeld.Empty;

  }

  // getting the angles current position
  public double getAngle() {

    // Just read the left and fixup the coordinates
    // if left/right are not in sync, not much we can do.
    // they will get there.
    double current_pos = leftServo.get();  // todo:fix any offset so math is correct
    //if customServo - wristServo.getPosition() * wristServo.getServoRange() + wristServo.getMinServoAngle();
    return current_pos;
  }

  // Not sure if the part below is correct?
  public void setAngle(double degrees) {

    // do some math on the requested angle to get left/right settings
    // assume left "unmirrored"
    double left_cmd =  degrees;             // todo: may need to add left_offset
    double right_cmd = 180.0 - left_cmd;    // mirror it, maybe rt_offset (example, needs geometry )

    // Command the wrist servos to 
    rightServo.set(right_cmd);
    leftServo.set(left_cmd);

    //save our commanded position
    wrist_cmd = degrees; 

    //TODO handle a timer to know if we are at position...

  }

  public boolean atAngle() {
    //TODO: 2/7/23 need to figure out tolerance for when at position

    return false;
  }

  /*
   * waitEstimate(degrees) -
   * creates an estimate in seconds that it will take the wrist to move from
   * where we are to where we want...  This might be needed if we can't actually
   * measure the servo's position.   (PWM is really a one-way street) 
   */
  public double waitEstimate(double degrees) {
    return Math.abs(degrees - wrist_cmd)*KServoSpd;
  }



  @Override
  public void periodic() {
    clawStatus();  
    // check any lightgates

    // TODO: 2/7/23 how do we know what game piece?  Work with HW guys see what they are thinking
  }

  // setting solenoid  NOTE:2/7 don't need OpenClaw... there will be a Claw Object so it will read  claw.open() or claw.close() 
  public void open() {
    solenoid.set(OPEN);
    is_open = true;
  }

  public void close() {
    solenoid.set(CLOSE);
    is_open = false;
  }

  public boolean isOpen() {
    return is_open;
  }

  // getting status of solenoid (open/close)
  // and keep the is_open flag up to date
  // commands can use isOpen()
  private Value clawStatus() {
    var value = solenoid.get();
    is_open = (value == OPEN);
    return value;
  }

  /**
   ******** NETWORK TABLES ***********
   */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Claw");

  NetworkTableEntry nt_servoPos;
  NetworkTableEntry nt_angle;
  NetworkTableEntry nt_isOpen;
  NetworkTableEntry nt_gamePieceHeld;

  public void ntcreate() {
    nt_servoPos = table.getEntry("wrist_cmd");
    nt_angle = table.getEntry("Current Angle");
    nt_isOpen = table.getEntry("Is Claw Open");
    nt_gamePieceHeld = table.getEntry("Game Piece Held");
  }

  public void ntupdates() {
    // info
    nt_servoPos.setDouble(wrist_cmd);
    nt_angle.setDouble(getAngle());
    nt_isOpen.setBoolean(is_open);
    nt_gamePieceHeld.setString(piece_held.toString());

  }

}