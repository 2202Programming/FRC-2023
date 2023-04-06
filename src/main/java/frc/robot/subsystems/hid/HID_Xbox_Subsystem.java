/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

import java.util.HashMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.subsystems.hid.SwitchboardController.SBButton;

/**
 * HID_Subsystem - Human Input Device
 * 
 * Use this class bind devices to meaningful functions.
 * 
 * Add any needed member functions to the DriverControls interface and then add
 * to any implementations. This way they can work with different controller
 * setup. It will also make it easy to switch to a different Joystick.
 * 
 * Shouldn't need to make this subsystem a requirement for any command, just
 * reference it. This class is intended to run in the periodic(). It should be
 * run first by being first on the list.
 * 
 * XBox stick signs:
 *   Y stick forward is -1.0, backward is 1.0.
 *   X stick left is -1.0, right is 1.0.
 *  
 * Conventions used robot body axis:
 *    Arcade
 *      Y stick forward creates positive velocity, robot moves forward.
 *      X stick left create positive angular velocity, robots rotates counter-clockwise.
 * 
 *    Tank
 *      Y stick forward will be positive creates positive velocity for that side.
 * 
 */
public class HID_Xbox_Subsystem extends SubsystemBase {
  /**
   * Creates a new HID_Subsystem.
   */
  private final CommandXboxController driver;
  private final CommandXboxController operator;
  private final CommandSwitchboardController switchBoard;

  // Buttons onStartup - in case you want to do something based on controls
  // being held at power up or on switchboard.
  int initDriverButtons;
  int initAssistentButtons;
  int initSwitchBoardButtons;

  //boolean limitRotation = true;
  //Scale back the sticks for precision control
  double scale_xy = 1.0;
  double scale_rot = 1.0;

  //XYRot / Swerve (field or robot relative)
  ExpoShaper velXShaper;    // left/right  
  ExpoShaper velYShaper;    // forward/backward 
  ExpoShaper swRotShaper;   // rotation for XYRot

  //values updated each frame
  double vel, z_rot;           //arcade
  double velLeft, velRight;    //tank
  double velX,velY, xyRot;     //XTRot
  final double deadzone;;

  // invertGain is used to change the controls for driving backwards easily.
  // A negative value indicates you're driving backwards with forwards controls.
  double invertGain = 1.0;

  public HashMap<Id, CommandGenericHID> deviceMap = new HashMap<Id, CommandGenericHID>();

  public HID_Xbox_Subsystem(final double velExpo, final double rotExpo, final double deadzone) {

    // register the devices
    driver = (CommandXboxController) registerController(Id.Driver, new CommandXboxController(Id.Driver.value));
    operator = (CommandXboxController) registerController(Id.Operator, new CommandXboxController(Id.Operator.value));
    switchBoard = (CommandSwitchboardController) registerController(Id.SwitchBoard, new CommandSwitchboardController(Id.SwitchBoard.value));
   
    this.deadzone = deadzone;
    /**
     * All Joysticks are read and shaped without sign conventions.
     * Sign convention added on periodic based on the type of driver control
     * being used.
     */

    // XYRot or Swerve Drive
    // Rotation on Left-X axis,  X-Y throttle on Right
    velXShaper = new ExpoShaper(velExpo,  () -> driver.getRightY()); // X robot is Y axis on Joystick
    velYShaper = new ExpoShaper(velExpo,  () -> driver.getRightX()); // Y robot is X axis on Joystick
    swRotShaper = new ExpoShaper(rotExpo, () -> driver.getLeftX());

    // deadzone for swerve
    velXShaper.setDeadzone(deadzone);
    velYShaper.setDeadzone(deadzone);
    swRotShaper.setDeadzone(deadzone);

    // read some values to remove unused warning
    // CHANGED for 2022
    operator.getRightX();
    switchBoard.getRawAxis(0);

    // read initial buttons for each device - maybe used for configurions
    initDriverButtons = getButtonsRaw(Id.Driver);
    initAssistentButtons = getButtonsRaw(Id.Operator);
    initSwitchBoardButtons = getButtonsRaw(Id.SwitchBoard);
  }

  /**
   * SideBoard low level access or just fixed configuration for a controls set
   * This is a bit field that must be decoded into something meaningful. Buttons
   * start counting at 1, bits at zero button 1 = bit 0 1/0 button 2 = bit 1 2/0
   * ... button n = bit (n-1) 2^(n-1)/0
   * 
   * Use and/or logic to decode as needed.
   */
  public int getButtonsRaw(Id id) {
    return DriverStation.getStickButtons(id.value);
  }

  // accesors for our specific controllers to bind triggers 
  public CommandXboxController Driver() {return driver; }
  public CommandXboxController Operator() {return operator;}
  public CommandSwitchboardController SwitchBoard() {return switchBoard; }

  /**
   * constructor of the implementing class.
   * 
   * @param id  Id.Driver, Id.Assistent, Id.Sideboard
   * @param hid Input device, xbox or other stick
   * @return
   */
  public CommandGenericHID registerController(Id id, CommandGenericHID hid) {
    deviceMap.put(id, hid);
    return hid;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler frame and read all stick inputs
    // for all possible modes.  This is a few extra calcs and could be made modal to
    // only read/shape the stick mode.

    //XYRot - field axis, pos X away from driver station, pos y to left side of field
    //Added scale-factors for low-speed creeper mode
    velX = -velXShaper.get() * scale_xy;    //invert, so right stick moves robot, right, lowering Y 
    velY = -velYShaper.get() * scale_xy;    //invert, so forward stick is positive, increase X
    xyRot = -swRotShaper.get() * scale_rot; //invert, so positive is CCW 
  }
  
  //public void setLimitRotation(boolean enableLimit) {
  //  this.limitRotation = enableLimit;
  //}

  public double getVelocityX() {
    return velX;
  }

  public double getVelocityY() {
    return velY;
  }

  public double getXYRotation() {
    return xyRot;
  }

  public double getVelocity() {
    return vel;
  }

  public double getRotation() {
    return z_rot;
  }

  public boolean isNormalized() {
    return true;
  }
  
  /*
   * Scales the stick, must be between 0.1 and 1.0
   */
  public void setStickScale(double scale_xy, double scale_rot) {
   this.scale_xy = MathUtil.clamp(scale_xy, 0.1, 1.0);
   this.scale_rot = MathUtil.clamp(scale_rot, 0.1, 1.0);
  }

  /**
   * Returns the stick scaling to 1.0
   */
  public void resetStickScale() {
    this.scale_rot = 1.0;
    this.scale_xy = 1.0;
  }

  /**
   * 
   * rightStickMotion<Driver || Operator>
   * 
   * Useful to abort commands with an unitl() option.
   * 
   *   cmd.until(dc::rightStickMotionDriver)
   * 
   * @return  true - stick moved past deadzone
   *          false - no stick
   */
  public boolean rightStickMotionDriver() {
    final CommandXboxController device = driver;
    double x = Math.abs(device.getRightX());
    double y = Math.abs(device.getRightY());
    return (x > (deadzone * 2.0)) || (y > (deadzone * 2.0)); // 2x multiplier so doesn't abort too easily
 }

 public boolean rightStickMotionOperator() {
  final CommandXboxController device = operator;
  double x = Math.abs(device.getRightX());
  double y = Math.abs(device.getRightY());
  return (x > (deadzone * 2.0)) || (y > (deadzone * 2.0)); // 2x multiplier so doesn't abort too easily
}



  public int getInitialButtons(final Id id) {
    switch (id) {
    case Driver:
      return initSwitchBoardButtons;
    case Operator:
      return initAssistentButtons;
    case SwitchBoard:
      return initSwitchBoardButtons;
    default:
      return 0;
    }
  }

public boolean readSideboard(SBButton buttonId) {
  return this.switchBoard.getHID().getRawButton(buttonId.value);
  // The below isn't working. The above works. We don't have time to debug the below. --nren 02-15-2023 9:50pm
  // int switches = getInitialButtons(Id.SwitchBoard);
  // int mask = 1 << (buttonId.value -1);
  // return (switches & mask) !=0 ? true : false ;
}


public void turnOnRumble(Id id, RumbleType type){
  switch (id) {
    case Driver:
      driver.getHID().setRumble(type, 1);
    case Operator:
      operator.getHID().setRumble(type, 1);
    default:
      break;
  }
}

public void turnOffRumble(Id id, RumbleType type){
  switch (id) {
    case Driver:
      driver.getHID().setRumble(type, 0);
    case Operator:
      operator.getHID().setRumble(type, 0);
    default:
      break;
  }
}

public boolean isConnected(Id id){
  switch (id){
    case Driver:
      return driver.getHID().isConnected();
    case Operator:
      return operator.getHID().isConnected();
    case SwitchBoard:
      return switchBoard.getHID().isConnected();
    default:
      return false;
  }
}

}
