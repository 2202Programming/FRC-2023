/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.subsystems.hid.CommandSideboardController.SBButton;


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

  static class ActionOnEdge {
    boolean prev;
    boolean toggle;
    final Trigger button;
    final String name;

    public ActionOnEdge(String name, Trigger b) {
      this.name = name;
      toggle = false;
      button = b;
      read();
    }
    
    private boolean read() {
      prev = button.getAsBoolean();
      return prev;
    }

    boolean risingEdge() {
      if (prev == true) {
          read();
          return false;
        }
      else if (read()) {
        toggle = !toggle;
        return true;
      }
      return false;
    }
    boolean get() {return toggle;}
  }



  /**
   * Creates a new HID_Subsystem.
   */
  private final CommandXboxController driver;
  private final CommandXboxController assistant;
  private final CommandSideboardController switchBoard;
  // private final XboxController phantom = new XboxController(3);
  private final JoystickButton fieldRelativeButton;
  ActionOnEdge fieldRel;

  // Buttons onStartup - in case you want to do something based on controls
  // being held at power up or on switchboard.
  int initDriverButtons;
  int initAssistentButtons;
  int initSwitchBoardButtons;

  boolean limitRotation = true;

  //XYRot / Swerve (field or robot relative)
  ExpoShaper velXShaper;    // left/right  
  ExpoShaper velYShaper;    // forward/backward 
  ExpoShaper swRotShaper;   // rotation for XYRot

  //values updated each frame
  double vel, z_rot;           //arcade
  double velLeft, velRight;    //tank
  double velX,velY, xyRot;     //XTRot

  // invertGain is used to change the controls for driving backwards easily.
  // A negative value indicates you're driving backwards with forwards controls.
  double invertGain = 1.0;

  public HashMap<Id, CommandGenericHID> deviceMap = new HashMap<Id, CommandGenericHID>();

  public HID_Xbox_Subsystem(final double velExpo, final double rotExpo, final double deadzone) {

    // register the devices
    driver = (CommandXboxController) registerController(Id.Driver, new CommandXboxController(Id.Driver.value));
    assistant = (CommandXboxController) registerController(Id.Assistant, new CommandXboxController(Id.Assistant.value));
    switchBoard = (CommandSideboardController) registerController(Id.SwitchBoard, new CommandSideboardController(Id.SwitchBoard.value));

   

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
    fieldRelativeButton = (deviceMap.get(Id.Driver) != null) ? new JoystickButton(deviceMap.get(Id.Driver).getHID(), XboxController.Button.kLeftBumper.value) : null;
    fieldRel = new ActionOnEdge("field-relative", fieldRelativeButton);

    // deadzone for swerve
    velXShaper.setDeadzone(deadzone);
    velYShaper.setDeadzone(deadzone);
    swRotShaper.setDeadzone(deadzone);

    // read some values to remove unused warning
    // CHANGED for 2022
    assistant.getRightX();
    switchBoard.getRawAxis(0);

    // read initial buttons for each device - maybe used for configurions
    initDriverButtons = getButtonsRaw(Id.Driver);
    initAssistentButtons = getButtonsRaw(Id.Assistant);
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
    velX = -velXShaper.get();    //invert, so right stick moves robot, right, lowering Y 
    velY = -velYShaper.get();    //invert, so forward stick is positive, increase X
    xyRot = -swRotShaper.get();  //invert, so positive is CCW 

    fieldRel.risingEdge();
  }
  
  public void setLimitRotation(boolean enableLimit) {
    this.limitRotation = enableLimit;
  }

  private void limitTankRotation() {
    if (limitRotation == false) return;

    // Apply a rotation limit on tank based on command
    double Kv = 33.0;
    double avg = (velRight + velLeft) / 2.0;
    double absV = Math.abs(avg);

    double maxDelta = 1.0 / (Kv * absV * absV * absV + 1.0);
    double absDelta = Math.abs(velLeft - velRight);

    // Handle the different quadrents of the stick for tank drive
    if ((velLeft > 0) && (velRight > 0) || (velLeft < 0) && (velRight < 0)) {
      // Commanding forward or reverse, sticks in same direction

      if (absDelta > maxDelta) {
        // equalize the sticks so no rotation
        velLeft = avg;
        velRight = avg;
      }
    }
    else {
      //Sticks in opposite direction
    }
  }

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

  public int getInitialButtons(final Id id) {
    switch (id) {
    case Driver:
      return initSwitchBoardButtons;
    case Assistant:
      return initAssistentButtons;
    case SwitchBoard:
      return initSwitchBoardButtons;
    default:
      return 0;
    }
  }

  public boolean useFieldRelative() {
    return fieldRel.get();
  }

public boolean initialSideboard(SBButton buttonId) {
  int switches = getInitialButtons(Id.SwitchBoard);
  int mask = 1 << (buttonId.value -1);
  return (switches & mask) !=0 ? true : false ;
}


public void turnOnRumble(Id id){
  switch (id) {
    case Driver:
      driver.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1);
      driver.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 1);
    case Assistant:
      assistant.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 1);
      assistant.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 1);
    case SwitchBoard:
      break;
  }
}

public void turnOffRumble(Id id){
  switch (id) {
    case Driver:
      driver.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
      driver.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0);
    case Assistant:
      assistant.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
      assistant.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0);
    case SwitchBoard:
      break;
  }
}

public boolean isConnected(Id id){
  switch (id){
    case Driver:
      return driver.getHID().isConnected();
    case Assistant:
      return assistant.getHID().isConnected();
    case SwitchBoard:
      return switchBoard.getHID().isConnected();
    default:
      return false;
  }
}

}
