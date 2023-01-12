/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Add your docs here.
 */
public class SwitchboardController extends GenericHID {
    
    public enum SBButton {
      // Buttons
      // functional names
      DelayA(1),DelayB(2),DelayC(3), 
      //Row1
      Sw11(1), Sw12(2), Sw13(3), Sw14(4), Sw15(5), Sw16(6),
      //Row2
      Sw21(7), Sw22(8), Sw23(9), Sw24(10), Sw25(11), Sw26(12);
      //Row3
      //cursed - Sw31(13), Sw32(14), Sw33(15), Sw34(16);
      
      @SuppressWarnings({ "MemberName", "PMD.SingularField" })
      public final int value;
      private SBButton(final int val) {
        value = val;
      }
    }

    public SwitchboardController(final int port){
     super(port);
     //hack
     HAL.report(tResourceType.kResourceType_XboxController, port + 1);
   }

   /**
   * Read the value of Sw11 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw11() {
    return getRawButton(SBButton.Sw11.value);
  }

  /**
   * Read the value of Sw12 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw12() {
    return getRawButton(SBButton.Sw12.value);
  }

  /**
   * Read the value of Sw13 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw13() {
    return getRawButton(SBButton.Sw13.value);
  }

  /**
   * Read the value of Sw14 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw14() {
    return getRawButton(SBButton.Sw14.value);
  }

  /**
   * Read the value of Sw15 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw15() {
    return getRawButton(SBButton.Sw15.value);
  }

  /**
   * Read the value of Sw16 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw16() {
    return getRawButton(SBButton.Sw16.value);
  }

  /**
   * Read the value of Sw21 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw21() {
    return getRawButton(SBButton.Sw21.value);
  }

  /**
   * Read the value of Sw22 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw22() {
    return getRawButton(SBButton.Sw22.value);
  }

  /**
   * Read the value of Sw23 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw23() {
    return getRawButton(SBButton.Sw23.value);
  }

  /**
   * Read the value of Sw24 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw24() {
    return getRawButton(SBButton.Sw24.value);
  }

  /**
   * Read the value of Sw25 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw25() {
    return getRawButton(SBButton.Sw25.value);
  }

  /**
   * Read the value of Sw26 on the switchboard.
   *
   * @return The state of the button.
   */
  public boolean getSw26() {
    return getRawButton(SBButton.Sw26.value);
  }

  /**
   * Constructs an event instance around Sw11's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw11 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw11(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw11);
  }

  /**
   * Constructs an event instance around Sw12's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw12 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw12(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw12);
  }

  /**
   * Constructs an event instance around Sw13's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw13 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw13(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw13);
  }

  /**
   * Constructs an event instance around Sw14's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw14 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw14(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw14);
  }

  /**
   * Constructs an event instance around Sw15's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw15 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw15(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw15);
  }

  /**
   * Constructs an event instance around Sw16's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw16 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw16(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw16);
  }

  /**
   * Constructs an event instance around Sw21's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw21 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw21(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw21);
  }

  /**
   * Constructs an event instance around Sw22's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw22 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw22(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw22);
  }

  /**
   * Constructs an event instance around Sw23's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw23 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw23(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw23);
  }

  /**
   * Constructs an event instance around Sw24's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw24 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw24(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw24);
  }

  /**
   * Constructs an event instance around Sw25's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw25 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw25(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw25);
  }

  /**
   * Constructs an event instance around Sw26's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the Sw26 button's digital signal attached to the given
   *     loop.
   */
  public BooleanEvent sw26(EventLoop loop) {
    return new BooleanEvent(loop, this::getSw26);
  }

  @Override
  public double getRawAxis(int axis) {
    return 0;
  }

}
