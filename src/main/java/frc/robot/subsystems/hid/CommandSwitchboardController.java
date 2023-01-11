/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.hid;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Add your docs here.
 */
public class CommandSwitchboardController extends CommandGenericHID {
  private final SwitchboardController m_hid;
    
  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandSwitchboardController(int port) {
    super(port);
    m_hid = new SwitchboardController(port);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public SwitchboardController getHID() {
    return m_hid;
  }

  /**
   * Constructs an event instance around Sw11's digital signal.
   *
   * @return an event instance representing Sw11's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw11(EventLoop)
   */
  public Trigger sw11() {
    return sw11(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw11's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw11's digital signal attached to the given
   *     loop.
   */
  public Trigger sw11(EventLoop loop) {
    return m_hid.sw11(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw12's digital signal.
   *
   * @return an event instance representing Sw12's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw12(EventLoop)
   */
  public Trigger sw12() {
    return sw12(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw12's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw12's digital signal attached to the given
   *     loop.
   */
  public Trigger sw12(EventLoop loop) {
    return m_hid.sw12(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw13's digital signal.
   *
   * @return an event instance representing Sw13's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw13(EventLoop)
   */
  public Trigger sw13() {
    return sw13(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw13's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw13's digital signal attached to the given
   *     loop.
   */
  public Trigger sw13(EventLoop loop) {
    return m_hid.sw13(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw14's digital signal.
   *
   * @return an event instance representing Sw14's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw14(EventLoop)
   */
  public Trigger sw14() {
    return sw14(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw14's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw14's digital signal attached to the given
   *     loop.
   */
  public Trigger sw14(EventLoop loop) {
    return m_hid.sw14(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw15's digital signal.
   *
   * @return an event instance representing Sw15's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw15(EventLoop)
   */
  public Trigger sw15() {
    return sw15(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw15's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw15's digital signal attached to the given
   *     loop.
   */
  public Trigger sw15(EventLoop loop) {
    return m_hid.sw15(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw16's digital signal.
   *
   * @return an event instance representing Sw16's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw16(EventLoop)
   */
  public Trigger sw16() {
    return sw16(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw16's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw16's digital signal attached to the given
   *     loop.
   */
  public Trigger sw16(EventLoop loop) {
    return m_hid.sw16(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw21's digital signal.
   *
   * @return an event instance representing Sw21's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw21(EventLoop)
   */
  public Trigger sw21() {
    return sw21(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw21's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw21's digital signal attached to the given
   *     loop.
   */
  public Trigger sw21(EventLoop loop) {
    return m_hid.sw21(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw22's digital signal.
   *
   * @return an event instance representing Sw22's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw22(EventLoop)
   */
  public Trigger sw22() {
    return sw22(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw22's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw22's digital signal attached to the given
   *     loop.
   */
  public Trigger sw22(EventLoop loop) {
    return m_hid.sw22(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw23's digital signal.
   *
   * @return an event instance representing Sw23's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw23(EventLoop)
   */
  public Trigger sw23() {
    return sw23(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw23's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw23's digital signal attached to the given
   *     loop.
   */
  public Trigger sw23(EventLoop loop) {
    return m_hid.sw23(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw24's digital signal.
   *
   * @return an event instance representing Sw24's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw24(EventLoop)
   */
  public Trigger sw24() {
    return sw24(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw24's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw24's digital signal attached to the given
   *     loop.
   */
  public Trigger sw24(EventLoop loop) {
    return m_hid.sw24(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw25's digital signal.
   *
   * @return an event instance representing Sw25's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw25(EventLoop)
   */
  public Trigger sw25() {
    return sw25(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw25's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw25's digital signal attached to the given
   *     loop.
   */
  public Trigger sw25(EventLoop loop) {
    return m_hid.sw25(loop).castTo(Trigger::new);
  }

  /**
   * Constructs an event instance around Sw26's digital signal.
   *
   * @return an event instance representing Sw26's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #sw26(EventLoop)
   */
  public Trigger sw26() {
    return sw26(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around Sw26's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing Sw26's digital signal attached to the given
   *     loop.
   */
  public Trigger sw26(EventLoop loop) {
    return m_hid.sw26(loop).castTo(Trigger::new);
  }


}
