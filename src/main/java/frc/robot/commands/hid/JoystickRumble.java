// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hid;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverControls.Id;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

public class JoystickRumble extends CommandBase {
  /** Creates a new JoystickRumble. */

  HID_Xbox_Subsystem dc;
  Timer timer;
  Id id;
  double duration;
  int segments;
  int current_segment;
  double segment_duration;
  double silent_duration = 0.2;
  boolean finished = false;
  boolean silent_segment = false;

  //rumble the joystick given by id, duration in sec
  public JoystickRumble(HID_Xbox_Subsystem driverControls, Id id, double duration) {
    this(driverControls, id, duration, 1);
  }

  //divide duration up into equal SEGMENTS with a fixed length silent gap between.  Segments are the # of rumble segments.
  public JoystickRumble(HID_Xbox_Subsystem driverControls, Id id, double duration, int segments) {
    dc = driverControls;
    addRequirements(driverControls);
    this.id = id;
    this.duration = duration;
    this.segments = segments;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    current_segment = 1;
    segment_duration = duration/segments;
    dc.turnOnRumble(id);
    silent_segment = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (silent_segment) {
      if(timer.hasElapsed(silent_duration)){ //silent segment over, start rumble again
        silent_segment = false;
        timer.reset();
        timer.start();
        dc.turnOnRumble(id);
      }
    }
    else if (timer.hasElapsed(segment_duration)){ 
      current_segment++;
      if (current_segment > segments){ //all done
        finished = true;
      }
      else { //more segments to go, start a silent segment (these don't count against total segment count)
        silent_segment = true;
        dc.turnOffRumble(id);
        timer.reset();
        timer.start();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dc.turnOffRumble(id);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
