// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.Constants.DriverControls.Id;

public class JoystickRumble extends Command {
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
  public JoystickRumble(Id id, double duration) {
    this(id, duration, 1);
  }

  //divide duration up into equal SEGMENTS with a fixed length silent gap between.  Segments are the # of rumble segments.
  public JoystickRumble(Id id, double duration, int segments) {
    dc = RobotContainer.RC().dc;
    this.id = id;
    this.duration = duration;
    this.segments = segments;
    if (segments<1){
      segments = 1; // need at least one segment, prevent divide by zero
    }
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    current_segment = 1;
    segment_duration = (duration - ((segments-1) * silent_duration))/segments;
    RobotContainer.RC().dc.turnOnRumble(id, RumbleType.kBothRumble);
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
        RobotContainer.RC().dc.turnOnRumble(id, RumbleType.kBothRumble);
      }
    }
    else if (timer.hasElapsed(segment_duration)){ 
      current_segment++;
      if (current_segment > segments){ //all done
        finished = true;
      }
      else { //more segments to go, start a silent segment (these don't count against total segment count)
        silent_segment = true;
        RobotContainer.RC().dc.turnOffRumble(id, RumbleType.kBothRumble);
        timer.reset();
        timer.start();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.RC().dc.turnOffRumble(id, RumbleType.kBothRumble);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
