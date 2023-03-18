// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlowDown extends InstantCommand {
  public SlowDown() {
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double time = Timer.getFPGATimestamp();
    
    double currentTime = Timer.getFPGATimestamp();

    System.out.println(time);
    while(currentTime - time < 1.5){
      currentTime = Timer.getFPGATimestamp();
      System.out.println(currentTime);
      new PathConstraints(0.25, 0.25);
    }
  }
  public void end()
  {

  }
}
