// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class VisionWatchdog {

    private Timer timer;
    private double lastUpdateTime;
    private double updateInterval;

    /* VisionWatchdog tracks time since it's last update was called, and only prints if the time interval has been longer than defined
     * Prevents hammering prints for every vision update
     */
    public VisionWatchdog(double updateInterval){
        timer.start();
        this.updateInterval = updateInterval;
    }

    public void update(Pose2d currentPose, Pose2d lastPose){
        double currentTime = timer.get();
        double timeDiff = currentTime-lastUpdateTime;
        if(timeDiff > updateInterval){
            double xDiff = Math.abs(currentPose.getX() - lastPose.getX());
            double yDiff = Math.abs(currentPose.getY() - lastPose.getY());
            System.out.println("***Vision Pose Update - " + timeDiff + "s since last update.  X,Y Diff = ("+xDiff+","+yDiff+").");
        }
        lastUpdateTime = currentTime;
    }
}
