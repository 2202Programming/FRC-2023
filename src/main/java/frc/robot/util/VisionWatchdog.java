// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class VisionWatchdog {

    private Timer timer = new Timer();
    private double lastUpdateTime;
    private double updateInterval;
    private NetworkTable table;
    private NetworkTableEntry nt_diffX;
    private NetworkTableEntry nt_diffY;
    private NetworkTableEntry nt_updateInterval;
    
    public final String NT_Name = "Vision"; // expose data under Vision table

    /* VisionWatchdog tracks time since it's last update was called, and only prints if the time interval has been longer than defined
     * Prevents hammering prints for every vision update
     */
    public VisionWatchdog(double updateInterval){
        timer.start();
        this.updateInterval = updateInterval;
        table = NetworkTableInstance.getDefault().getTable(NT_Name);
        nt_diffX = table.getEntry("/X Diff");
        nt_diffY = table.getEntry("/Y Diff");
        nt_updateInterval = table.getEntry("/Update Interval");
    }

    public void update(Pose2d currentPose, Pose2d lastPose){
        double currentTime = timer.get();
        double timeDiff = currentTime-lastUpdateTime;
        double xDiff = Math.abs(currentPose.getX() - lastPose.getX());
        double yDiff = Math.abs(currentPose.getY() - lastPose.getY());
        nt_diffX.setDouble(xDiff);
        nt_diffY.setDouble(yDiff);
        nt_updateInterval.setDouble(timeDiff);

        if(timeDiff > updateInterval){
            System.out.println("***Vision Pose Update - " + timeDiff + "s since last update.  X,Y Diff = ("+xDiff+","+yDiff+").");
        }
        lastUpdateTime = currentTime;
    }
}
