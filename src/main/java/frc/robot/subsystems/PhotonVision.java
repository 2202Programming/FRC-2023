// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */

  private NetworkTable table;
  private NetworkTableEntry targetPixelsX;
  private NetworkTableEntry targetPixelsY;
  private NetworkTableEntry targetPixelsArea;
  private NetworkTableEntry hasTarget;
  
  private double m_targetPixelsX;
  private double m_targetPixelsY;
  private double m_targetPixelsArea;
  private boolean m_hasTarget;

  public PhotonVision() {
    table = NetworkTableInstance.getDefault().getTable("photonvision");
    targetPixelsX = table.getEntry("Global_Shutter_Camera/targetPixelsX");
    targetPixelsY = table.getEntry("Global_Shutter_Camera/targetPixelsY");
    targetPixelsArea = table.getEntry("Global_Shutter_Camera/targetArea");
    hasTarget = table.getEntry("Global_Shutter_Camera/hasTarget");
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_targetPixelsX = targetPixelsX.getDouble(0);
    m_targetPixelsY = targetPixelsY.getDouble(0);
    m_targetPixelsArea = targetPixelsArea.getDouble(0);
    m_hasTarget = hasTarget.getBoolean(false);

    SmartDashboard.putNumber("targetX", m_targetPixelsX);
    SmartDashboard.putNumber("targetY", m_targetPixelsY);
    SmartDashboard.putNumber("targetArea", m_targetPixelsArea);
    SmartDashboard.putBoolean("hasTarget", m_hasTarget);

  }
}
