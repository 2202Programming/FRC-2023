// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors_Subsystem;

public class AntiTipWatcher {


    PIDController tipRollPid;
    double roll_kP = 0.05;
    double roll_kI = 0.0;
    double roll_kD = 0.0;
    double tipRollPidOutput = 0.0;
  
    PIDController tipPitchPid;
    double pitch_kP = 0.05;
    double pitch_kI = 0.0;
    double pitch_kD = 0.0;
    double tipPitchPidOutput = 0.0;

    private ChassisSpeeds robotCentricCorrection;
    private Sensors_Subsystem sensors;

    double kOffBalanceAngleThresholdDegrees; 
    double kOnBalanceAngleThresholdDegrees;
    boolean tip_correction_mode = false;

    public final String NT_Name = "DT"; // expose data under DriveTrain table
    NetworkTableEntry nt_roll_factor;
    NetworkTableEntry nt_pitch_factor;
    NetworkTable table;
    
   /**
   * Constructs a AntiTipWatcher
   *
   * @param kOffBalanceAngleThresholdDegrees degrees to start tip correction
   * @param kOnBalanceAngleThresholdDegrees degrees to stop tip correction
   */
    public AntiTipWatcher(double kOffBalanceAngleThresholdDegrees, double kOnBalanceAngleThresholdDegrees){
        sensors = RobotContainer.RC().sensors;
        this.kOffBalanceAngleThresholdDegrees = kOffBalanceAngleThresholdDegrees;
        this.kOnBalanceAngleThresholdDegrees = kOnBalanceAngleThresholdDegrees;

        table = NetworkTableInstance.getDefault().getTable(NT_Name);
        nt_roll_factor = table.getEntry("/DriveController/RollFactor");
        nt_pitch_factor = table.getEntry("/DriveController/PitchFactor");
    }

    public AntiTipWatcher(){
        this(4.0,3.0); //default values for tip threshold angles
    }

    public void tipCalculate(){
    
        //NOTE: PITCH IS FRONT/BACK OF ROBOT, Positive towards intake
        //NOTE: ROLL IS POSITIVE WITH CLOCKWISE ROTATION (LOOKING FROM BACK TOWARDS INTAKE)
        //Y direction is left/right, positive towards left when facing intake from back

        double pitchAngleDegrees = sensors.getPitch();
        double rollAngleDegrees = sensors.getRoll();

        tipRollPid.setSetpoint(0); 
        tipRollPidOutput = tipRollPid.calculate(rollAngleDegrees);
        nt_roll_factor.setDouble(tipRollPidOutput);

        tipPitchPid.setSetpoint(0); 
        tipPitchPidOutput = tipPitchPid.calculate(-pitchAngleDegrees);
        nt_pitch_factor.setDouble(tipPitchPidOutput);

        robotCentricCorrection = new ChassisSpeeds(tipPitchPidOutput, tipRollPidOutput, 0); //create chassis speeds from robot centric pitch and roll (X and Y)

        //TODO: only enter tip correction if arm is extended

        //enter tip correction mode if either roll or pitch is high enough
        if (!tip_correction_mode &&
            (Math.abs(pitchAngleDegrees)>kOnBalanceAngleThresholdDegrees ||
            Math.abs(rollAngleDegrees)>kOnBalanceAngleThresholdDegrees))
        {
            tip_correction_mode = true;
            System.out.println("***STARTING TIP CORRECTION: Pitch="+pitchAngleDegrees+" Roll="+rollAngleDegrees);
        }
        
        //exit tip correction mode if both are low enough
        if (tip_correction_mode &&
                (Math.abs(pitchAngleDegrees)<kOffBalanceAngleThresholdDegrees &&
                Math.abs(rollAngleDegrees)<kOffBalanceAngleThresholdDegrees))
        {
            tip_correction_mode = false;
            System.out.println("***END TIP CORRECTION: Pitch="+pitchAngleDegrees+" Roll="+rollAngleDegrees);
        }

    }

    //return tip correction in robot-centric
    public ChassisSpeeds getRobotCentricTipCorrection(){
        return (tip_correction_mode) ? robotCentricCorrection : new ChassisSpeeds(0.0,0.0,0.0); //return zero if not in tip correction mode
    }
    
    //return tip correction in field-centric
    public ChassisSpeeds getFieldCentricTipCorrection(){
        if (!tip_correction_mode) return new ChassisSpeeds(0.0,0.0,0.0); //return zero if not in tip correction mode

        return getFieldRelativeSpeed(robotCentricCorrection, RobotContainer.RC().drivetrain.getPose().getRotation());
        
    }

    //convert (rotate) robot-centric to field centric
    ChassisSpeeds getFieldRelativeSpeed(ChassisSpeeds chassisSpeeds, Rotation2d robotAngle) {

        double robotX = chassisSpeeds.vxMetersPerSecond;
        double robotY = chassisSpeeds.vyMetersPerSecond;

        // Gets the field relative X and Y components of the robot's speed in the X axis
        double robotXXComp = robotAngle.getCos() * robotX;
        double robotXYComp = robotAngle.getSin() * robotX;

        // Gets the field relative X and Y components of the robot's speed in the Y axis
        double robotYXComp = robotAngle.getSin() * robotY;
        double robotYYComp = robotAngle.getCos() * robotY;

        // Adds the field relative X and Y components of the robot's X and Y speeds to get the overall field relative X and Y speeds
        double fieldX = robotXXComp + robotYXComp;
        double fieldY = robotXYComp + robotYYComp;
        
        return new ChassisSpeeds(fieldX, fieldY, chassisSpeeds.omegaRadiansPerSecond);
    }

}
