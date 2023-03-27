// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    public double getTipRollPidOutput(){
            return (tip_correction_mode) ? tipRollPidOutput : 0.0; //return zero if not in tip correction mode
    }

    public double getTipPitchPidOutput(){
        return (tip_correction_mode) ? tipPitchPidOutput : 0.0; //return zero if not in tip correction mode
    }
    
    public double getFieldCentricTipRollPidOutput(){

        if (!tip_correction_mode) return 0.0;

        Rotation2d currrentHeading = RobotContainer.RC().drivetrain.getPose().getRotation();
        //convert field centric speeds to robot centric
        ChassisSpeeds tempChassisSpeed = new ChassisSpeeds(tipRollPidOutput, 0, 0);

        SwerveModuleState[] output_states = RobotContainer.RC().drivetrain.getKinematics().toSwerveModuleStates(tempChassisSpeed);
        ChassisSpeeds chassisSpeeds = RobotContainer.RC().drivetrain.getKinematics().toChassisSpeeds(output_states);
        
        (DriverStation.getAlliance().equals(Alliance.Blue))
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currrentHeading) 
        : ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, rot, currrentHeading);

        return (tip_correction_mode) ? tipRollPidOutput : 0.0; //return zero if not in tip correction mode
    }

    public double getFieldCentricTipPitchPidOutput(){

        if (!tip_correction_mode) return 0.0;

        return (tip_correction_mode) ? tipPitchPidOutput : 0.0; //return zero if not in tip correction mode
    }


}
