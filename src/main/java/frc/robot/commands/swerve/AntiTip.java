// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.RobotContainer;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.subsystems.Sensors_Subsystem;

public class AntiTip {


    double roll_kP = 0.05;
    double roll_kI = 0.0;
    double roll_kD = 0.0;
    double tipRollPidOutput = 0.0;
    PIDController tipRollPid = new PIDController(roll_kP, roll_kI, roll_kD);
  
    double pitch_kP = 0.05;
    double pitch_kI = 0.0;
    double pitch_kD = 0.0;
    double tipPitchPidOutput = 0.0;
    PIDController tipPitchPid = new PIDController(pitch_kP, pitch_kI, pitch_kD);

    private ChassisSpeeds robotCentricCorrection;
    private Sensors_Subsystem sensors;

    double kOffBalanceAngleThresholdDegrees; 
    double kOnBalanceAngleThresholdDegrees;
    boolean tip_correction_mode = false;
    
   /**
   * Constructs a AntiTip
   *
   * @param kOffBalanceAngleThresholdDegrees degrees to start tip correction
   * @param kOnBalanceAngleThresholdDegrees degrees to stop tip correction
   */
    public AntiTip(double kOffBalanceAngleThresholdDegrees, double kOnBalanceAngleThresholdDegrees){
        sensors = RobotContainer.RC().sensors;
        this.kOffBalanceAngleThresholdDegrees = kOffBalanceAngleThresholdDegrees;
        this.kOnBalanceAngleThresholdDegrees = kOnBalanceAngleThresholdDegrees;
    }

    public AntiTip(){
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

        tipPitchPid.setSetpoint(0); 
        tipPitchPidOutput = tipPitchPid.calculate(-pitchAngleDegrees);

        robotCentricCorrection = new ChassisSpeeds(tipPitchPidOutput, tipRollPidOutput, 0); //create chassis speeds from robot centric pitch and roll (X and Y)

        //TODO: only enter tip correction if arm / elbow is extended enough -- do we determine this by taking the magnitude of extension w/ trig? 
        // TODO or ignore the height and just use the magnitude?

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
        // This function is the rotating of a vector counterclockwise by θ radians.
        /*
         * Matrix for rotating clockwise:
         *  -                      -
         * |    cos(θ)  -sin(θ)     |
         * |    sin(θ)   cos(θ)     |
         *  -                      -
         */
        Matrix<N2, N2> angleMatrix = Matrix.mat(Nat.N2(), Nat.N2()).fill(robotAngle.getCos(), robotAngle.getSin(), -robotAngle.getSin(), robotAngle.getCos());

        /*
         *  Current velocity vector: 
         *   -         -
         *  |   v(x)    |
         *  |   v(y)    |
         *   -         -
         */
        Matrix<N2, N1> velocityVector = Matrix.mat(Nat.N2(), Nat.N1()).fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);

        /*
         *   (Angle matrix)(Velocity vector) = (transformed velocity vector, which is the original velocity vector rotated by θ)
         *  -                      -     -        -            -         -  
         * |    cos(θ)  -sin(θ)     |   |   v(x)   |   ----   |   v1(x)   |
         * |    sin(θ)   cos(θ)     |   |   v(y)   |   ----   |   v1(y)   |
         *  -                      -     -        -            -         -
         */
        Matrix<N2, N1> transformedVelocityVector = angleMatrix.times(velocityVector);
        
        // vx, vy, ω
        return new ChassisSpeeds(transformedVelocityVector.get(0, 0), transformedVelocityVector.get(1, 0), chassisSpeeds.omegaRadiansPerSecond);
    }

    class AntiTipWatcher extends WatcherCmd {
        
        public final String NT_Name = "DT/AntiTip"; // expose data under DriveTrain table
        NetworkTableEntry nt_roll_factor;
        NetworkTableEntry nt_pitch_factor;
        NetworkTable table;

        @Override
        public void ntcreate() {
            table = NetworkTableInstance.getDefault().getTable(NT_Name);
            nt_roll_factor = table.getEntry("/DriveController/RollFactor");
            nt_pitch_factor = table.getEntry("/DriveController/PitchFactor");
        }

        @Override
        public void ntupdate() {
            nt_roll_factor.setDouble(tipRollPidOutput);
            nt_pitch_factor.setDouble(tipPitchPidOutput);
        }

        @Override
        public String getTableName() {
            return NT_Name;
        }
    }

}
