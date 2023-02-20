package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;

/*
 *
 * feedback/discussion:
 *  Good work, lots of the hard things figured out or very very close. Main problem 
 *  I see is a blended roll/theta being used, but that value is always posititive
 *  so it won't work to catch changing direction or running from different sides.
 *  Angles don't blend that way.
 * 
 * I recommend we decouple the problem first. Don't use roll and roll.  pick one, and
 * alingn the robot to the ChargeStn.  Here's a first cut using roll.
 * 
 * 
 * Description:
 * Robot is moved into a positon on one side of the ramp and up the ramp so the bot is tilted.
 * The heading is set to align with the field so it can drive straight up the ramp.  A rotate-to
 * command could preceed this one to rotate the bot to the proper field heading. This command
 * just deals with the balancing.
 *
 *  Assumptions:
 *      Robot is on the ramp, with an initial tilt.
 *      Robot is aligned, forward/back, only roll[theta] is used
 *      Either side of charge station can work, roll angle used to determin which way to move.
 *      Robot moves only in forward/back directions based on sign of roll.
 *      PID around roll controls speed, may need vmin for friction?
 *              vmin could just be a bang-bang control and no pid.
 *  
 *      We detect level for N frames then exit.
 *      May want to low-pass filter roll for level detection
 *      Exit-on-level : false --> keep running 
 *                      true --> command ends when level
 * 
 *      TODO: look at PID atSetpoint() see why we don't exit
 *      TODO: calibrate all the angles somewhere?
 * 
 *      TODO: may need to correct roll and roll with any offsets due to alignment
 *      if it is really badly aligned, back out corrected angles with eular angles for coupling
 *      TODO: look at pigeon2 docs for mounting corections, there may be some calibration to help.
 * 
 *      TODO: clean up discussion comments and dead code when we all agree and is tested
 * 
 */

public class ChargeStationBalance extends CommandBase implements BlinkyLightUser {

    final boolean exitOnLevel; // mode
    // Constants, some may be beter as args or from Constants.java
    final double vmax = 0.9; // [m/s] fastest speed we allow
    final double vmin = 0.0; // [m/s] small stiction speed if there is tilt, sign corrected  TODO: Try stiction fix to speed steady-state climb
    // also could be simple bang-bang...
    final double pitch_offset = -0.8349609375; // [deg] simple sensor correction TODO:calibrate in sensor_SS

    // tolerance limits
    final double pitchPosTol = 1.5; // [deg] level more or less
    final double pitchRateTol = 3.0; // [deg/s] little motion TODO: tell it to stop being annoying
    final int levelN = 5; // [frame-counts] stable roll for n frames, maybe use FIR?                          

    // Subsystems
    SwerveDrivetrain sdt; // required
    Sensors_Subsystem sensors; // read-only, not require

    //measured & output values
    double pitchRate;                    //[deg/s]
    double xSpeed_p;                    //[m/s]  controller on roll
    double xSpeed_pr;                   //[m/s]  controller on rollRate
    double xSpeed;                      //[m/s]  output vel
    double unfilteredPitch;              //[deg/s]
    double filteredPitch;                //[deg/s]
    double prev_filteredPitch;           //[deg/s]

    // state vars, cleared on init()
    int levelCount;
    PIDController csBalancePID = new PIDController(0.015, 0.0, 0.000); // kp [m/s per deg] kd .003 or less for TIM,  kp >.018 is unstable
    double kpPR = -0.0055;   //[m/s/deg/s] vel compensation based on direct rollRate

    LinearFilter pitchFilter = LinearFilter.singlePoleIIR(0.3, Constants.DT);
    LinearFilter pitchRateFilter = LinearFilter.singlePoleIIR(0.1, Constants.DT);


    public ChargeStationBalance() {
        this(true);
    }

    public ChargeStationBalance(boolean exitOnLevel) {
        this.exitOnLevel = exitOnLevel;
        sdt = RobotContainer.RC().drivetrain;
        sensors = RobotContainer.RC().sensors;
        addRequirements(sdt);

        // pid setpoint is always 0.0, aka level, include tolerances
        csBalancePID.setSetpoint(0.0);
        csBalancePID.setTolerance(pitchPosTol, pitchRateTol);

        // NT creation
        ntcreate();
    }

    @Override
    public void initialize() {
        levelCount = 0;
        unfilteredPitch = sensors.getPitch() - pitch_offset - 2.0;
        pitchRate = sensors.getPitchRate();
       
        //reset, use current measured roll & rollRate to initialize
        pitchFilter.calculate(unfilteredPitch);
        pitchFilter.calculate(unfilteredPitch);
        filteredPitch = pitchFilter.calculate(unfilteredPitch);

        // Same for roll Rate, should be zero
        pitchRateFilter.reset();
        pitchRateFilter.calculate(pitchRate);
        pitchRateFilter.calculate(pitchRate);

        // set PID internal states
        csBalancePID.reset();
        csBalancePID.calculate(filteredPitch);
        csBalancePID.calculate(filteredPitch);

        enableLights();
    }

    @Override
    public Color8Bit colorProvider() {
        return BlinkyLights.GREEN;
    }

    @Override
    public boolean requestBlink() {
        return !isLevel();     //blink when not level
    }

    @Override
    public void execute() {
        ntupdates();
        sdt.drive(calculate());
    }

    @Override
    public void end(boolean interrupted) {
        sdt.stop();
    }

    /**
     * 
     * Calculates swerve module command vales based on PID loop around
     * roll.
     * 
     * @return SwerveModuleState[] swerve speeds to command
     */
    private SwerveModuleState[] calculate() {
        // try simple 1-D around roll
        unfilteredPitch = sensors.getPitch() - pitch_offset - 2.0;
        filteredPitch = pitchFilter.calculate(unfilteredPitch); // simple best guess of our roll after physical alignment
        pitchRate = pitchRateFilter.calculate(sensors.getPitchRate());

        // pid speed + min speed in proper direction, then clamp to our max
        xSpeed_p = csBalancePID.calculate(filteredPitch);
        xSpeed_pr = kpPR*pitchRate;
        xSpeed = MathUtil.clamp(xSpeed_p + xSpeed_pr, -vmax, vmax);

        /**
         * if we use an unaligned start, decouple the desired speed into components
         * double xSpeed = MathUtil.clamp(speed * Math.cos(yaw), -vmax, vmax);
         * double ySpeed = MathUtil.clamp(speed * Math.sin(yaw), -vmax, vmax);
         * 
         * Keep simple 1-D for now
         */

        // check for level using PID's tolerance, failure resets counter
        levelCount = (csBalancePID.atSetpoint()) ? ++levelCount : 0;

        // if at setpoint force xSpeed to 0
        xSpeed = csBalancePID.atSetpoint() ? 0 : xSpeed;

        // move the robot our desired speed, forward, zero y, and keep
        // current heading, no rotation
        return sdt.getKinematics().toSwerveModuleStates(new ChassisSpeeds(xSpeed, 0.0, 0.0));
    }

    // we must be at pid's tolerance for N frames before we call it good
    boolean isLevel() {
        return levelCount >= levelN;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return exitOnLevel && isLevel();
    }


    /**
     * ==================
     * NT stuff
     * ==================
     */

    NetworkTable table = NetworkTableInstance.getDefault().getTable("csBalance");

    NetworkTableEntry nt_rollOffset;
    NetworkTableEntry nt_framesStable;
    NetworkTableEntry nt_atSetpoint;
    NetworkTableEntry nt_kP;
    NetworkTableEntry nt_kI;
    NetworkTableEntry nt_kD;
    NetworkTableEntry nt_filteredGyro;
    NetworkTableEntry nt_unfilteredGyro;
    NetworkTableEntry nt_rollRate;
    NetworkTableEntry nt_xSpeed;
    NetworkTableEntry nt_xSpeedR;
    NetworkTableEntry nt_xSpeedRR;
    NetworkTableEntry nt_KPRR;


    private void ntcreate() {
        nt_framesStable = table.getEntry("Frames Stable");
        nt_atSetpoint = table.getEntry("At Setpoint?");
        nt_kP = table.getEntry("kP");
        nt_kI = table.getEntry("kI");
        nt_kD = table.getEntry("kD");
        nt_filteredGyro = table.getEntry("Filtered Roll");
        nt_unfilteredGyro = table.getEntry("Unfiltered Roll");
        nt_rollRate = table.getEntry("fRollRate");
        nt_xSpeed = table.getEntry("xSpeed");
        nt_xSpeedR = table.getEntry("xSpeedRoll");
        nt_xSpeedRR = table.getEntry("xSpeedRR");
        nt_KPRR = table.getEntry("KPRR");

        nt_kP.setDouble(csBalancePID.getP());
        nt_kI.setDouble(csBalancePID.getI());
        nt_kD.setDouble(csBalancePID.getD());
        nt_KPRR.setDouble(kpPR);
    }

    private void ntupdates() {
        // info
        nt_framesStable.setDouble(levelCount);
        nt_atSetpoint.setBoolean(csBalancePID.atSetpoint());
        nt_filteredGyro.setDouble(filteredPitch);
        nt_unfilteredGyro.setDouble(unfilteredPitch);
        nt_rollRate.setDouble(pitchRate);
        nt_xSpeed.setDouble(xSpeed);
        nt_xSpeedR.setDouble(xSpeed_p);
        nt_xSpeedRR.setDouble(xSpeed_pr);

        // setters
        csBalancePID.setP(nt_kP.getDouble(0.02));
        csBalancePID.setI(nt_kI.getDouble(0.0));
        csBalancePID.setD(nt_kD.getDouble(0.0));
        
        //rollrate kp
        kpPR = nt_KPRR.getDouble(0.0);

    }
}
