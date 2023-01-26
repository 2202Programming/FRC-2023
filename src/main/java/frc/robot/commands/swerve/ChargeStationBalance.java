package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
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

public class ChargeStationBalance extends CommandBase {

    final boolean exitOnLevel; // mode
    // Constants, some may be beter as args or from Constants.java
    final double vmax = 0.4; // [m/s] fastest speed we allow
    final double vmin = 0.0; // [m/s] small stiction speed if there is tilt, sign corrected
    // also could be simple bang-bang...
    final double roll_offset = -0.8349609375; // [deg] simple sensor correction TODO:calibrate in sensor_SS

    // tolerance limits
    final double rollPosTol = 1.5; // [deg] level more or less
    final double rollRateTol = 3.0; // [deg/s] little motion TODO: tell it to stop being annoying
    final int levelN = 5; // [frame-counts] stable roll for n frames, maybe use FIR?                          

    // Subsystems
    SwerveDrivetrain sdt; // required
    Sensors_Subsystem sensors; // read-only, not require

    //measured & output values
    double rollRate;                    //[deg/s]
    double xSpeed_r;                    //[m/s]  controller on roll
    double xSpeed_rr;                   //[m/s]  controller on rollRate
    double xSpeed;                      //[m/s]  output vel
    double unfilteredRoll;              //[deg/s]
    double filteredRoll;                //[deg/s]
    double prev_filteredRoll;           //[deg/s]

    // state vars, cleared on init()
    int levelCount;
    PIDController csBalancePID = new PIDController(0.025, 0.0, 0.0); // kp [m/s per deg]
    double kpRR = 0.0;   //[m/s/deg/s] vel compensation based on direct rollRate

    LinearFilter rollFilter = LinearFilter.singlePoleIIR(0.07, Constants.DT);
    LinearFilter rollRateFilter = LinearFilter.singlePoleIIR(0.06, Constants.DT);

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
        csBalancePID.setTolerance(rollPosTol, rollRateTol);

        // NT creation
        ntcreate();
    }

    @Override
    public void initialize() {
        levelCount = 0;
        unfilteredRoll = sensors.getRoll() - roll_offset;
        rollRate = sensors.getRollRate();
        csBalancePID.reset();
        
        //reset, use current measured roll & rollRate to initialize
        rollFilter.calculate(unfilteredRoll);
        rollFilter.calculate(unfilteredRoll);

        // Same for roll Rate
        rollFilter.reset();
        rollRateFilter.calculate(rollRate);
        rollRateFilter.calculate(rollRate);

        rollRateFilter.reset();
        System.out.println("***Starting automatic charging station balancing***");
    }

    @Override
    public void execute() {
        ntupdates();
        sdt.drive(calculate());
    }

    @Override
    public void end(boolean interrupted) {
        sdt.stop();
        System.out.println("***Ending charging station balancing***");
    }

    /**
     * 
     * Calculates swerve module command vales based on PID loop around
     * roll.
     * 
     * @return SwerveModuleState[] swerve speeds to command
     */
    private SwerveModuleState[] calculate() {
        // double tiltRate = sensors.getTotalTiltRate(); // TODO: hard to do a neg.
        // feedback loop when this is always >0
        // double tilt = sensors.getTotalTilt(); // dito no indication of direction, really want net tilt in direction of ramp-plane (a bit hard)
        // double yaw = Math.toRadians(sensors.getYaw()); //sensor are in degrees, just keep that for intuition
        
        //TODO: filter magic number to constants
        
        // try simple 1-D around roll
        unfilteredRoll = sensors.getRoll() - roll_offset;
        filteredRoll = rollFilter.calculate(unfilteredRoll); // simple best guess of our roll after physical alignment
        rollRate = rollRateFilter.calculate(sensors.getRollRate());

        // pid speed + min speed in proper direction, then clamp to our max
        xSpeed_r = csBalancePID.calculate(filteredRoll);
        xSpeed_rr = kpRR*rollRate;
        xSpeed = MathUtil.clamp(xSpeed + xSpeed_rr, -vmax, vmax);

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
    }

    private void ntupdates() {
        // info
        nt_framesStable.setDouble(levelCount);
        nt_atSetpoint.setBoolean(csBalancePID.atSetpoint());
        nt_filteredGyro.setDouble(filteredRoll);
        nt_unfilteredGyro.setDouble(unfilteredRoll);
        nt_rollRate.setDouble(rollRate);
        nt_xSpeed.setDouble(xSpeed);
        nt_xSpeed.setDouble(xSpeed_r);
        nt_xSpeedRR.setDouble(xSpeed_rr);

        // setters
        csBalancePID.setP(nt_kP.getDouble(0.02));
        csBalancePID.setI(nt_kI.getDouble(0.0));
        csBalancePID.setD(nt_kD.getDouble(0.0));
        
        //rollrate kp
        kpRR = nt_kD.getDouble(0.0);

    }
}
