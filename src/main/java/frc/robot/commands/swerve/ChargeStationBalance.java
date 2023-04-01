package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkyLights;
import frc.robot.subsystems.BlinkyLights.BlinkyLightUser;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.commands.utility.WatcherCmd;

/*
 *
 * Description:
 * Robot is moved to one side of the ramp and up the ramp so the bot is tilted.
 * The heading is set to align with the field so it can drive straight up the ramp.  A rotate-to
 * command could preceed this one to rotate the bot to the proper field heading. 
 * This command just deals with the balancing.
 *
 *  Assumptions:
 *      Robot is on the ramp, with an initial tilt.
 *      Robot is aligned, forward/back, only roll[theta] is used
 *      Either side of charge station can work, roll angle used to determin which way to move.
 *      Robot moves only in forward/back directions based on sign of roll.
 *      PID around pitch controls speed, may need vmin for friction?
 *              vmin could just be a bang-bang control and no pid.
 *  
 *      We detect level for N frames then exit.
 *   
 *      Exit-on-level : false --> keep running, till button
 *                      true --> command ends when level
 * 
 */

public class ChargeStationBalance extends CommandBase implements BlinkyLightUser {

    final boolean exitOnLevel; // mode
    // Constants, some may be beter as args or from Constants.java
    final double vmax = 0.9; // [m/s] fastest speed we allow

    // non-linear
    static final double VMIN_CLIMB_DEFAULT = 0.45; // [m/s] small stiction speed if there is tilt, sign corrected
    final double PITCHRATE_DETECTED = 25.0; // [deg/s] - we are moving
    final double MIN_PITCH = 5.0; // [deg] min angle to know we are on the charge station (ramp)

    // allow the command to have different vmin_climb
    final double vmin_climb;

    // tolerance limits
    final double pitchPosTol = 1.5; // [deg] level more or less
    final double pitchRateTol = 3.0; // [deg/s] allow a little motion
    final int levelN = 5; // [frame-counts] stable pitch for n frames

    // Subsystems
    SwerveDrivetrain sdt; // required
    Sensors_Subsystem sensors; // read-only, not required

    // measured & output values
    double pitchRate; // [deg/s]
    double xSpeed_p; // [m/s] controller on pitch
    double xSpeed_pr; // [m/s] controller on pitchRate
    double xSpeed; // [m/s] output vel
    double unfilteredPitch; // [deg/s]
    double filteredPitch; // [deg/s]
    double prev_filteredPitch; // [deg/s]

    // state vars, set on init()
    int levelCount;
    double vmin;

    // kp [m/s per deg] kd .003 or less for TIM, kp > 0.018 is unstable
    PIDController csBalancePID = new PIDController(0.018, 0.0, 0.000);                                                                   

    // tunable constants
    double kpPR = -0.0055; // [m/s/deg/s] vel compensation based on direct pitchRate

    LinearFilter pitchFilter = LinearFilter.singlePoleIIR(0.3, Constants.DT);
    LinearFilter pitchRateFilter = LinearFilter.singlePoleIIR(0.1, Constants.DT);

    public ChargeStationBalance() {
        this(true);
    }

    public ChargeStationBalance(boolean exitOnLevel) {
        this(true, VMIN_CLIMB_DEFAULT);
    }

    public ChargeStationBalance(double vmin_climb) {
        this(true, vmin_climb);
    }

    public ChargeStationBalance(boolean exitOnLevel, double vmin_climb) {
        this.exitOnLevel = exitOnLevel;
        sdt = RobotContainer.RC().drivetrain;
        sensors = RobotContainer.RC().sensors;
        this.vmin_climb = vmin_climb;
        addRequirements(sdt);

        // pid setpoint is always 0.0, aka level, include tolerances
        csBalancePID.setSetpoint(0.0);
        csBalancePID.setTolerance(pitchPosTol, pitchRateTol);
    }

    @Override
    public void initialize() {
        levelCount = 0;
        unfilteredPitch = sensors.getPitch();
        pitchRate = sensors.getPitchRate();

        // non-linear, copy sign of current pitch to move in correct direction
        // or don't add vmin if we are level. note the sign of pitch being flipped.
        vmin = (Math.abs(unfilteredPitch) > MIN_PITCH) ? Math.copySign(vmin_climb, -unfilteredPitch) : 0.0;

        // reset, use current measured pitch & pitchRate to initialize
        pitchFilter.reset();
        filteredPitch = pitchFilter.calculate(unfilteredPitch);

        // Same for pitch Rate, should be zero
        pitchRateFilter.reset();
        pitchRateFilter.calculate(pitchRate); // prime filter with a measurement

        // set PID internal states
        csBalancePID.reset();
        csBalancePID.calculate(filteredPitch);

        enableLights();
    }

    @Override
    public Color8Bit colorProvider() {
        if (vmin == 0.0) return BlinkyLights.WHITE;
        return BlinkyLights.GREEN;
    }

    @Override
    public boolean requestBlink() {
        return !isLevel(); // blink when not level
    }

    @Override
    public void execute() {
        sdt.drive(calculate());
    }

    @Override
    public void end(boolean interrupted) {
        sdt.stop();
    }

    /**
     * 
     * Calculates swerve module command vales based on PID loop around
     * pitch, pitchrate and const climb speed. Const climb removed
     * once we start rotating.
     * 
     * @return SwerveModuleState[] swerve speeds to command
     */
    private SwerveModuleState[] calculate() {
        // try simple 1-D around roll
        unfilteredPitch = sensors.getPitch();
        filteredPitch = pitchFilter.calculate(unfilteredPitch);
        pitchRate = pitchRateFilter.calculate(sensors.getPitchRate());

        // non-linear vmin to speed ramp climb, zero it when pitch rate is detected
        vmin = (Math.abs(pitchRate) > PITCHRATE_DETECTED) ? 0.0 : vmin;

        // pid speed + min speed in proper direction, then clamp to our max
        xSpeed_p = csBalancePID.calculate(filteredPitch);
        xSpeed_pr = kpPR * pitchRate;
        xSpeed = MathUtil.clamp(xSpeed_p + xSpeed_pr + vmin, -vmax, vmax);

        // check for level using PID's tolerance, failure resets counter
        levelCount = (csBalancePID.atSetpoint()) ? ++levelCount : 0;

        // if at setpoint force xSpeed to 0
        xSpeed = csBalancePID.atSetpoint() ? 0.0 : xSpeed;

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

    public Command getWatcher() {
        return new MyWatcher();
    }

    /**
     * ==================
     * NT stuff
     * ==================
     */

    class MyWatcher extends WatcherCmd {
        /*
         * Publish = PUBLISHing data TO the NT
         * Subscribe = SUBSCRIBing to data FROM the NT
         */
        DoublePublisher nt_pub_rollOffset;
        DoublePublisher nt_pub_framesStable;
        BooleanPublisher nt_pub_atSetpoint;
        DoubleSubscriber nt_sub_kP;
        DoubleSubscriber nt_sub_kI;
        DoubleSubscriber nt_sub_kD;
        DoubleSubscriber nt_pub_kPRR;
        DoublePublisher nt_pub_filteredGyro;
        DoublePublisher nt_pub_unfilteredGyro;
        DoublePublisher nt_pub_rollRate;
        DoublePublisher nt_pub_xSpeed;
        DoublePublisher nt_pub_xSpeedR;
        DoublePublisher nt_pub_xSpeedRR;

        @Override
        public String getTableName() {
            return ChargeStationBalance.this.getName();
        }

        @Override
        public void ntcreate() {
            NetworkTable table = getTable();
            nt_pub_framesStable = table.getDoubleTopic("Frames Stable").publish();
            nt_pub_atSetpoint = table.getBooleanTopic("At Setpoint?").publish();
            nt_sub_kP = table.getDoubleTopic("kP").subscribe(csBalancePID.getP());
            nt_sub_kI = table.getDoubleTopic("kI").subscribe(csBalancePID.getI());
            nt_sub_kD = table.getDoubleTopic("kD").subscribe(csBalancePID.getD());
            nt_pub_filteredGyro = table.getDoubleTopic("Filtered Roll").publish();
            nt_pub_unfilteredGyro = table.getDoubleTopic("Unfiltered Roll").publish();
            nt_pub_rollRate = table.getDoubleTopic("fRollRate").publish();
            nt_pub_xSpeed = table.getDoubleTopic("xSpeed").publish();
            nt_pub_xSpeedR = table.getDoubleTopic("xSpeedRoll").publish();
            nt_pub_xSpeedRR = table.getDoubleTopic("xSpeedRR").publish();
            nt_pub_kPRR = table.getDoubleTopic("KPRR").subscribe(kpPR);
        }

        @Override
        public void ntupdate() {
            // publishers
            nt_pub_framesStable.set(levelCount);
            nt_pub_atSetpoint.set(csBalancePID.atSetpoint());
            nt_pub_filteredGyro.set(filteredPitch);
            nt_pub_unfilteredGyro.set(unfilteredPitch);
            nt_pub_rollRate.set(pitchRate);
            nt_pub_xSpeed.set(xSpeed);
            nt_pub_xSpeedR.set(xSpeed_p);
            nt_pub_xSpeedRR.set(xSpeed_pr);

            // setters
            csBalancePID.setP(nt_sub_kP.get());
            csBalancePID.setI(nt_sub_kI.get());
            csBalancePID.setD(nt_sub_kD.get());

            // rollrate kp setter
            kpPR = nt_pub_kPRR.get();

        }
    }
}
