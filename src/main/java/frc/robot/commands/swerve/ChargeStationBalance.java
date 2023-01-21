package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public class ChargeStationBalance extends CommandBase {
    //TODO: move this stuff to constants
    PIDController csBalancePID = new PIDController(0.01, 0.0, 0.0);

    SwerveDrivetrain sdt;
    Sensors_Subsystem sensors;

    public ChargeStationBalance() {
        sdt = RobotContainer.RC().drivetrain;
        sensors = RobotContainer.RC().sensors;
        csBalancePID.setSetpoint(0);
    }

    @Override
    public void initialize() {
        System.out.println("***Starting automatic charging station balancing***");
    }

    @Override
    public void execute() {
        sdt.drive(calculate());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("***Ending charging station balancing***");
        sdt.stop();
    }

    private SwerveModuleState[] calculate() {
        double tiltRate = sensors.getTotalTiltRate();
        double tilt = sensors.getTotalTilt();
        double yaw = Math.toRadians(sensors.getYaw());

        // TODO: arbitrary number, move to constants
        // TODO: how to consider both rotation / rot rate? Assign a "score" or a before-after approach ( as currently implemented)?
        if (Math.abs(tiltRate) > 2) {
            // TODO: fix 
            // but for now trick the pid into thinking no tilt and thus vel should be 0
            tilt = 0;
        }

        // TODO: magic numbers into constants
        // these values are small b/c don't want to overadjust
        double speed = MathUtil.clamp(csBalancePID.calculate(tilt), -0.5, 0.5);
        double xSpeed = MathUtil.clamp(speed * Math.cos(yaw), -0.5, 0.5);
        double ySpeed = MathUtil.clamp(speed * Math.sin(yaw), -0.5, 0.5);

        return sdt.getKinematics().toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, yaw));
    }
}
