package frc.robot.commands.Intake.Washer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake_Constants;
import frc.robot.subsystems.Intake;

public class IntakeForward extends CommandBase {

    final double direction = 1.0;
    final double speed; 
    final Intake intake;
    final boolean useLightgate;

    public IntakeForward() {
        this(false);
    }

    public IntakeForward(boolean useLightgate) {
       this(useLightgate, Intake_Constants.IntakeMotorStrength);
    }

    public IntakeForward(boolean useLightgate, double speed) {
        intake = RobotContainer.RC().intake;
        this.useLightgate = useLightgate;
        this.speed = Math.abs(speed)*direction;
    }

    @Override
    public void initialize() {
        intake.setIntakeSpeed(speed);
    }

    /*
     * runs until stopped or if the lightgate is used, stops when
     * the lightgate is blocked.
     */
    @Override
    public boolean isFinished() {
        return useLightgate && intake.lightgateIsBlocked();
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeOff();
    }
    
}
