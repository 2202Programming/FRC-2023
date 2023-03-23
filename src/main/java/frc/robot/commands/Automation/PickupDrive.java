package frc.robot.commands.Automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.ColorSensors.GamePiece;
import frc.robot.subsystems.SwerveDrivetrain;

public class PickupDrive extends CommandBase {
    // Subsystems
    SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
    Claw_Substyem claw = RobotContainer.RC().claw;

    // State vars
    final GamePiece pieceType;
    int timeBroken = 0;

    // Constants
    final double SPEED; // [m/s] speed of driving
    final int FRAMES_BREAK = 5; // [count] num frames needed to affirm an object in claw

    /**
     * Constructs a new PickupDrive, meant for picking up from double substation
     * 
     * @param pieceType Which piece will be picked up
     */
    public PickupDrive(double speed, GamePiece pieceType) {
        this.pieceType = pieceType;
        this.SPEED = speed;
        addRequirements(sdt);
    }

    @Override
    public void initialize() {
        System.out.println("Starting init");;
        //var states = sdt.getKinematics().toSwerveModuleStates(new ChassisSpeeds(SPEED, 0.0, 0.0));

        // slowly drive forward, open claw
        //sdt.drive(states);

        switch (pieceType) {
            case Cube:
                claw.wheelsIn();
                break;
            default:
                claw.open();
        }
        System.out.println("finished init");
    }

    @Override
    public void execute() {
        timeBroken = (claw.isGateBlocked()) ? timeBroken + 1 : 0;
        System.out.println("Status: timeBroken is " + timeBroken + ", claw blocked? is " + claw.isGateBlocked());
    }

    @Override
    public boolean isFinished() {
        // can stop moving and pick up piece if lightgate broken for long enough
        return (timeBroken >= FRAMES_BREAK);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Finished");
        // stop moving, shut claw
        sdt.stop();

        switch (pieceType) {
            case Cube:
                claw.wheelsOff();
                break;
            default:
                claw.close();
        }
    }
}
