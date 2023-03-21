package frc.robot.commands.Automation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ColorSensors.GamePiece;

public class PickupDrive extends CommandBase {
    // Subsystems
    SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
    Claw_Substyem claw = RobotContainer.RC().claw;

    // State vars
    final GamePiece pieceType;
    int timeBroken = 0;

    // Constants
    final double SPEED = 0.5; // [m/s] speed of driving
    final int FRAMES_BREAK = 5; // [count] num frames needed to affirm an object in claw

    /**
     * Constructs a new PickupDrive, meant for picking up from double substation
     * 
     * @param pieceType Which piece will be picked up
     */
    public PickupDrive(GamePiece pieceType) {
        this.pieceType = pieceType;
    }

    @Override
    public void initialize() {
        SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(SPEED, new Rotation2d()),
            new SwerveModuleState(SPEED, new Rotation2d()),
            new SwerveModuleState(SPEED, new Rotation2d()),
            new SwerveModuleState(SPEED, new Rotation2d())
        };

        // slowly drive forward, open claw
        sdt.drive(states);

        switch (pieceType) {
            case Cube:
                claw.wheelsIn();
                break;
            default:
                claw.open();
        }
    }

    @Override
    public void execute() {
        timeBroken = (claw.isGateBlocked()) ? timeBroken + 1 : 0;
    }

    @Override
    public boolean isFinished() {
        // can stop moving and pick up piece if lightgate broken for long enough
        return (timeBroken >= FRAMES_BREAK);
    }

    @Override
    public void end(boolean interrupted) {
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
