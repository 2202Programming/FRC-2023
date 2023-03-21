package frc.robot.commands.Placement;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Arm.MoveCollectiveArm.CollectiveMode;
import frc.robot.commands.Automation.PickupDrive;
import frc.robot.commands.auto.goToPickupPosition;
import frc.robot.commands.swerve.RotateTo;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ColorSensors.GamePiece;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

public class Pickup extends CommandBase {
    // substation enum
    public enum Substation {
        Left, Right
    }

    // dc and related vars
    HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
    final double DEADZONE = 0.0;

    // sdt and related vars
    SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
    final double SPEED = 1.0; // [m/s] backup speed
    final double BACKUP_TIME = 1.0; // [s] time to backup for

    // state vars
    final Substation substation;
    final GamePiece gamePiece;
    SequentialCommandGroup cmd = new SequentialCommandGroup();

    public Pickup(Substation substation, GamePiece gamePiece) {
        this.substation = substation;
        this.gamePiece = gamePiece;
    }

    @Override
    public void initialize() {
        move();
        extend();
        getPiece();

        cmd.until(() -> {
            boolean xStickStill = (Math.sqrt(Math.pow(dc.Driver().getLeftX(), 2) + Math.pow(dc.Driver().getLeftY(), 2)) > DEADZONE); 
            boolean yStickStill = (Math.sqrt(Math.pow(dc.Driver().getRightX(), 2) + Math.pow(dc.Driver().getRightY(), 2)) > DEADZONE);
            return !(xStickStill && yStickStill);
          }).schedule();
    }

    /**
     * Moves to the requisite location and rotates afterwards to ensure rotation is
     * on point
     */
    public void move() {
        cmd.addCommands(
                new goToPickupPosition(new PathConstraints(3.0, 3.0), substation),
                new RotateTo(new Rotation2d(DriverStation.getAlliance().equals(Alliance.Blue) ? 180 : 0)));
    }

    /**
     * Extends arm out to pickup position
     */
    public void extend() {
        cmd.addCommands(new MoveCollectiveArm(CollectiveMode.pickupShelfFS));
    }

    /**
     * Picks up piece w/ claw
     */
    public void getPiece() {
        cmd.addCommands(new PickupDrive(gamePiece));
    }

    /**
     * Backs up to safe distance to retract arm
     */
    public void backup() {
        cmd.addCommands(new InstantCommand(() -> {
            sdt.drive(new SwerveModuleState[] {
                    new SwerveModuleState(SPEED, new Rotation2d()),
                    new SwerveModuleState(SPEED, new Rotation2d()),
                    new SwerveModuleState(SPEED, new Rotation2d()),
                    new SwerveModuleState(SPEED, new Rotation2d())
            });
        }).withTimeout(1.0));
    }

    /**
     * Retracts arm
     */
    public void retract() {
        cmd.addCommands(new MoveCollectiveArm(CollectiveMode.travelFS),
                        new MoveCollectiveArm(CollectiveMode.travelLockFS));
    }

}
