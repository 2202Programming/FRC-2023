package frc.robot.commands.Automation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Arm.MoveCollectiveArm.CollectiveMode;
import frc.robot.commands.swerve.RotateTo;
import frc.robot.commands.swerve.VelocityMove;
import frc.robot.subsystems.ColorSensors.GamePiece;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;

public class Pickup extends CommandBase {
    // substation enum
    public enum Substation {
        Left, Right
    }

    // dc and related vars
    HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
    final double DEADZONE2 = (0.025);  //put sq here

    // sdt and related vars
    SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
    //BACKUP_SPEED is relative to the arm, which is on the -X direction
    final double BACKUP_SPEED = 0.5; // [m/s] backup speed + should move opp the arm
    final double BACKUP_TIME = 1.0; // [s] time to backup for

    // state vars
    final Substation substation;
    final GamePiece gamePiece;
    SequentialCommandGroup cmd;

    public Pickup(Substation substation, GamePiece gamePiece) {
        this.substation = substation;
        this.gamePiece = gamePiece;
    }

    @Override
    public void initialize() {
        cmd = new SequentialCommandGroup();
        move();      // must move to close, but clear spot for arm extend
        extend();  
        getPiece();
        backup();
        retract();

        // bail when the driver says so
        cmd.until(() -> {
            boolean xStickStill = (Math.pow(dc.Driver().getLeftX(), 2) + Math.pow(dc.Driver().getLeftY(), 2)) > DEADZONE2; 
            boolean yStickStill = (Math.pow(dc.Driver().getRightX(), 2) + Math.pow(dc.Driver().getRightY(), 2)) > DEADZONE2;
            return !(xStickStill && yStickStill);
          }).schedule();
    }

    /**
     * Moves to the requisite location and rotates afterwards to ensure rotation is
     * on point
     */
    public void move() {
        cmd.addCommands(
               // WIP ---> new goToPickupPosition(new PathConstraints(3.0, 3.0), substation),
                new RotateTo(new Rotation2d(DriverStation.getAlliance().equals(Alliance.Blue) ? 0 : 180)));
    }

    /**
     * Extends arm out to pickup position
     */
    public void extend() {
        cmd.addCommands(new MoveCollectiveArm(CollectiveMode.pickupShelfFS));
    }

    /**
     * Picks up piece w/ claw
     * Slowly drives in arm-dir to get piece.  Will get a lightgate break or driver will 
     * have to interrupt the sequence.
     */
    public void getPiece() {
        cmd.addCommands(new PickupDrive(0.5 , gamePiece));  
    }

    /**
     * Backs up to safe distance to retract arm
     */
    public void backup() {
        cmd.addCommands(new VelocityMove(BACKUP_SPEED, 0.0, BACKUP_TIME));
    
    }

    /**
     * Retracts arm
     */
    public void retract() {
        cmd.addCommands(new MoveCollectiveArm(CollectiveMode.travelFS),
                        new MoveCollectiveArm(CollectiveMode.travelLockFS));
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
