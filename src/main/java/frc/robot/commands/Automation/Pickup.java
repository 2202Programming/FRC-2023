package frc.robot.commands.Automation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.swerve.RotateTo;
import frc.robot.commands.swerve.VelocityMove;
import frc.robot.subsystems.ColorSensors.GamePiece;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;
import frc.robot.util.FlexibleSCG;

public class Pickup extends FlexibleSCG {
    // substation enum
    public enum Substation {
        Left, Right
    }

    // dc and related vars
    HID_Xbox_Subsystem dc = RobotContainer.RC().dc;

    // sdt and related vars
    SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
    //BACKUP_SPEED is relative to the arm, which is on the -X direction
    final double BACKUP_SPEED = 0.5; // [m/s] backup speed + should move opp the arm
    final double BACKUP_TIME = 1.0; // [s] time to backup for

    // state vars
    final Substation substation;
    final GamePiece gamePiece;

    public Pickup(Substation substation, GamePiece gamePiece) {
        this.substation = substation;
        this.gamePiece = gamePiece;
    }

    @Override
    public void doFirstOnInit() {
        this.addCommands(new PrintCommand("New SCG created"));
        //move();      // must move to close, but clear spot for arm extend
        extend();  
        getPiece();
        //backup();
        //retract();
        this.addCommands(new PrintCommand("SCG should be ending"));
    }

    @Override
    public boolean isFinishedCondition() {
        // bail when the driver says so
        return dc.rightStickMotionDriver();
    }

    /**
     * Moves to the requisite location and rotates afterwards to ensure rotation is
     * on point
     */
    public void move() {
        this.addCommands(
               // WIP ---> new goToPickupPosition(new PathConstraints(3.0, 3.0), substation),
                new RotateTo(new Rotation2d(DriverStation.getAlliance().equals(Alliance.Blue) ? 0 : 180)));
    }

    /**
     * Extends arm out to pickup position
     */
    public void extend() {
        this.addCommands(
            new PrintCommand("Before pickup shelf"),
            new MoveCollectiveArm(CollectivePositions.pickupShelfFS),
            new PrintCommand("After pickup shelf")
        );
    }

    /**
     * Picks up piece w/ claw
     * Slowly drives in arm-dir to get piece.  Will get a lightgate break or driver will 
     * have to interrupt the sequence.
     */
    public void getPiece() {
        this.addCommands(
            new PrintCommand("Before PickupDrive"),
            new PickupDrive(0.5 , gamePiece),
            new PrintCommand("After PickupDrive")
            );  
    }

    /**
     * Backs up to safe distance to retract arm
     */
    public void backup() {
        this.addCommands(new VelocityMove(BACKUP_SPEED, 0.0, BACKUP_TIME));
    
    }

    /**
     * Retracts arm
     */
    public void retract() {
        this.addCommands(new MoveCollectiveArm(CollectivePositions.travelFS),
                        new MoveCollectiveArm(CollectivePositions.travelLockFS));
    }
}
