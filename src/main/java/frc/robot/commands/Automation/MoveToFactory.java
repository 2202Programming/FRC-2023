package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.HorizontalSubstationLane;
import frc.robot.commands.Arm.CollectivePositions;
import frc.robot.commands.Arm.MoveCollectiveArm;
import frc.robot.commands.auto.goToScoringPosition;
import frc.robot.commands.swerve.RotateTo;

public class MoveToFactory extends CommandBase {
    // controlllers
    CommandXboxController driver = RobotContainer.RC().dc.Driver();
    CommandXboxController operator = RobotContainer.RC().dc.Operator();

    // state vars
    HorizontalScoringLane horizLane;
    HorizontalSubstationLane subLane;
    CollectivePositions armPos;
    SequentialCommandGroup cmd;

    // constants
    private final double DEADZONE2 = 0.025; // [%^2] percent of deadzone squared

    /**
     * Creates a new MoveToFactory object.
     * 
     * @param triggers The triggers currently used so they're exposed in RobotContainer
     */
    public MoveToFactory(Trigger... triggers) {
       // do nothing
    }

    public void initialize() {
        cmd = new SequentialCommandGroup();
        // Station lane
        if (driver.leftBumper().getAsBoolean()) horizLane = HorizontalScoringLane.Left;
        else if (driver.rightBumper().getAsBoolean()) horizLane = HorizontalScoringLane.Right;
        else horizLane = HorizontalScoringLane.Center;

        // Substation lane
        if (operator.leftBumper().getAsBoolean()) subLane = HorizontalSubstationLane.Left;
        else if (operator.rightBumper().getAsBoolean()) subLane = HorizontalSubstationLane.Right;
        else subLane = HorizontalSubstationLane.Center;

        // Arm mid/high
        armPos = (operator.povUp().getAsBoolean()) ? CollectivePositions.placeConeHighFS : CollectivePositions.placeConeMidFS;

        Rotation2d scoreRotation = new Rotation2d((DriverStation.getAlliance().equals(Alliance.Blue)) ? 180.0 : 0.0);

        System.out.println("Ready to create SCG, Horizontal Scoring Lane: " + horizLane.toString() + 
                            ", Substation Lane: " + subLane.toString() + ", arm position: " + armPos.toString());

        cmd.addCommands(
            new PrintCommand("Starting SCG"),
            new goToScoringPosition(new PathConstraints(2.0, 3.0), horizLane, subLane),
            new PrintCommand("End autopath"),
            new RotateTo(scoreRotation),
            new PrintCommand("End rotation"),
            new MoveCollectiveArm(armPos),
            new PrintCommand("End SCG")
        );
        
        // bail when the driver says so
        cmd.until(() -> {
            boolean xStickStill = (Math.pow(driver.getLeftX(), 2) + Math.pow(driver.getLeftY(), 2)) > DEADZONE2; 
            boolean yStickStill = (Math.pow(driver.getRightX(), 2) + Math.pow(driver.getRightY(), 2)) > DEADZONE2;
            return !(xStickStill && yStickStill);
          }).schedule();
    }

    @Override
    public boolean isFinished() {return true;}
    
}
