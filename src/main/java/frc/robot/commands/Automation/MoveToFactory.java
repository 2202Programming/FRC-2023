package frc.robot.commands.Automation;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.Constants.HorizontalScoringLane;
import frc.robot.Constants.HorizontalSubstationLane;
import frc.robot.commands.auto.goToScoringPosition;

public class MoveToFactory extends CommandBase {
    // controlllers
    CommandXboxController driver = RobotContainer.RC().dc.Driver();
    CommandXboxController operator = RobotContainer.RC().dc.Operator();

    // state vars
    HorizontalScoringLane horizLane;
    HorizontalSubstationLane subLane;
    Command cmd;

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
        // Station lane
        if (driver.leftBumper().getAsBoolean()) horizLane = HorizontalScoringLane.Left;
        else if (driver.rightBumper().getAsBoolean()) horizLane = HorizontalScoringLane.Right;
        else horizLane = HorizontalScoringLane.Center;

        // Substation lane
        if (operator.leftBumper().getAsBoolean()) subLane = HorizontalSubstationLane.Left;
        else if (operator.rightBumper().getAsBoolean()) subLane = HorizontalSubstationLane.Right;
        else subLane = HorizontalSubstationLane.Center;

        cmd = new goToScoringPosition(new PathConstraints(2.0, 3.0), horizLane, subLane);
        
        // bail when the driver says so
        cmd.until(() -> {
            boolean xStickStill = (Math.pow(driver.getLeftX(), 2) + Math.pow(driver.getLeftY(), 2)) > DEADZONE2; 
            boolean yStickStill = (Math.pow(driver.getRightX(), 2) + Math.pow(driver.getRightY(), 2)) > DEADZONE2;
            return !(xStickStill && yStickStill);
          }).schedule();
    }
    
}
