package frc.robot.commands.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class FollowPPTrajectory extends CommandBase {
    private final SwerveDrivetrain m_robotDrive;

  // Since a PathPlannerTrajectory extends the WPILib Trajectory, it can be referenced as one
  // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
  PathPlannerTrajectory path;
  Command runcommand;

  public FollowPPTrajectory(SwerveDrivetrain drive, PathPlannerTrajectory path) {
    m_robotDrive = drive;
    this.path = path;
        // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // grab the trajectory determined by the AutoPath
    if (path != null) {
      // Reset odometry to the starting pose of the trajectory.
      //IMPORTANT: Pathplanner heading of first point is the assumed starting heading of your bot
      //If first point has a non-zero heading, the gryo will get offset with this setPose
      m_robotDrive.setPose(path.getInitialPose());
    }
  }

    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command runcommand = getPathCommand();
    runcommand.schedule();
  }

  public Command getPathCommand() {
    if (path == null) {
      System.out.println("No path");
      return new InstantCommand();  // no path selected
    }
      
    // get initial state from the trajectory
    PathPlannerState initialState = path.getInitialState();
    Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);

      PIDController xController = new PIDController(4.0, 0.0, 0.0, Constants.DT);   // [m]
      PIDController yController = new PIDController(4.0, 0.0, 0.0, Constants.DT);   // [m]
      PIDController thetaController = new PIDController(4, 0, 0, Constants.DT);     // [rad]
       /*
      DPL - 10/26/22 profiled pid not allowed in PPSwerveController..
      ProfiledPIDController thetaController = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(3, 3));
      */

      //Units are radians for thetaController; PPSwerveController is using radians internally.
      thetaController.enableContinuousInput(-Math.PI, Math.PI); //prevent piroutte paths over continuity

      PPSwerveControllerCommand swerveControllerCommand =
      new PPSwerveControllerCommand(
          path,
          m_robotDrive::getPose, // Functional interface to feed supplier
          m_robotDrive.getKinematics(),
          // Position controllers 
          xController,
          yController,
          thetaController,
          m_robotDrive::drive,
          (Subsystem)m_robotDrive
      );
        
    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
      new InstantCommand(()-> {
        m_robotDrive.setPose(startingPose);
      }),
      new PrintCommand("***Running Path "),
      swerveControllerCommand,
      new InstantCommand(m_robotDrive::stop),
      new PrintCommand("***Done Running Path ")
      );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
