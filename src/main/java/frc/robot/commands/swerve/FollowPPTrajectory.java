package frc.robot.commands.swerve;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public class FollowPPTrajectory extends CommandBase {

  // Since a PathPlannerTrajectory extends the WPILib Trajectory, it can be referenced as one
  // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
  PathPlannerTrajectory path;
  Command runcommand;
  boolean useOdo;
  SwerveDrivetrain sdt;
  Sensors_Subsystem sensors;

  public FollowPPTrajectory(PathPlannerTrajectory path, boolean useOdo) {
    sdt = RobotContainer.RC().drivetrain;
    sensors = RobotContainer.RC().sensors;
    this.path = path;
        // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sdt, sensors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // grab the trajectory determined by the AutoPath
    if (path != null) {
      // Reset odometry to the starting pose of the trajectory.
      //IMPORTANT: Pathplanner heading of first point is the assumed starting heading of your bot
      //If first point has a non-zero heading, the gryo will get offset with this setPose
      sdt.setPose(path.getInitialPose());
    }
  }

    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command runcommand = getPathCommand();
    runcommand.schedule();
  }

  public static PathPlannerTrajectory getPPPath(String pathName) {
    return PathPlanner.loadPath(pathName, 1, 1); //last two parameters are max velocity and max accelleration
  }

  /*
   * @param boolean useOdo when true, use odometry's current robot position.
   *                       when false, use the path's starting point as the true robot position.
   *         First auto path (or when going from a known position), useOdo should be false, otherwise true
   */
  public Command getPathCommand() {

    if (path == null) {
      return new InstantCommand();  // no path selected
    }
    
    //TODO: wtf does htis do
    //sdt.m_field.getObject(pathname).setTrajectory(path);

    // get initial state from the trajectory
    PathPlannerState initialState = path.getInitialState();
    Pose2d startingPose = new Pose2d(initialState.poseMeters.getTranslation(), initialState.holonomicRotation);
    //TODO: take PID values from Constants or construction?
    PIDController xController = new PIDController(1.0, 0.0, 0.0, Constants.DT);   // [m]
    PIDController yController = new PIDController(1.0, 0.0, 0.0, Constants.DT);   // [m]
    PIDController thetaController = new PIDController(0.08, 0, 0, Constants.DT);     // [rad]
      //Units are radians for thetaController; PPSwerveController is using radians internally.
      thetaController.enableContinuousInput(-Math.PI, Math.PI); //prevent piroutte paths over continuity

      PPSwerveControllerCommand swerveControllerCommand =
      new PPSwerveControllerCommand(
          path,
          sdt::getPose, // Functional interface to feed supplier
          sdt.getKinematics(),
          // Position controllers 
          xController,
          yController,
          thetaController,
          sdt::drive,
          sdt
      );


    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
      new InstantCommand(()-> {
        if (!useOdo) // useOdo is false, starting pose is position/heading as defined in path. Otherwise go from where we are
          sensors.setAutoStartPose(startingPose);
      }),
      new PrintCommand("***Running Path"),
      swerveControllerCommand,
      new InstantCommand(sdt::stop),
      new PrintCommand("***Done Running Path")
      );
  }

  public static PathPlannerTrajectory pathFactoryAuto(PathConstraints constraints, String pathName) {
    return PathPlanner.loadPath(pathName, constraints); 
  }

  public static PathPlannerTrajectory pathFactoryTele(PathConstraints constraints, Pose2d finalPoint) {
    SwerveDrivetrain sdt = RobotContainer.RC().drivetrain;
    //TODO: actually figure out max vel / accel and move to constants
    return PathPlanner.generatePath(constraints, 
                          new PathPoint(sdt.getPose().getTranslation(), finalPoint.getRotation(), sdt.getPose().getRotation()), 
                          new PathPoint(finalPoint.getTranslation(), finalPoint.getRotation(), finalPoint.getRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
