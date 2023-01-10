// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.NTStrings;
import frc.robot.RobotContainer;
import frc.robot.commands.Shoot.SolutionProvider;
import frc.robot.subsystems.Limelight_Subsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.shooter.Shooter_Subsystem;

public class DriveControllerDrivetrain extends CommandBase implements SolutionProvider {

  public enum DriveModes {
    robotCentric("Robot Centric"),
    fieldCentric("Field Centric"),
    hubCentric("Hub Centric"),
    intakeCentric("Intake Centric");
    private String name;
    private DriveModes(String name) {
      this.name = name;
    }
    public String toString() {
      return name;
    }
  }

  SwerveDrivetrain drivetrain;
  DriverControls dc;
  Shooter_Subsystem shooter;
  Limelight_Subsystem limelight;
  RobotCentricDrive m_robotCentricDrive;
  FieldCentricDrive m_fieldCentricDrive;
  HubCentricDrive m_hubCentricDrive;
  IntakeCentricDrive m_intakeCentricDrive;

  Command currentCmd;
  DriveModes requestedDriveMode = DriveModes.fieldCentric;
  DriveModes currentDriveMode = DriveModes.fieldCentric;
  DriveModes lastDriveMode = DriveModes.fieldCentric;
  boolean currentlyShooting = false;
  boolean shootingRequested = false;
  boolean hasSolution = false;

  NetworkTable table;
  NetworkTable shooterTable;
  NetworkTable positionTable;
  NetworkTableEntry driveMode;
  public final String NT_Name = "DC"; 
  public final String NT_ShooterName = "Shooter"; 
  
  int log_counter = 0;

  public DriveControllerDrivetrain()  {
    this.drivetrain = RobotContainer.RC().drivetrain;
    this.dc = RobotContainer.RC().driverControls;
    this.limelight = RobotContainer.RC().limelight;

    m_robotCentricDrive = new RobotCentricDrive(drivetrain, dc);
    m_fieldCentricDrive = new FieldCentricDrive(drivetrain, dc);
    m_hubCentricDrive = new HubCentricDrive(drivetrain, dc, limelight);
    m_intakeCentricDrive = new IntakeCentricDrive(drivetrain, dc);

    table = NetworkTableInstance.getDefault().getTable(NT_Name);
    positionTable = NetworkTableInstance.getDefault().getTable(NTStrings.NT_Name_Position);
    driveMode = table.getEntry("/DriveController/driveMode");
    //NThasSolution = shooterTable.getEntry("/DriveController/HasSolution");
  }

  @Override
  public void initialize() {
    currentCmd = m_fieldCentricDrive;
    CommandScheduler.getInstance().schedule(currentCmd); // start default drive mode
  }

  @Override
  public void execute() {
    checkDropout();
    checkRequests();
    updateNT();
  }

  /**
   * isOnTarget provides feedback to shoot command being run.
   */
  @Override
  public boolean isOnTarget(){
    // free shoot mode - just return true 
    if (currentDriveMode == DriveModes.hubCentric)
      return m_hubCentricDrive.isReady();
    return true;   //all other modes driver is the targeting solution
  }

  void checkDropout(){
    if ((Math.abs(dc.getXYRotation())>0.1) && (currentDriveMode==DriveModes.intakeCentric)){
      //driver is trying to rotate while in intakeCentric, drop out of intake mode
      requestedDriveMode = DriveModes.fieldCentric;
    }
  }

  void checkRequests(){
    if (requestedDriveMode != currentDriveMode){ //new drive mode requested
      lastDriveMode = currentDriveMode;
      currentDriveMode = requestedDriveMode;
      currentCmd.end(true); //stop prior command
      switch (currentDriveMode){
        case robotCentric:
          currentCmd = m_robotCentricDrive;
          break;
  
        case fieldCentric:
          currentCmd = m_fieldCentricDrive;
          break;    

        case hubCentric:
          currentCmd = m_hubCentricDrive;
          break;      
        
        case intakeCentric:
          currentCmd = m_intakeCentricDrive;
          break;
      }
      CommandScheduler.getInstance().schedule(currentCmd);
    }
  }

  public void cycleDriveMode() {
    //Current use case is only to allow toggling between field and intake centric
    //Make sure if in hubcentric (trigger held) that toggling drops back to default fieldcentric
    switch (currentDriveMode) {
      case robotCentric:
      case hubCentric:
      case fieldCentric:
        requestedDriveMode = DriveModes.intakeCentric;
        break;

      case intakeCentric:
        requestedDriveMode = DriveModes.fieldCentric;
        break;
    }
  }

  public void setRobotCentric() {
    requestedDriveMode = DriveModes.robotCentric;
  }

  public void setFieldCentric() {
    requestedDriveMode = DriveModes.fieldCentric;
  }

  public void turnOnShootingMode(){
    shootingRequested = true;
    limelight.enableLED();
  }

  public void turnOffShootingMode(){
    shootingRequested = false;
  }

  @Override
  public void end(boolean interrupted) {
    currentCmd.end(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  void updateNT() {
    log_counter++;
    if ((log_counter%20)==0) {
      // update network tables
      driveMode.setString(currentDriveMode.toString());
    }
  }

  //for future use in autobalance
  void checkTip(){
    double kOffBalanceAngleThresholdDegrees = 5;
    double pitchAngleDegrees = RobotContainer.RC().sensors.getPitch();    
    double rollAngleDegrees = RobotContainer.RC().sensors.getRoll();
    if (Math.abs(pitchAngleDegrees)>kOffBalanceAngleThresholdDegrees){
      //System.out.println("***PITCH WARNING: Pitch angle:"+pitchAngleDegrees);
    }
    if (Math.abs(rollAngleDegrees)>kOffBalanceAngleThresholdDegrees){
      //System.out.println("***ROLL WARNING: Roll Angle:"+rollAngleDegrees);
    }
  }

}
