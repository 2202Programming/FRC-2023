// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Automation.Pickup.Substation;
import frc.robot.commands.auto.goToPickupPosition;
import frc.robot.subsystems.ColorSensors;
import frc.robot.subsystems.hid.HID_Xbox_Subsystem;


public class PickFromShelf extends Command {

  // SSs
  private HID_Xbox_Subsystem dc = RobotContainer.RC().dc;
  public ColorSensors colorSensors = RobotContainer.RC().colorSensors;

  // deadzone
  final double DEADZONE = 0; // [percent]

  // state vars
  private Substation moveDirection;
  
  // the cmd
  private SequentialCommandGroup cmd = new SequentialCommandGroup();

  /**
   * This class acts as a command factory.
   * 
   * Constructs and schedules a new Place command. Encompasses sdt movement, arm extension / retraction.
   * Assumes piece is already in claw (this should be done upon picking up).
   * 
   * @param moveDirection
   */
  public PickFromShelf(Substation moveDirection) {
    this.moveDirection = moveDirection;

    addRequirements(dc, colorSensors);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // 1. Move to pickup position
    move();

    // 2. Grab the piece
    pickPiece();
    
    //should be same as the code commented out below - TODO remove after testing
    cmd.until(dc::rightStickMotionDriver).schedule();

    // Stealing this from the place command. Probably overkill, but I thought it would be easier on the drive team if picking and placing were about the same as possible.
    //cmd.until(() -> {
    //  boolean xStickStill = (Math.sqrt(Math.pow(dc.Driver().getLeftX(), 2) + Math.pow(dc.Driver().getLeftY(), 2)) > DEADZONE); 
    //  boolean yStickStill = (Math.sqrt(Math.pow(dc.Driver().getRightX(), 2) + Math.pow(dc.Driver().getRightY(), 2)) > DEADZONE);
    //  return (xStickStill && yStickStill);
    //}).schedule();
  }

  /**
   * Adds sdt movement to shelf pickup location.
   */
  private void move() {
    // 1. move to general vicinity
    cmd.addCommands(new goToPickupPosition(new PathConstraints(2,3), moveDirection));

    // 2. TODO check that there is room to extend the arm without putting the arm through the wall
    // X positions in goToPickupPosition were estimates. They need to be tested on the field as does the arm motion
    // i.e. should the wrist/elbow get rotated up for the arm to extend and rotate back down once its extended to clear the shelf,
    // or should the robot start back a little bit, extend the arm and then move forward until the light gate breaks and then back up
    // with the piece to retract the arm? I'm thinking the later makes more sense, or a combination of the two for the retract the rotates
    // the wrist up so the claw and cone clear the shelf. Might even be better for driving to flip the cone back into the bot
  }

  /**
   * Adds commands necessary for grabbing a piece from the shelf.
   */
  private void pickPiece() {
    cmd.addCommands(
      new takeConeFromShelf()
    );
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    // do nothing
  }
  
  @Override
  public void end(boolean interrupted) {
    // do nothing
  }
  
  @Override
  public boolean isFinished() {
    // it's movement cmd path factory, should be done after init / upon first time execute is called
    return true;
  }
}