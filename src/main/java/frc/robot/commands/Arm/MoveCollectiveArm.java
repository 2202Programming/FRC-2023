// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSS;
import frc.robot.subsystems.Claw_Substyem;
import frc.robot.subsystems.Claw_Substyem.ClawTrackMode;
import frc.robot.subsystems.Elbow;
import frc.robot.util.VelocityControlled;

/*
 *   MoveCollectiveArm - moves the whole system to a desired location
 *   while making sure any claw flips are done in a safe area or ignored.
 * 
 *   Flip only uses the elbow angle and may slow the elbow speed down to
 *   ensure there is time to flip.
 * 
 *   todo: use the arm extension to change the SafeFlipPoint
 * 
 */

public class MoveCollectiveArm extends CommandBase {

  // hardware commanded
  ArmSS arm = RobotContainer.RC().armSS;
  Elbow elbow = RobotContainer.RC().elbow;
  Claw_Substyem claw = RobotContainer.RC().claw;

  // filled in at init based on where we are
  Positions start; // filled in init
  Positions target; // taken from CollectiveMode provided in command (constant positions)

  // elbow safe angle, flip be done by here going in, or starts here
  // when elbow is heading out
  double SafeFlipPoint = 55.0; // elbow at zero arm
  double SafeMinArm = 20.0; // safe to flip at or above zero elbow

  // flip controls - filled in in init()
  final VelocityControlled wrist;

  double flip_point; // where to flip when we get there
  boolean heading_out; // moving arm away
  double T_flip; // time it takes to flip claw
  double old_elbow_max_vel; // elbow speed
  double old_arm_max_vel;
  double new_elbow_max_vel; // elbow speed giving time to flip (heading in)
  double start_flip_pos;
  double flip_dist; // degrees (targ - start)
  boolean flip_requested;
  boolean flip_possible;
  boolean arm_flip_possible;
  boolean flip_started;
  Timer fliptimer = new Timer();

  boolean heading_out_arm;

  /**
   * State of colective Positions, either start or target.
   */
  public static class Positions {
    double armPos;
    double elbowPos;
    double wristPos;
    double armMaxVel; // <0.0 means use existing maxvel
    double elbowMaxVel; // <0.0 means use existing maxvel
    ClawTrackMode mode;

    public Positions(double armPos, double elbowPos, double wristPos, ClawTrackMode mode) {
      this(armPos, elbowPos, wristPos, mode, -1.0, -1.0);
    }

    public Positions(double armPos, double elbowPos, double wristPos, ClawTrackMode trackmode, double armVel,
        double elbowVel) {
      this.armPos = armPos;
      this.elbowPos = elbowPos;
      this.mode = trackmode;
      // use given angle for freemode, else use mode's value
      this.wristPos = (mode == ClawTrackMode.free) ? wristPos : mode.angle();
      this.armMaxVel = armVel;
      this.elbowMaxVel = elbowVel;
    }

    // some copy constructors to make it easy to change fixed values
    public Positions(Positions src) {
      this(src.armPos, src.elbowPos, src.wristPos, src.mode, src.armMaxVel, src.elbowMaxVel);
    }

    public Positions(CollectivePositions src) {
      this(src.pos_info);
    }
  }

  /** Creates a new MoveCollectiveArm. */
  public MoveCollectiveArm(CollectivePositions where_to) {
    this(where_to.pos_info);
  }

  /** Creates a new MoveCollectiveArm. */
  public MoveCollectiveArm(Positions pos_info) {
    target = pos_info;
    wrist = claw.getWrist();
    addRequirements(arm, elbow, claw);
  }

  // Called when the command is initially scheduled.
  // todo: move flip setting to a function to make more readable
  @Override
  public void initialize() {
    old_elbow_max_vel = elbow.getMaxVel();
    old_arm_max_vel = arm.getMaxVel();

    if (target.armMaxVel > 0.0)
      arm.setMaxVel(target.armMaxVel);
    if (target.elbowMaxVel > 0.0)
      elbow.setMaxVel(target.elbowMaxVel);

    fliptimer.reset();

    // capture where we are
    start = getStart();

    // figure out if and how filpping
    flip_started = false;
    flip_point = 10000.0; // never going to get here

    flip_dist = target.wristPos - start.wristPos;

    // crude - todo make this better
    flip_requested = Math.abs(start.wristPos - target.wristPos) > 90.0;
    flip_possible = false; // until proven otherwise
    arm_flip_possible = false;

    if (flip_requested) {
      //need to be in a tracking mode, Free could be pointing weird
      //so track nearest frontside/backside angle
      start.mode = claw.setNearestClawTrackMode();
    }

    // which way are we moving, in or out
    heading_out = (target.elbowPos - start.elbowPos) > 0.0;
    heading_out_arm = (target.armPos - start.armPos) > 0.0;

    // guess time to flip based on 180 and wrist max speed
    T_flip = (wrist.getMaxVel() > 1.0) ? 180.0 / wrist.getMaxVel() : 5000.0; // [sec]

    if (heading_out) {
      flip_point = Math.max(SafeFlipPoint, start.elbowPos);
      flip_possible = true;
    } else {
      // moving in, either there is time to flip or not
      double dist_to_safe = start.elbowPos - SafeFlipPoint; // [deg]
      // see if we can even flip, are we outside the flip point to start
      if (dist_to_safe >= 0.0) {
        flip_point = start.elbowPos; // start as soon as possible
        flip_possible = true;

        // calculate a new elbow speed that would give us time to flip
        new_elbow_max_vel = dist_to_safe / T_flip; // [deg/s]

        // keep slowest elbow speed, giving time to flip.
        new_elbow_max_vel = Math.min(new_elbow_max_vel, old_elbow_max_vel);

      } else {
        // not possible to flip given where we started
        flip_possible = false;
        if (flip_requested) {
          System.out.println("Flip requsested but not possible, heading in with start < safe angle");
        }
      }
    }

    // check ending arm for being in safe zone, independent of elbow
    if (target.armPos >= SafeMinArm) {
      arm_flip_possible = true;
      flip_possible = true;
    }

    if (flip_requested && flip_possible && !heading_out)
      elbow.setMaxVel(new_elbow_max_vel); // slow the elbow down

    // move towards our target, wrist done in exec
    arm.setSetpoint(target.armPos);
    elbow.setSetpoint(target.elbowPos);

    // move wrist if free mode and no flip
    if (target.mode == ClawTrackMode.free && flip_requested == false) {
      claw.setWristAngle(target.wristPos);
    } else if (!flip_requested) {
      // no flip, just take the track mode
      claw.setTrackElbowMode(target.mode);
    }
  }

  @Override
  public void execute() {
    // look for flip complete so we can speed up elbow again
    if (flip_started && fliptimer.hasElapsed(T_flip)) {
      // done flipping, restore elbow vel
      elbow.setMaxVel(old_elbow_max_vel);
    }

    if (flip_started)
      return;

    // Safe to transition sides?
    if (!(flip_requested && flip_possible))
      return;

    // watch for the flip point
    if (flip_possible &&
        (heading_out && elbow.getPosition() >= flip_point) ||
        (!heading_out && elbow.getPosition() <= flip_point)) {
      // safe to move the wrist/claw
      if (target.mode == ClawTrackMode.free) {
        claw.setWristAngle(target.wristPos); // will set to free
      } else
        claw.setTrackElbowMode(target.mode);

      flip_started = true;
      System.out.println("Flip started......");
      fliptimer.start();
      return;
    }

    if (arm_flip_possible && arm.getPosition() >= SafeMinArm) {
      claw.setTrackElbowMode(target.mode);
      flip_started = true;
      System.out.println("Arm limit Flip started......");
      fliptimer.start();
      return;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elbow.setMaxVel(old_elbow_max_vel);
    arm.setMaxVel(old_arm_max_vel);
    System.out.println("MCA done intr=" +interrupted);
  }

  /*
   * getStart() captures where we are now, ignoring velocities
   */
  Positions getStart() {
    return new Positions(
        arm.getPosition(),
        elbow.getPosition(),
        claw.getWristAngle(),
        claw.getTrackElbowMode());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.atSetpoint() &&
        elbow.atSetpoint() &&
        (wrist.atSetpoint() || !flip_possible || !flip_requested));
  }

}
