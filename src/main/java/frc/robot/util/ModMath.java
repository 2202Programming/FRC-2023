// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class ModMath {

/**
   * Performs  (a1 - a2) and returns proper delta accounting for a discontinuity 
   * accross the given mod. This assumes the rate of change is less than the 
   * mod value.  
   * 
   *  Example Mod = 180
   *   a1 = 175
   *   a0 = -179
   *       (a1 - a0) = 354 (simple diff)
   *        354 - 360 = -6 (correct rotated 6 CW negative)
   * 
   * @param a1  current sample
   * @param a0  prev sample
   * @param mod  modulo discontinuity (180 for +/180) 
   * @return (a1 - a0) corrected for mod discontinuity
   */
  public static double delta_mod(double a1, double a0, double mod ) {
    double delta = a1 - a0;
    double mod_delta = fmod(delta, mod);
    return fixDiscontinuity(mod_delta, mod/2.0);
  }

  public static double delta360(double a1, double a0) {
    double delta = a1 - a0;
    return fmod360(delta);
  }


  /**
   * fmod360(x) - fixed mod intuitive implementation
   *              range fixed to +/- 180
   * @param x  
   * @return  
   */
  public static double fmod360_2(double x)  {
    final double MOD = 360.0;
    if (x > 180.0) {
      while ( x > 180.0) {
        x = x - MOD;
      }
    }
    if (x < -180.0) {
      while (x < -180.0) {
        x = x + MOD;
      }
    }
    return x;
  }

  /**
   * fmod360() - returns the fixed angle mod 360, range fixed to +/- 180 .
   * @param x
   * @return
   */
  public static double fmod360(double x) {
    final double MOD = 360.0;
    double r = Math.IEEEremainder(x, MOD);
    return fixDiscontinuity(r, MOD/2.0);
  }

  public static double fmod(double x, double mod) {
    return Math.IEEEremainder(x, mod);
  }

  public static double fixDiscontinuity(double x, double discontinuity) {
    if (x > discontinuity) {
      x -= 2*discontinuity; 
    }
    else if ( x < -discontinuity) {
      x += 2*discontinuity;
    }
    return x;
  }


}
