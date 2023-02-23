package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public interface NetworkTableUtil {
    /**
     * Returns whether the network table entries for this class are necessary for comp
     * 
     * @return whether the NTEs for this class are necessary for comps
     */
    default boolean necessaryForCompetition() { return false; }

    /**
     * Create NetworkTables here
     */
    void ntcreate();

    /**
     * Update NetworkTables here
     */
    void ntupdate();

    /**
     * Call this in the constructor
     */
    default void ntconstructor() { if (!necessaryForCompetition() && DriverStation.isFMSAttached()) return; ntcreate(); }

    /**
     * Call this in the periodic / execute; runs every period (default 20ms)
     */
    default void ntperiod() { if (!necessaryForCompetition() && DriverStation.isFMSAttached()) return; ntupdate(); }
}
