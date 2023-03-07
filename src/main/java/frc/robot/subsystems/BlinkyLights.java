// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.RobotSpecs.RobotNames;

public class BlinkyLights {
    //Some common colors
    static public Color8Bit BLACK = new Color8Bit(0, 0, 0);
    static public Color8Bit WHITE = new Color8Bit(255, 255, 255);
    static public Color8Bit RED = new Color8Bit(255, 0, 0);
    static public Color8Bit GREEN = new Color8Bit(0, 255, 0);
    static public Color8Bit BLUE = new Color8Bit(0, 0, 255);

    // State vars
    CANdle candle_l;
    CANdle candle_r;
    double brightness;
    Color8Bit currentColor;
    boolean amIReal;

    /** Creates a new BlinkyLights. */
    public BlinkyLights() {
        amIReal = ((RobotContainer.RC().robotSpecs.myRobotName == RobotNames.CompetitionBot2023) && !Robot.isSimulation());
        if (amIReal) {
            System.out.println("***I have blinkylights, I must be one of the cool robots.");
            candle_l = new CANdle(3);
            candle_r = new CANdle(4);
            candle_l.configAllSettings(new CANdleConfiguration());
            candle_r.configAllSettings(new CANdleConfiguration());

            BlinkyLightController.controlledLights = this;
            // dont really need nt ntcreate();
        } else {
            System.out.println("***I have no blinkylights :( ... setting up poser methods");
        }
    }

    public void setColor(Color8Bit color) {
        if (amIReal) {
            candle_l.setLEDs(color.red, color.green, color.blue);
            candle_r.setLEDs(color.red, color.green, color.blue);
        }
        currentColor = color;
    }

    public void setBlinking(boolean blink) {
        if (blink)
            setBlinking(currentColor);
        else
            stopBlinking();
    }

    public void setBlinking(Color8Bit color) {
        if (amIReal) {
            Animation animation = new StrobeAnimation(color.red, color.green, color.blue, 0, 0.5, 8);

            candle_l.animate(animation, 0);
            candle_r.animate(animation, 0);
        }
    }

    public void stopBlinking() {
        if (amIReal) {
            candle_l.clearAnimation(0);
            candle_r.clearAnimation(0);
        }
    }

    /*
     * Brightness on a scale from 0-1, with 1 being max brightness
     */

    public void setBrightness(double brightness) {
        if (amIReal) {
            candle_l.configBrightnessScalar(brightness);
            candle_r.configBrightnessScalar(brightness);
        }
    }

    public void setAllianceColors() {
        Alliance alliance = DriverStation.getAlliance();
        //System.out.println("***** Robot Alliance: " + DriverStation.getAlliance().name());
        switch (alliance) {
            case Blue:
                setColor(new Color8Bit(0, 0, 255));
                break;
            case Red:
                setColor(new Color8Bit(255, 0, 0));
                break;
            default:
                setColor(new Color8Bit(0, 255, 0));
                break;
        }
    }

    public static boolean differentColors(Color8Bit c1, Color8Bit c2) {
        return (c1.red != c2.red) ||
                (c1.green != c2.green) ||
                (c1.blue != c2.blue);
    }

    /**
     * NETWORK TABLES AHH
     * 
     * 
     * NetworkTable table = NetworkTableInstance.getDefault().getTable("Blinky
     * Lights");
     * 
     * NetworkTableEntry nt_brightness;
     * NetworkTableEntry nt_color_r;
     * NetworkTableEntry nt_color_g;
     * NetworkTableEntry nt_color_b;
     * 
     * public void ntcreate() {
     * nt_brightness = table.getEntry("Brightness");
     * nt_color_r = table.getEntry("Color-R");
     * nt_color_g = table.getEntry("Color-G");
     * nt_color_b = table.getEntry("Color-B");
     * }
     * 
     * public void ntUpdates() {
     * nt_brightness.setDouble(brightness);
     * nt_color_r.setInteger(currentColor.red);
     * nt_color_g.setInteger(currentColor.green);
     * nt_color_b.setInteger(currentColor.blue);
     * 
     * }
     * 
     */
    // commands and sub-systems
    public interface BlinkyLightUser {
        default void onRobotInit() {
        };

        default void onAutomousInit() {
        };

        default void onTeleopInit() {
        };

        default void onTestInit() {
        };

        // used in commands, Override to your preferences
        default Color8Bit colorProvider() {
            return WHITE;
        };

        // used in commands, Override to your preferences
        default boolean requestBlink() {
            return false;
        }

        default void enableLights() {
            if (this instanceof Command) {
                Command cmd = BlinkyLightController.colorMonitorCmdFactory(this);
                cmd.schedule();
                return;
            }
            BlinkyLightController.setCurrentUser(this);
        }

    }

    // static because there can be only one controller for now
    // tracks who is using the lights and runs the allianceColor during
    // disabled periodic.
    public static class BlinkyLightController {
        protected static BlinkyLights controlledLights = null;
        private static BlinkyLightUser defaultUser = new BlinkyLightUser() {
        };
        private static BlinkyLightUser currentUser = defaultUser;

        public static void setCurrentUser(BlinkyLightUser user) {
            currentUser = user;
        };

        public static void release() {
            currentUser = defaultUser;
            if (controlledLights != null){
                controlledLights.setAllianceColors();
            }
        };

        /* Static methods to intercept robot state changes */
        public static void onRobotInit() {
            currentUser.onRobotInit();
        };

        public static void onDisabledPeriodic() {
            // always do our alliance colors
            if (controlledLights != null)
                controlledLights.setAllianceColors();
        }

        public static void onAutomousdInit() {
            currentUser.onAutomousInit();
        };

        public static void onTeleopInit() {
            currentUser.onTeleopInit();
        };

        public static void onTestInit() {
            currentUser.onTestInit();
        };

        public static void onTestPeriodic() {
            currentUser.onTestInit();
        };

        /**
         * color monitoring command factory
         * 
         * @param user  object using the blinky lights, should be a Command & BlinkyLightUser
         * @return  cmd that will run and follow
         */
        protected static Command colorMonitorCmdFactory(BlinkyLightUser user) {
            if (!(user instanceof Command))
                return null;

            Command colorCmd = new Command() {
                protected Set<Subsystem> m_requirements = new HashSet<>();
                // Read from BlinkyLightUser
                Color8Bit currentColor;
                boolean blinkState;
                final BlinkyLightUser myUser = user;
                final Command myCmd = (Command) user;

                @Override
                public Set<Subsystem> getRequirements() {
                    return m_requirements;
                }

                @Override
                public void initialize() {
                    setCurrentUser(user);
                    if (controlledLights == null) return;

                    currentColor = myUser.colorProvider();
                    controlledLights.setColor(currentColor);
                    blinkState = myUser.requestBlink();
                    controlledLights.setBlinking(blinkState);
                }

                @Override
                public void execute() {
                    Color8Bit newColor = user.colorProvider();
                    if (controlledLights == null) return;

                    // avoid CAN bus traffic if color isn't changing
                    if (!currentColor.equals(newColor)) {
                        currentColor = newColor;
                        controlledLights.setColor(currentColor);
                    }
                    if (user.requestBlink() != blinkState) {
                        blinkState = user.requestBlink();
                        controlledLights.setBlinking(blinkState);
                    }
                }

                @Override
                public void end(boolean interrupted) {
                    release();
                }

                @Override
                public boolean isFinished() {
                    // run until my parent command is done
                    return myCmd.isScheduled();
                }
            };
            return colorCmd;
        }
    }// static class BlinkyLightController
}
