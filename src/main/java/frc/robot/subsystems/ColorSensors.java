package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Mux;

public class ColorSensors extends SubsystemBase implements AutoCloseable {

    private class SensorData {
        RawColor color;
        double distance;
    }

    // sensor infos and arrays
    private final int[] sensorMuxPorts = {0, 1, 2};
    private final List<ColorSensorV3> colorSensors = new ArrayList<>();
    private final int numSensors = 3;
    private volatile SensorData[] colorSensorData = new SensorData[numSensors];

    // mux
    private final Mux mux = new Mux();

    // notifier
    private final Notifier notifier;

    /**
     * Constructs the ColorSensors subsystem
     */
    public ColorSensors() {
        setupColorMatching();

        colorSensorData[0] = new SensorData();
        colorSensorData[1] = new SensorData();
        colorSensorData[2] = new SensorData();

        for (int id : sensorMuxPorts) assert id < mux.availableBuses();

        for (int i = 0; i < sensorMuxPorts.length; i++) {
            mux.setEnabledBuses(sensorMuxPorts[i]);
            ColorSensorV3 sensor = new ColorSensorV3(Port.kOnboard);
            
            // Config 8 bit for least amount of load on can bus
            // 12ms so we get assuredely one measurement every frame (20ms)
            sensor.configureProximitySensor(ProximitySensorResolution.kProxRes8bit, ProximitySensorMeasurementRate.kProxRate12ms);

            // 13bit is the lowest res possible, again less load on CAN bus. We're only measuring purple and yellow, we should be ok
            // 25ms is the shortest, not quite once every frame, but close
            // gain of 9x appears to be the default, that's what we'll go with
            sensor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit, ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain9x);

            colorSensors.add(sensor);
        }

        // Recommended to run this in a thread due to multiple posts on ChiefDelphi on how the onboard I2C port can be shady and can be expensive
        notifier = new Notifier(() -> {
            for (int i = 0; i < colorSensorData.length; i++) {
                mux.setEnabledBuses(sensorMuxPorts[i]);
                ColorSensorV3 sensor = colorSensors.get(i);
                colorSensorData[i].color = sensor.getRawColor();
                colorSensorData[i].distance = sensor.getProximity();
            }
        });

        notifier.startPeriodic(Constants.DT); 
    }

    /**
     * Retrieves the color value of a sensor
     * 
     * @param sensorIndex The index of the sensor to retrieve color of
     * @return the RawColor (includes r, g, b, ir) as detected by a color sensor
     */
    public RawColor getColor(int sensorIndex) {
        return colorSensorData[sensorIndex].color;
    }

    /**
     * Retrieves the proximity value of a sensor
     * 
     * @param sensorIndex The index of the sensor to retrieve proximity of
     * @return The proximity, in millimeters
     */
    public double getProximity(int sensorIndex) {
        return colorSensorData[sensorIndex].distance;
    }

    @Override
    public void close() {
        notifier.stop();
        notifier.close();
    }

    /**
     * ===========================================================
     * End of sensor init and value gathering, beginning of usage
     * ===========================================================
     */

    private GamePiece currentGamePiece;
    private GamePiece prevFrameGamePiece;

    // Color matching state vars
    ColorMatch colorMatcher = new ColorMatch();

    public static Color CONE_YELLOW = new Color(236, 223, 76); // change to better values
    public static Color CUBE_PURPLE = new Color(63, 16, 184);

    final double CONFIDENCE_THRESHOLD = 0.90;

    public enum GamePiece {
        ConeFacingFront, ConeFacingBack, Cube, None
    }

    // Assuming sensor 1 is closest to the intake (front)
    // Assuming sensor 2 is in the middle
    // Assuming sensor 3 is closest to the battery (back)

    /**
     * Sets up colors for the color matching algo
     */
    private void setupColorMatching() {
        colorMatcher.addColorMatch(CONE_YELLOW); // change to better values
        colorMatcher.addColorMatch(CUBE_PURPLE); // change to better values
        colorMatcher.setConfidenceThreshold(CONFIDENCE_THRESHOLD); // 10% variation allowed, change?
    }

    @Override
    public void periodic() {
        prevFrameGamePiece = currentGamePiece;
        currentGamePiece = getGamePiece();

        if (currentGamePiece != prevFrameGamePiece) {
            // do stuff here upon game piece change
            switch (currentGamePiece) {
                case Cube:
                    
                    break;
                case ConeFacingFront:
                    
                    break;
                case ConeFacingBack:
                    
                    break;
                case None:
                    
                    break;
                default:
                    
                    break;
            }
        }
    }

    /**
     * Returns the current game piece as detected by the color sensor
     * 
     * @return The game piece
     */
    public GamePiece getCurrentGamePiece() {
        return currentGamePiece;
    }

    /**
     * Returns the game piece in the car wash via an algorithm
     * 
     * @return Game piece in the car wash
     */
    public GamePiece getGamePiece() {
        // Creates an results array and loops throug each sensor to try to match the color to the game piece colors
        ColorMatchResult[] results = new ColorMatchResult[numSensors];
        for (int i = 0; i < numSensors; i++) {
            RawColor rawColor = colorSensorData[i].color;
            results[i] = colorMatcher.matchColor(new Color(rawColor.red, rawColor.green, rawColor.blue));   
        }


        // Counts matches to each color and if 3 purple return cube, if >=2 yellow run through cone detection, otherwise return 0
        int numPurple = 0;
        int numYellow = 0;
        for (ColorMatchResult result : results) {
            if (result.color == null) continue;

            if (result.color.equals(CUBE_PURPLE)) numPurple++;
            else if (result.color.equals(CONE_YELLOW)) numYellow++;
        }
        if (numPurple == 3) return GamePiece.Cube;
        else if (numYellow >= 2) return getConeOrientation(results);
        else return GamePiece.None;
        
    }

    /**
     * 
     */
    public GamePiece getConeOrientation(ColorMatchResult[] results) {
        // if no color is detected from the front color sensor then skinny end probably facing front
        if (results[0] == null) return GamePiece.ConeFacingFront;
        // if no color is detected from the back color sensor then skinny end probably facinb back
        else if (results[2] == null) return GamePiece.ConeFacingBack;

        // if the distance from sensor to object is greater in the front sensor than the second skinny end probably facing front, else facing back
        return ((colorSensorData[0].distance - colorSensorData[2].distance) > 0) ? GamePiece.ConeFacingFront : GamePiece.ConeFacingBack;
    }
}