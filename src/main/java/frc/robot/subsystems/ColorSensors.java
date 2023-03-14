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

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DigitalIO;
import frc.robot.commands.utility.WatcherCmd;
import frc.robot.util.Mux;

public class ColorSensors extends SubsystemBase implements AutoCloseable {
    // SSs
    private Intake intake = RobotContainer.RC().intake;

    // a "struct"
    private class SensorData {
        Color color;
        double ir;
        double distance;
    }

    // sensor infos
    private final int[] sensorMuxPorts = { 0, 1, 2 };
    private final List<ColorSensorV3> colorSensors = new ArrayList<>();
    private final int numSensors = 3;
    private final DigitalInput lightgate = new DigitalInput(DigitalIO.ColorSensorGate);

    // sensor results
    private volatile SensorData[] colorSensorData = new SensorData[numSensors];
    public ColorMatchResult[] results; // TODO switch back to private post-St Louis
    private int numYellow;
    private int numPurple;

    // mux
    private final Mux mux = new Mux();

    // notifier, which contains the updating colorSensorData periodic
    private final Notifier notifier;

    /**
     * Constructs the ColorSensors subsystem
     */
    public ColorSensors() {
        setupColorMatching();

        colorSensorData[0] = new SensorData();
        colorSensorData[1] = new SensorData();
        colorSensorData[2] = new SensorData();

        for (int id : sensorMuxPorts)
            assert id < mux.availableBuses();

        for (int i = 0; i < sensorMuxPorts.length; i++) {
            mux.setEnabledBuses(sensorMuxPorts[i]);
            ColorSensorV3 sensor = new ColorSensorV3(Port.kOnboard);

            // Config 8 bit for least amount of load on can bus
            // 12ms so we get assuredely one measurement every frame (20ms)
            sensor.configureProximitySensor(ProximitySensorResolution.kProxRes8bit,
                    ProximitySensorMeasurementRate.kProxRate12ms);

            // 13bit is the lowest res possible, again less load on CAN bus. We're only
            // measuring purple and yellow, we should be ok
            // 25ms is the shortest, not quite once every frame, but close
            // gain of 9x appears to be the default, that's what we'll go with
            sensor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit,
                    ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain9x);

            colorSensors.add(sensor);
        }

        // Recommended to run this in a thread due to multiple posts on ChiefDelphi on
        // how the onboard I2C port can be shady and can be expensive
        notifier = new Notifier(() -> {
            for (int i = 0; i < colorSensorData.length; i++) {
                mux.setEnabledBuses(sensorMuxPorts[i]);
                ColorSensorV3 sensor = colorSensors.get(i);
                colorSensorData[i].color = sensor.getColor();
                colorSensorData[i].distance = sensor.getProximity();
                colorSensorData[i].ir = sensor.getRawColor().ir;
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
    public Color getColor(int sensorIndex) {
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

    // Current and previous frame game piece
    private GamePiece currentGamePiece = GamePiece.None;
    private GamePiece prevFrameGamePiece = currentGamePiece;
    private int sameGamePiece = 0;

    // Outtake lightgate tripped
    private int outtakeLightgateTrippedFrames = 0;
    private int outtakeLightgateTrueFrames = 0;
    private boolean updateOuttakeNumbers = false;
    private final int OUTTAKE_THRESHOLD = 3;

    // object detection via lightgate
    private int intakeLightgateTrippedFrames = 0;
    private boolean updateIntakeNumbers = false;
    final int INTAKE_THRESHOLD = 5; // [frames]
    final int RETRACT_TIMER_THRESHOLD = 20; // [frames]

    // Color matching state vars
    ColorMatch colorMatcher = new ColorMatch();

    // Colors to be matched
    public static Color CONE_YELLOW = new Color(0.35, 0.55, 0.0); // TODO change to better values
    public static Color CUBE_PURPLE = new Color(0.3, 0.2, 0.5);

    /*
     * Confidence threshold, which is the Euclidean vector distance between the
     * actual color vector and the expected color vector,
     * which is 1 - (euclidean distance between actual, matched color)
     */
    final double CONFIDENCE_THRESHOLD = 0.85;

    // enum of possible game piece orientations to return
    public enum GamePiece {
        ConeFacingFront, ConeFacingBack, Cube, None
    }

    // Assuming sensor 0 is closest to the intake (front)
    // Assuming sensor 1 is in the middle
    // Assuming sensor 2 is closest to the battery (back)

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
        if (updateIntakeNumbers) {
            intakeLightgateTrippedFrames = (!lightgate.get()) ? intakeLightgateTrippedFrames + 1 : 0;
        }

        if (updateOuttakeNumbers) {
            outtakeLightgateTrippedFrames = (!lightgate.get()) ? outtakeLightgateTrippedFrames + 1 : 0;
            outtakeLightgateTrueFrames = ((outtakeLightgateTrippedFrames >= OUTTAKE_THRESHOLD) && (lightgate.get())) ? outtakeLightgateTrueFrames + 1 : 0;
        }
        // if the lightgate was broken for at least 1/4sec continuously and intake raised for at least 1sec then see if there's an object
        if ((intakeLightgateTrippedFrames >= INTAKE_THRESHOLD) && (intake.getRetractTimer() >= RETRACT_TIMER_THRESHOLD)) {
            prevFrameGamePiece = currentGamePiece;
            currentGamePiece = getCurrentGamePiece();

            sameGamePiece = (prevFrameGamePiece.equals(currentGamePiece)) ? sameGamePiece + 1 : 0;

            // confidence in current game piece
            if (sameGamePiece >= 3) {
                // reset the frames a *new, undetected* object has been detected to 0 now that the just-detected object has been recognized
                resetIntakeFrames();
            }
        }
    }

    /**
     * Returns the current game piece as detected by the color sensors
     * 
     * @return The game piece
     */
    public GamePiece getCurrentGamePiece() {
        if (sameGamePiece >= 3)
            return currentGamePiece;
        return GamePiece.None;
    }

    // Every time we place or eject a game peice, Call this method. This will clear
    // lines 172-179, and allow us to detect peices again
    public void clearCurrentGamePiece() {
        currentGamePiece = GamePiece.None;
    }

    /**
     * Returns the game piece in the car wash via an algorithm
     * 
     * @return Game piece in the car wash
     */
    public GamePiece getGamePiece() {
        // loops throug each sensor to try to match the color to the game piece colors
        results = new ColorMatchResult[numSensors];
        for (int i = 0; i < numSensors; i++) {
            Color color = colorSensorData[i].color;
            results[i] = colorMatcher.matchColor(color);
        }

        numPurple = 0;
        numYellow = 0;
        // Counts matches to each color
        for (int i = 0; i < numSensors; i++) {
            if (colorSensorData[i].distance < 7.0)
                results[i] = null;
            if (results[i] == null)
                continue;
            if (results[i].color.equals(CUBE_PURPLE))
                numPurple++;
            else if (results[i].color.equals(CONE_YELLOW))
                numYellow++;
        }

        /**
         * If all 3 are purple it's probably a cube
         * If all 3 are yellow it's probably a cone facing backwards
         * If 1 or 2 are yellow it's probably a cone facing backwords
         * Otherwise it's probably nothing
         */
        if (numPurple == 3)
            return GamePiece.Cube;
        else if (numYellow == 3)
            return GamePiece.ConeFacingBack;
        else if (numYellow >= 1)
            return GamePiece.ConeFacingFront;
        else
            return GamePiece.None;
    }

    /**
     * Gets the orientation of the cone based on the end color sensors, in the
     * following order:
     * (a) if a matching color is found; (b) the sign of the difference in proximity
     * between the two
     * 
     * @return the orientation of the cone as a GamePiece enum. Will only ever
     *         return GamePiece.ConeFacingFront or GamePiece.ConeFacingBack
     */
    public GamePiece getConeOrientation() {
        // if no color is detected from the front color sensor then skinny end probably
        // facing front
        if (results[0] == null)
            return GamePiece.ConeFacingFront;
        // if no color is detected from the back color sensor then skinny end probably
        // facinb back
        else if (results[2] == null)
            return GamePiece.ConeFacingBack;

        // if the distance from sensor to object is greater in the front sensor than the
        // second skinny end probably facing front, else facing back
        return ((colorSensorData[0].distance - colorSensorData[2].distance) > 0) ? GamePiece.ConeFacingFront
                : GamePiece.ConeFacingBack;
    }
    
    /**
     * Sets whether to update outtake frame counts.
     * 
     * @param toUpdate Whether to update outtake frame counts.
     */
    public void updateOuttakeFrames(boolean toUpdate) {
        updateOuttakeNumbers = toUpdate;
    }

    /**
     * Whether the lightgate detects a new object (tripped for the threshold frames)
     * 
     * @return Whether the lightgate detectsc a new object.
     */
    public boolean intakeObjectDetectedLightgate() {
        return (intakeLightgateTrippedFrames >= INTAKE_THRESHOLD);
    }

    /**
     * Resets the object detected timer to 0.
     */
    public void resetIntakeFrames() {
        intakeLightgateTrippedFrames = 0;
    }
    
    /**
     * Returns whether the number of frames the lightgate is complete meets the threshold.
     * 
     * Only checks the outtakeLightgateTrueFrames variable because said variable is only updtaed when outtakeLightgateTrippedFrames meets threshold as well.
     * 
     * @return Whether an object has been ejectd.
     */
    public boolean getObjectOuttake() {
        return (outtakeLightgateTrueFrames >= OUTTAKE_THRESHOLD);
    }

    /**
     * Resets the frame counts of all outtake variables to 0.
     */
    public void resetOuttakeFrames() {
        outtakeLightgateTrippedFrames = 0;
        outtakeLightgateTrueFrames = 0;
    }

    /**
     * Sets whether to update the object detected time counter.
     * 
     * @param toUpdate Whether to update the object detected time counter.
     */
    public void setIntakeFramesUpdates(boolean toUpdate) {
        updateIntakeNumbers = toUpdate;
    }

    /**
     * Gets the frames for which an object has been detected.
     * 
     * @return The frames for which an object has been detected.
     */
    public int getIntakeFrames() {
        return intakeLightgateTrippedFrames;
    }

    Command getWatcher() {
        return new MyWatcher();
    }

    /**
     * NetworkTables
     */
    class MyWatcher extends WatcherCmd {

        // NT itself
        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Color Sensors");

        // Sensor 0 (closest to intake -- front)
        DoublePublisher nt_sensor0_r;
        DoublePublisher nt_sensor0_g;
        DoublePublisher nt_sensor0_b;
        DoublePublisher nt_sensor0_ir;
        DoublePublisher nt_sensor0_prox;
        StringPublisher nt_sensor0_object;

        // Sensor 1 (middle)
        DoublePublisher nt_sensor1_r;
        DoublePublisher nt_sensor1_g;
        DoublePublisher nt_sensor1_b;
        DoublePublisher nt_sensor1_ir;
        DoublePublisher nt_sensor1_prox;
        StringPublisher nt_sensor1_object;

        // Sensor 2 (closest to car wash -- back)
        DoublePublisher nt_sensor2_r;
        DoublePublisher nt_sensor2_g;
        DoublePublisher nt_sensor2_b;
        DoublePublisher nt_sensor2_ir;
        DoublePublisher nt_sensor2_prox;
        StringPublisher nt_sensor2_object;

        // Overall numbers
        IntegerPublisher nt_numYellow;
        IntegerPublisher nt_numPurple;
        StringPublisher nt_objectDetected;

        public String getTableName() {
            return ColorSensors.this.getName();
        }

        @Override
        public void ntcreate() {
            NetworkTable nt = getTable();
            // Sensor 0 (closest to intake -- front)
            nt_sensor0_r = nt.getDoubleTopic("Sensor 0 Red").publish();
            nt_sensor0_g = nt.getDoubleTopic("Sensor 0 Green").publish();
            nt_sensor0_b = nt.getDoubleTopic("Sensor 0 Blue").publish();
            nt_sensor0_ir = nt.getDoubleTopic("Sensor 0 IR").publish();
            nt_sensor0_prox = nt.getDoubleTopic("Sensor 0 Proximity").publish();
            nt_sensor0_object = nt.getStringTopic("Sensor 0 Object Detected").publish();

            // Sensor 1 (middle)
            nt_sensor1_r = nt.getDoubleTopic("Sensor 1 Red").publish();
            nt_sensor1_g = nt.getDoubleTopic("Sensor 1 Green").publish();
            nt_sensor1_b = nt.getDoubleTopic("Sensor 1 Blue").publish();
            nt_sensor1_ir = nt.getDoubleTopic("Sensor 1 IR").publish();
            nt_sensor1_prox = nt.getDoubleTopic("Sensor 1 Proximity").publish();
            nt_sensor1_object = nt.getStringTopic("Sensor 1 Object Detected").publish();

            // Sensor 2 (closest to car wash -- back)
            nt_sensor2_r = nt.getDoubleTopic("Sensor 2 Red").publish();
            nt_sensor2_g = nt.getDoubleTopic("Sensor 2 Green").publish();
            nt_sensor2_b = nt.getDoubleTopic("Sensor 2 Blue").publish();
            nt_sensor2_ir = nt.getDoubleTopic("Sensor 2 IR").publish();
            nt_sensor2_prox = nt.getDoubleTopic("Sensor 2 Proximity").publish();
            nt_sensor2_object = nt.getStringTopic("Sensor 2 Object Detected?").publish();

            // Overall numbers
            nt_numYellow = nt.getIntegerTopic("Number of sensors yellow").publish();
            nt_numPurple = nt.getIntegerTopic("Number of sensors purple").publish();
            nt_objectDetected = nt.getStringTopic("Object detected").publish();
        }

        @Override
        public void ntupdate() {
            // Sensor 0 (closest to intake -- front)
            nt_sensor0_r.set(colorSensorData[0].color.red);
            nt_sensor0_g.set(colorSensorData[0].color.green);
            nt_sensor0_b.set(colorSensorData[0].color.blue);
            nt_sensor0_ir.set(colorSensorData[0].ir);
            nt_sensor0_prox.set(colorSensorData[0].distance);
            nt_sensor0_object
                    .set((results[0] == null) ? "Nothing" : (results[0].color.equals(CONE_YELLOW)) ? "Cone" : "Cube");

            // Sensor 1 (middle)
            nt_sensor1_r.set(colorSensorData[1].color.red);
            nt_sensor1_g.set(colorSensorData[1].color.green);
            nt_sensor1_b.set(colorSensorData[1].color.blue);
            nt_sensor1_ir.set(colorSensorData[1].ir);
            nt_sensor1_prox.set(colorSensorData[1].distance);
            nt_sensor1_object
                    .set((results[1] == null) ? "Nothing" : (results[1].color.equals(CONE_YELLOW)) ? "Cone" : "Cube");

            // Sensor 2 (closest to car wash -- back)
            nt_sensor2_r.set(colorSensorData[2].color.red);
            nt_sensor2_g.set(colorSensorData[2].color.green);
            nt_sensor2_b.set(colorSensorData[2].color.blue);
            nt_sensor2_ir.set(colorSensorData[2].ir);
            nt_sensor2_prox.set(colorSensorData[2].distance);
            nt_sensor2_object
                    .set((results[2] == null) ? "Nothing" : (results[2].color.equals(CONE_YELLOW)) ? "Cone" : "Cube");

            // Overall numbers
            nt_numYellow.set(numYellow);
            nt_numPurple.set(numPurple);
            nt_objectDetected.set(currentGamePiece.toString());
        }
    }
}