package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * Constants generated by LearnRoadRunner.com/drive-constants
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class Constants {
    // voltage compensated kF 14.834777618422862

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(40, 0, 9,
            15.35);

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 1.88976; // in
    public static double GEAR_RATIO = 38.0/34.62029076719591 * 43.0/36.0; // output (wheel) speed / input (motor) speed. Target: 36, Measured: 43
    public static double TRACK_WIDTH = 12.4/Math.sqrt(2); // in 11.9/Math.sqrt(2)

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    /*
     * Note from LearnRoadRunner.com:
     * The velocity and acceleration constraints were calculated based on the following equation:
     * ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85
     * Resulting in 52.48180821614297 in/s.
     * This is only 85% of the theoretical maximum velocity of the bot, following the recommendation above.
     * This is capped at 85% because there are a number of variables that will prevent your bot from actually
     * reaching this maximum velocity: voltage dropping over the game, bot weight, general mechanical inefficiencies, etc.
     * However, you can push this higher yourself if you'd like. Perhaps raise it to 90-95% of the theoretically 
     * max velocity. The theoretically maximum velocity is 61.74330378369762 in/s.
     * Just make sure that your bot can actually reach this maximum velocity. Path following will be detrimentally
     * affected if it is aiming for a velocity not actually possible.
     * 
     * The maximum acceleration is somewhat arbitrary and it is recommended that you tweak this yourself based on
     * actual testing. Just set it at a reasonable value and keep increasing until your path following starts
     * to degrade. As of now, it simply mirrors the velocity, resulting in 52.48180821614297 in/s/s
     *
     * Maximum Angular Velocity is calculated as: maximum velocity / trackWidth * (180 / Math.PI) but capped at 360??/s.
     * You are free to raise this on your own if you would like. It is best determined through experimentation.
     
     */
//    public static double MAX_VEL = 38.69921064314256;
//    public static double MAX_ACCEL = 52.48180821614297;
//    public static double MAX_ANG_VEL = Math.toRadians(140.25002251507732);
//    public static double MAX_ANG_ACCEL = Math.toRadians(184.02607784577722);
    public static double MAX_VEL = 30;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = Math.toRadians(120);
    public static double MAX_ANG_ACCEL = Math.toRadians(120);

    //region JebRunner Constants
    public static final int ARM_TPS = 1500;
    public static final int MOVE_TPS = 900;

    public static final int INTAKE_CONE_DISTANCE = 8;
    public static final double DEFAULT_ARM_POWER = 0.4;
    public static final double MIN_ARM_POWER = 0.2;
    public static final double DEFAULT_DRIVE_POWER = 0.4;
    public static final double MIN_DRIVE_POWER = 0.2;
    public static final double ARM_DRIVE_POWER = 0.2;

    public static final int MIN_ARM_SLIDE_POS = 0;
    public static final int MAX_ARM_SLIDE_POS = 3500;

    public static final int PICKUP_ARM_POS = 184;
    public static final int LOW_ARM_POS = 1552;
    public static final int MID_ARM_POS = 2662;
    public static final int HIGH_ARM_POS = 3850;

    public static final double POWER_PER_P = 0.5;

    public static final int SLIDE_VEL = 1500;

    public static final double TICKS_PER_POWER = 537.7 * 312 / 60; // ppr * rpm / (seconds per min)
    public static final double TICKS_PER_CM = 24.97406315350325;
    public static final double CM_PER_INCH = 2.54;
    public static final double CM_PER_TILE = 60.325;


    public static final float MIN_RESULT_CONFIDENCE = 0.45f;
    public static final int TFOD_INPUT_SIZE = 600;

    public static final String VUFORIA_KEY = "ASAgPkT/////AAABmcTllI2PFk1wiMjhlIY1WS8Ovl54qUtjzOSa3fzMnC9V2C5Ow73wnC6xQbPR2agidsoI2fC8QGvo7TXT03j1B6dUlZ4azcy/1gOlzGwY9ZahRTEz7Ey9uuCKTh4sZrXVjD5oxAzVYIVo+3GR+YzwLtT843sIZGRx0eBlHyokJiAyb+fAkwnwIMg137n6Jxeyw4Opm18oWD+GgtZVCg25IhpIf53nrPC0ABvVuL1Lz4qErWFcQbMEXqNbMQVSIBMuX1LkrVoFPPmhMDY1c7bTI4VPLmPyUfGauoYw9DIW7c/ZfwAGs4cf+va2oJZBOwUyd3CPomicBKcsMgey6RBaKLHO4oNhNazpvFvxnr399ckg";

    public static final String[] CONE_LABELS = new String[] { "drax", "spring", "ryan" }; // 0 1 2
    public static final String CONE_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/mechjeb_lite3.tflite";
    //endregion

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
      // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
      return 32767 / ticksPerSecond;
    }
}