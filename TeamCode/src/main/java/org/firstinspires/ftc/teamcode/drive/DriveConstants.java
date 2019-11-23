package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.motors.Matrix12vMotor;
import com.qualcomm.hardware.motors.MatrixLegacyMotor;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;
/*
 * Constants shared between multiple drive types.
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
public class DriveConstants {
    /*
     * The type of motor used on the drivetrain. While the SDK has definitions for many common
     * motors, there may be slight gear ratio inaccuracies for planetary gearboxes and other
     * discrepancies. Additional motor types can be defined via an interface with the
     * @DeviceProperties and @MotorType annotations.
     */
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);// NeveRest20Gearmotor
            // Matrix12vMotor  GoBILDA5202Series MatrixLegacyMotor (757.12)
    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = true;
    public static final PIDCoefficients MOTOR_VELO_PID = null;

    public static final boolean RUN_USING_ODOMETRY_WHEEL = false;
    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 2*MOTOR_CONFIG.getGearing(); // ???  output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 12.0;
    public static double HARDCODED_TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 0.0111; //1.0 / rpmToVelocity(getMaxRpm()); // 195
    public static double kA = 0;
    public static double kStatic = 0;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     */
    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            24.0, 12.0, Math.PI / 2,
            Math.toRadians(180.0), Math.toRadians(180.0), Math.PI / 4
    );


    public static double encoderTicksToInches(double ticks) {
        RobotLog.dd("DriveConstants", "MOTOR_CONFIG.getTicksPerRev(vs. 383.6): " + Double.toString(MOTOR_CONFIG.getTicksPerRev())
        + " GEAR_RATIO "+Double.toString(GEAR_RATIO*1.0));

        double s = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / HARDCODED_TICKS_PER_REV; //MOTOR_CONFIG.getTicksPerRev();
        RobotLog.dd("DriveConstants", "encoderTicksToInches: " + "ticks: " + Double.toString(ticks) + " inches: " + Double.toString(s));
        return s;
    }

    public static double rpmToVelocity(double rpm) {
        double s = rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        RobotLog.dd("DriveConstants", "rpmToVelocity: " + "rpm " + Double.toString(rpm) + " v " + Double.toString(s));
        return s;
    }

    public static double getMaxRpm() {
        return 435.0;
        /*
        RobotLog.dd("DriveConstants", "MOTOR_CONFIG.getAchieveableMaxRPMFraction(): " + Double.toString(MOTOR_CONFIG.getAchieveableMaxRPMFraction()));
        RobotLog.dd("DriveConstants", "MOTOR_CONFIG.getMaxRPM(): " + Double.toString(MOTOR_CONFIG.getMaxRPM()));
        return MOTOR_CONFIG.getMaxRPM() *
                (RUN_USING_ENCODER ? MOTOR_CONFIG.getAchieveableMaxRPMFraction() : 1.0);  */
    }

    public static double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        //double t = MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getTicksPerRev() / 60.0;
        //double t = MOTOR_CONFIG.getMaxRPM() * HARDCODED_TICKS_PER_REV / 60.0;
        double t = 435.0 * HARDCODED_TICKS_PER_REV / 60.0;
        RobotLog.dd("DriveConstants", "getTicksPerSec "+Double.toString(t));
        return t;
    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        double t = getTicksPerSec();
        RobotLog.dd("DriveConstants", "getTicksPerSec "+Double.toString(t));
        return 32767 / getTicksPerSec();
    }
}
