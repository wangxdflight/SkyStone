package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Arrays;
import java.util.List;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1400; //  TBD: correct?
    public static double WHEEL_RADIUS = 2.9; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.0; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 6.5; // in; offset of the lateral wheel
    private String TAG = "StandardTrackingWheelLocalizer";

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        frontEncoder = hardwareMap.dcMotor.get("frontEncoder");
        RobotLog.dd(TAG, "StandardTrackingWheelLocalizer created");
        /*
        (double) hwMap.leftIntake.getCurrentPosition(),
        (double) hwMap.liftTwo.getCurrentPosition(),  //@TODO: Switch to "hwMap.backRight.getCurrentPosition()" later
        double) hwMap.rightIntake.getCurrentPosition() */
    }

    public static double encoderTicksToInches(int ticks) {
        double t = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
        RobotLog.dd("StandardTrackingWheelLocalizer", "encoderTicksToInches: " + " ticks: " + Double.toString(ticks) + " inches: " + Double.toString(t));
        return t;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RobotLog.dd(TAG, "getWheelPositions");
        RobotLog.dd(TAG, "leftEncoder: " + leftEncoder.getCurrentPosition());
        RobotLog.dd(TAG, "rightEncoder: " + rightEncoder.getCurrentPosition());
        RobotLog.dd(TAG, "frontEncoder: " + frontEncoder.getCurrentPosition());

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }
}
