package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;


import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_IMU_LOCALIZER;

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
    public static double TICKS_PER_REV = 1400.0; //  TBD: correct?
    public static double WHEEL_RADIUS = 1.25;//1.14173; // 2.9cm; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //public static double LATERAL_DISTANCE = 15.5; // in; distance between the left and right wheels
    //public static double FORWARD_OFFSET = -5.49; // in; offset of the lateral wheel
	private String TAG = "StandardTrackingWheelLocalizer";
    private DcMotorEx leftEncoder, rightEncoder, frontEncoder;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    Pose2d poseEstimate_new = new Pose2d(0, 0, 0);

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, BNO055IMU imu_) {
        super(Arrays.asList(
                new Pose2d(0, DriveConstants.ODOMETRY_TRACK_WIDTH / 2, 0), // left
                new Pose2d(0, -1 * DriveConstants.ODOMETRY_TRACK_WIDTH / 2, 0), // right
                new Pose2d(DriveConstantsPID.ODOMERY_FORWARD_OFFSET, -1.56, Math.toRadians(90)) // front
        ));
        this.imu = imu_;
        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        RobotLog.dd(TAG, "StandardTrackingWheelLocalizer created");

        motors = Arrays.asList(leftEncoder, rightEncoder, frontEncoder);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        /*
        (double) hwMap.leftIntake.getCurrentPosition(),
        (double) hwMap.liftTwo.getCurrentPosition(),  //@TODO: Switch to "hwMap.backRight.getCurrentPosition()" later
        double) hwMap.rightIntake.getCurrentPosition() */
        if (DriveConstants.RUN_USING_IMU_LOCALIZER==true) {
            // imu is already initialized in Drive class;
        }
        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
    }

    public static double encoderTicksToInches(int ticks) {
        double t = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
        RobotLog.dd("StandardTrackingWheelLocalizer", "encoderTicksToInches: " + " ticks: " + Double.toString(ticks) + " inches: " + Double.toString(t));
        return t;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int x = leftEncoder.getCurrentPosition();
        int y = rightEncoder.getCurrentPosition();
        int z = frontEncoder.getCurrentPosition();
        RobotLog.dd(TAG, "getWheelPositions");
        RobotLog.dd(TAG, "leftEncoder: " + x);
        RobotLog.dd(TAG, "rightEncoder: " + y);
        RobotLog.dd(TAG, "frontEncoder: " + (-1)*z);
        return Arrays.asList(
                encoderTicksToInches(x),
                encoderTicksToInches(y),
                encoderTicksToInches(-1*z)
        );
    }

    @Override
    public Pose2d getPoseEstimate() {
        RobotLog.dd(TAG, "getPoseEstimate: " + Double.toString(poseEstimate_new.getX()) + ", " + Double.toString(poseEstimate_new.getY()) + ", " +
                Double.toString(poseEstimate_new.getHeading()));
        return poseEstimate_new;
    }

    @Override
    public void setPoseEstimate(Pose2d pose2d) {
        super.setPoseEstimate(pose2d);
        RobotLog.dd(TAG, "setPoseEstimate: X "+Double.toString(pose2d.getX())+ ", Y "+Double.toString(pose2d.getY()));
    }

    @Override
    public void update() {
        super.update();
        Pose2d s_poseEstimate=super.getPoseEstimate();

        if (DriveConstants.RUN_USING_IMU_LOCALIZER == true) {
            poseEstimate_new = new Pose2d(s_poseEstimate.getX(), s_poseEstimate.getY(),
                    imu.getAngularOrientation().firstAngle);
            RobotLog.dd(TAG, "using IMU: IMU heading " + Double.toString(poseEstimate_new.getHeading()) + " non-IMU heading: "
            + Double.toString(s_poseEstimate.getHeading()));
        }
        else {
            poseEstimate_new = s_poseEstimate;
            RobotLog.dd(TAG, "not using IMU");
        }
        RobotLog.dd(TAG, "poseEstimate: "+Double.toString(poseEstimate_new.getX()) + ", " + Double.toString(poseEstimate_new.getY()) + ", " +
                Double.toString(poseEstimate_new.getHeading()));
    }
}
