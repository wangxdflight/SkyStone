package org.firstinspires.ftc.teamcode.drive;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotLogger;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.lang.reflect.Array;
import java.util.ArrayList;
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
 */
@Config
public class  StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double WHEEL_RADIUS = 1.25; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    private String TAG = "StandardTrackingWheelLocalizer";
    private ExpansionHubEx hubMotors;
    //private ExpansionHubMotor leftEncoder, rightEncoder, frontEncoder;
    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    //private List<ExpansionHubMotor> motors;

    //private DcMotor leftEncoder0, rightEncoder0, frontEncoder0;
    private List<DcMotor> motors;
    private List<Double> odomTicksPerRev;

    private org.firstinspires.ftc.teamcode.drive.localizer.IMUBufferReader imuReader;

    Pose2d poseEstimate_new = new Pose2d(0, 0, 0);

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(1.0, DriveConstants.ODOMETRY_TRACK_WIDTH / 2, 0), // left
                new Pose2d(1.0, -DriveConstants.ODOMETRY_TRACK_WIDTH / 2, 0), // right
                new Pose2d(DriveConstants.ODOMERY_FORWARD_OFFSET, -0.7, Math.toRadians(90)) // front
        ));
        if (DriveConstants.USING_BULK_READ) {
            hubMotors = hardwareMap.get(ExpansionHubEx.class, "ExpansionHub3");
            leftEncoder = hardwareMap.get(ExpansionHubMotor.class, "leftEncoder");
            rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "rightEncoder");
            frontEncoder = hardwareMap.get(ExpansionHubMotor.class, "frontEncoder");
        }
        else {
            leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
            rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
            frontEncoder = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        }
        RobotLogger.dd(TAG, "StandardTrackingWheelLocalizer created");

        motors = Arrays.asList(leftEncoder, rightEncoder, frontEncoder);

        odomTicksPerRev = Arrays.asList(DriveConstants.odoEncoderTicksPerRevLeft,
                DriveConstants.odoEncoderTicksPerRevRight, DriveConstants.odoEncoderTicksPerRevFront);

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        imuReader = org.firstinspires.ftc.teamcode.drive.localizer.IMUBufferReader.getSingle_instance(hardwareMap);
    }

    public double encoderTicksToInches(int ticks, int index) {
        RobotLogger.dd("StandardTrackingWheelLocalizer", "encoderTicksToInches: " + " ticks: " +
                Double.toString(ticks) + " inches: " + Double.toString(WHEEL_RADIUS * 2 * Math.PI *
                GEAR_RATIO * ticks / odomTicksPerRev.get(index)));
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / odomTicksPerRev.get(index);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int x, y, z;
        RobotLogger.dd(TAG, "to getOdomWheelPositions (bulk? %d)", (DriveConstants.USING_BULK_READ==true?1:0));

        if (DriveConstants.USING_BULK_READ) {
            RevBulkData bulkData = hubMotors.getBulkInputData();
            if (bulkData == null) {
                RobotLogger.dd(TAG, "bulk data = null");
                return Arrays.asList(0.0, 0.0, 0.0, 0.0);
            }

            x = bulkData.getMotorCurrentPosition(leftEncoder) * (-1);
            y = bulkData.getMotorCurrentPosition(rightEncoder);
            z = bulkData.getMotorCurrentPosition(frontEncoder);
        }
        else
        {
            x = leftEncoder.getCurrentPosition() * (-1);
            y = rightEncoder.getCurrentPosition() ;
            z = frontEncoder.getCurrentPosition();
        }
        RobotLogger.dd(TAG, "getOdomWheelPositions");
        //RobotLogger.dd(TAG, "leftEncoder: " + x);
        //RobotLogger.dd(TAG, "rightEncoder: " + y);
        //RobotLogger.dd(TAG, "frontEncoder: " + (-1)*z);
        return Arrays.asList(
                encoderTicksToInches(x, 0),
                encoderTicksToInches(y, 1),
                encoderTicksToInches(-1*z, 2)
        );
    }

    @Override
    public Pose2d getPoseEstimate() {
        //RobotLogger.dd(TAG, "getPoseEstimate: " + Double.toString(poseEstimate_new.getX()) + ", " + Double.toString(poseEstimate_new.getY()) + ", " +
        //        Double.toString(poseEstimate_new.getHeading()));
        return poseEstimate_new;
    }

    @Override
    public void setPoseEstimate(Pose2d pose2d) {
        super.setPoseEstimate(pose2d);
        RobotLogger.dd(TAG, "setPoseEstimate: X "+Double.toString(pose2d.getX())+ ", Y "+Double.toString(pose2d.getY()));
    }

    @Override
    public void update() {
        super.update();
        Pose2d s_poseEstimate=super.getPoseEstimate();

        if (DriveConstants.RUN_USING_IMU_LOCALIZER == true) {
            RobotLogger.dd(TAG, "to read IMU");
            poseEstimate_new = new Pose2d(s_poseEstimate.getX(), s_poseEstimate.getY(),
                    imuReader.getLatestIMUData());
            RobotLogger.dd(TAG, "using IMU: IMU heading " + Double.toString(poseEstimate_new.getHeading()) + " non-IMU heading: "
            + Double.toString(s_poseEstimate.getHeading()));
        }
        else {
            RobotLogger.dd(TAG, "not using IMU for heading");
            poseEstimate_new = s_poseEstimate;
        }
        RobotLogger.dd(TAG, "poseEstimate: "+Double.toString(poseEstimate_new.getX()) + ", " + Double.toString(poseEstimate_new.getY()) + ", " +
                Double.toString(poseEstimate_new.getHeading()));
    }
    public void finalize() throws Throwable {
        imuReader.cleanUP();
    }
}
