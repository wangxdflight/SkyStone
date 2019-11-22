package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;
import java.util.List;
import java.lang.String;
/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class ManualParamTest extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private final int polling_interval = 1000;
    private String TAG = "ManualParamTest";
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        waitForStart();

        while (opModeIsActive()) {
            List<Double> velocities = drive.getWheelVelocities();
            RobotLog.dd(TAG, "velocities");
            print_list_double(velocities);

            List<Double> positions = drive.getWheelPositions();
            RobotLog.dd(TAG, "wheel positions");
            print_list_double(positions);

            List<Double> w_powers = drive.getWheelPowers(motors);
            RobotLog.dd(TAG, "wheel powers");
            print_list_double(w_powers);

            double heading = drive.getExternalHeading();
            RobotLog.dd(TAG, "getExternalHeading: x " + heading);

            Pose2d pose = drive.getPoseEstimate();
            RobotLog.dd(TAG, "Pose: x " + pose.getX());
            RobotLog.dd(TAG, "Pose: y " + pose.getY());
            RobotLog.dd(TAG, "Pose: heading " + Double.toString(pose.getHeading()));

            Pose2d error = drive.getLastError();
            RobotLog.dd(TAG, "xError " + error.getX());
            RobotLog.dd(TAG, "yError " + error.getY());
            RobotLog.dd(TAG, "headingError "  + error.getHeading());
            Thread.sleep(polling_interval);
        }
    }
}
