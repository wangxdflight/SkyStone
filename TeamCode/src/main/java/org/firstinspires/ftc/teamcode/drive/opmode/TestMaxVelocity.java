package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Arrays;
import java.util.List;
import java.lang.String;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ODOMETRY_WHEEL;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TestMaxVelocity extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private final int polling_interval = 1000;
    private String TAG = "TestMaxVelocity";
    private double maxVelocity = 0;
    public double MAX_POWER = 1.0;
    // use odometry wheel
    private StandardTrackingWheelLocalizer localizer = null;

    @Override
    public void runOpMode() throws InterruptedException {
        double currentVelocity;
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ODOMETRY_WHEEL) {
            localizer = new StandardTrackingWheelLocalizer(hardwareMap);
            RobotLog.dd(TAG, "StandardTrackingWheelLocalizer created");
        }
        waitForStart();
        for (DcMotorEx motor : motors) {
            motor.setPower(MAX_POWER);  // set maximum power;
        }
        while (opModeIsActive()) {
            currentVelocity = leftFront.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            List<Double> velocities = drive.getWheelVelocities();
            RobotLog.dd(TAG, "velocities");
            drive.print_list_double(velocities);

            List<Double> positions = drive.getWheelPositions();
            RobotLog.dd(TAG, "wheel positions");
            drive.print_list_double(positions);

            List<Double> w_powers = drive.getMotorPowers(motors);
            RobotLog.dd(TAG, "wheel powers");
            drive.print_list_double(w_powers);

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

            if (localizer!=null) {
                List<Double>  odo_positions = localizer.getWheelPositions();
                RobotLog.dd(TAG, "odometry positions");
                drive.print_list_double(odo_positions);
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
            RobotLog.dd(TAG, "current velocity "+Double.toString(currentVelocity)+ " max velocity: "
                    +Double.toString(maxVelocity));
            Thread.sleep(polling_interval);
        }
        for (DcMotorEx motor : motors) {
            motor.setPower(0);  // stop
        }

    }
}
