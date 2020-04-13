package org.firstinspires.ftc.teamcode.drive.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.Autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotLogger;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.VuforiaCameraChoice;

import org.firstinspires.ftc.teamcode.TeleOp.TeleopConstants;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;
import java.lang.String;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "ManualParamTest", group = "drive")
@Disabled
public class ManualParamTest extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private final int polling_interval = 1000;
    private String TAG = "ManualParamTest";
    Localizer localizer = null;
    private AllHardwareMap hwMap;
    private FieldPosition side = FieldPosition.RED_QUARY;

    private void sleep_millisec(int c)
    {
        try {
            Thread.sleep(c);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    //IMUBufferReader imu = IMUBufferReader.getSingle_instance(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.updateConstantsFromProperties();
        hwMap = new AllHardwareMap(hardwareMap);
        SampleMecanumDrive drive = null;
        if (DriveConstants.USING_BULK_READ == false)
            drive = new SampleMecanumDrive(hardwareMap);
        else
            drive = new SampleMecanumDrive(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        localizer = drive.getLocalizer();
        waitForStart();

        if (DriveConstants.ENABLE_ARM_ACTIONS) {
            Path.initGrab(hwMap, side, this);
        }

        VuforiaCamLocalizer vu = VuforiaCamLocalizer.getSingle_instance(hardwareMap, VuforiaCameraChoice.PHONE_BACK, true);
        while (!isStopRequested()) {
            if (DriveConstants.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
                StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
                List<Double>  odo_positions = t.getWheelPositions();

                RobotLogger.dd(TAG, "odometry positions");
                drive.print_list_double(odo_positions);
            }

            List<Double> velocities = drive.getWheelVelocities();
            RobotLogger.dd(TAG, "velocities");
            drive.print_list_double(velocities);

            List<Double> positions = drive.getWheelPositions();
            RobotLogger.dd(TAG, "wheel positions");
            drive.print_list_double(positions);

            Pose2d pose = drive.getPoseEstimate();
            RobotLogger.dd(TAG, "Pose: x " + pose.getX());
            RobotLogger.dd(TAG, "Pose: y " + pose.getY());
            RobotLogger.dd(TAG, "Pose: heading " + Double.toString(pose.getHeading()));

            Pose2d vPose = vu.getPoseEstimate();
            RobotLogger.dd(TAG, "vuforia loc: " + vPose.toString());
            double v_double = DriveConstants.getTeamCodePropertyValue("debug.ftc.grab");
            if (v_double != Double.MAX_VALUE) {
                int v_int = (int) v_double;
                if (v_int != 0) {
                    Path.grabStone(hwMap, side, this);
                }
            }
            Thread.sleep(polling_interval);
        };

    }
}
