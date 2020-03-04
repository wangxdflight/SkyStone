package org.firstinspires.ftc.teamcode.drive.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotLogger;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.VuforiaCameraChoice;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class VuforiaTest extends LinearOpMode {
    private Trajectory trajectory;
    private BaseTrajectoryBuilder builder, strafe_builder;
    private Pose2d current_pose;
    private String TAG = "VuforiaTest";
    private SampleMecanumDrive _drive = null;
    private HardwareMap hwMap;
    private Path path;
    private FieldPosition fieldPosition = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.updateConstantsFromProperties();
        RobotLogger.dd(TAG, "unit test for vuforia localizer");

        waitForStart();

        if (isStopRequested()) return;
        VuforiaCamLocalizer vLocalizer = null;

        int count = 0;
        while (opModeIsActive()) {
            if (opModeIsActive())
                vLocalizer = VuforiaCamLocalizer.getSingle_instance(hardwareMap, VuforiaCameraChoice.PHONE_BACK, true);
            else
                break;
            Pose2d poseEstimate;
            for (int i = 0; i < 10; i ++ ) {
                if (opModeIsActive()) {
                    poseEstimate = vLocalizer.getPoseEstimate();
                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", poseEstimate.getHeading());
                    telemetry.update();
                    RobotLogger.dd(TAG, "vuforia localization: " + poseEstimate.toString());
                }
                if (opModeIsActive())
                    Path.sleep_millisec_opmode(200, this);
            }
            vLocalizer.stop();

            count ++;
            if (opModeIsActive())
                vLocalizer = VuforiaCamLocalizer.getSingle_instance(hardwareMap, VuforiaCameraChoice.PHONE_FRONT, true);
            else
                break;
            for (int i = 0; i < 10; i ++ ) {
                if (opModeIsActive()) {
                    poseEstimate = vLocalizer.getPoseEstimate();
                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", poseEstimate.getHeading());
                    telemetry.update();
                    RobotLogger.dd(TAG, "vuforia localization: " + poseEstimate.toString());
                }
                if (opModeIsActive())
                    Path.sleep_millisec_opmode(200, this);
            }
            vLocalizer.stop();

            count ++;

        }

        RobotLogger.dd(TAG, "----------done --------------------- unit test for vuforia localizer");
    }
}
