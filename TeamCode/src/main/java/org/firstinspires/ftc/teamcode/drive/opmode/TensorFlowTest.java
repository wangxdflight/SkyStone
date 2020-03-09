package org.firstinspires.ftc.teamcode.drive.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.drive.localizer.TensorflowDetector;
import org.firstinspires.ftc.teamcode.drive.localizer.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.VuforiaCameraChoice;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class TensorFlowTest extends LinearOpMode {
    private Trajectory trajectory;
    private BaseTrajectoryBuilder builder, strafe_builder;
    private Pose2d current_pose;
    private String TAG = "TensorFlowTest";
    private SampleMecanumDrive _drive = null;
    private HardwareMap hwMap;
    private Path path;
    private FieldPosition fieldPosition = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.updateConstantsFromProperties();
        RobotLogger.dd(TAG, "unit test for TensorFlow skystone detector");

        waitForStart();

        if (isStopRequested()) return;
        TensorflowDetector vTester = null;

        int count = 0;
        while (opModeIsActive()) {
            RobotLogger.dd(TAG, "looop %d", count);
            if (opModeIsActive())
                vTester = TensorflowDetector.getSingle_instance(hardwareMap, VuforiaCameraChoice.PHONE_BACK);
            else break;

            for (int i = 0; i < 10; i ++ ) {
                if (opModeIsActive())
                    vTester.detectSkystone();
                if (opModeIsActive())
                    Path.sleep_millisec_opmode(200, this);
            }
            count ++;
            vTester.stop();

            if (opModeIsActive())
                vTester = TensorflowDetector.getSingle_instance(hardwareMap, VuforiaCameraChoice.PHONE_FRONT);
            else
                break;

            for (int i = 0; i < 10; i ++ ) {
                if (opModeIsActive())
                    vTester.detectSkystone();
                if (opModeIsActive())
                    Path.sleep_millisec_opmode(200, this);
            }
            count ++;
            vTester.stop();

        }

        RobotLogger.dd(TAG, "----------done --------------------- unit test for tensor flow");
    }
}
