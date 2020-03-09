package org.firstinspires.ftc.teamcode.drive.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(name = "FollowerPIDTunerStraight", group = "drive")
@Disabled
public class FollowerPIDTunerStraight extends LinearOpMode {
    public static double DISTANCE = 0; // update later;
    private String TAG = "FollowerPIDTunerStraight";
    SampleMecanumDrive drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        if (drive == null) {
            DriveConstants.updateConstantsFromProperties();  // Transitional PID is used in base class;;
            DISTANCE = DriveConstants.TEST_DISTANCE;

            if (DriveConstants.USING_BULK_READ == false)
                drive = new SampleMecanumDrive(hardwareMap);
            else
                drive = new SampleMecanumDrive(hardwareMap);
            drive.setBrakeonZeroPower(DriveConstants.BRAKE_ON_ZERO);
            drive.setPoseEstimate(new Pose2d(0, 0, drive.getExternalHeading()));

        }
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {

            //drive.setPoseEstimate(drive.getPoseEstimate());

            if (DriveConstants.RESET_FOLLOWER)
                drive.resetFollowerWithParameters(false, false);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(DISTANCE)
                            .build()
            );
            Pose2d currentPos = drive.getPoseEstimate();
            Pose2d error_pose = drive.follower.getLastError();
            RobotLog.dd(TAG, "currentPos %s, errorPos %s",currentPos.toString(), error_pose.toString());
            //drive.turnSync(Math.toRadians(90));
            Path.sleep_millisec_opmode(2000, this);

            if (DriveConstants.RESET_FOLLOWER)
                drive.resetFollowerWithParameters(false, false);

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(DISTANCE)
                            .build()
            );
            currentPos = drive.getPoseEstimate();
            error_pose = drive.follower.getLastError();
            RobotLog.dd(TAG, "currentPos %s, errorPos %s",currentPos.toString(), error_pose.toString());
            Path.sleep_millisec_opmode(2000, this);
        }
    }
}
