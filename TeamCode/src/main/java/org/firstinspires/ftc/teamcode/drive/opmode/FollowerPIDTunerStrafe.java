package org.firstinspires.ftc.teamcode.drive.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotLogger;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(name = "FollowerPIDTunerStrafe", group = "drive")
@Disabled
public class FollowerPIDTunerStrafe extends LinearOpMode {
    public static double DISTANCE = 0; // update later;
    private String TAG = "FollowerPIDTunerStrafe";
    SampleMecanumDriveBase _drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.updateConstantsFromProperties();  // Transitional PID is used in base class;;
        DISTANCE = DriveConstants.TEST_DISTANCE;

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            if (_drive == null) {
                if (DriveConstants.USING_BULK_READ == false)
                    _drive = new SampleMecanumDriveREV(hardwareMap, DriveConstants.USING_STRAFE_DIAGONAL);
                else
                    _drive = new SampleMecanumDriveREVOptimized(hardwareMap, DriveConstants.USING_STRAFE_DIAGONAL);
                _drive.setBrakeonZeroPower(DriveConstants.BRAKE_ON_ZERO);
                _drive.setPoseEstimate(new Pose2d(0, 0, _drive.getExternalHeading()));
            }
            Pose2d currentPos = _drive.getPoseEstimate();

            if (DriveConstants.USING_STRAFE_DIAGONAL == true) {
                if (DriveConstants.RESET_FOLLOWER)
                    _drive.resetFollowerWithParameters(DriveConstants.USING_STRAFE_DIAGONAL, false);

                _drive.followTrajectorySync(
                        _drive.trajectoryBuilder()
                                .strafeTo((new Vector2d(currentPos.getX() + DriveConstants.TEST_DISTANCE, currentPos.getY() + DriveConstants.TEST_DISTANCE_0)))
                                .build());
            }
            else {
                if (DriveConstants.RESET_FOLLOWER)
                    _drive.resetFollowerWithParameters(DriveConstants.USING_STRAFE_DIAGONAL, false);

                _drive.followTrajectorySync(
                        _drive.trajectoryBuilder()
                                .lineTo(new Vector2d(currentPos.getX() + DriveConstants.TEST_DISTANCE, currentPos.getY() + DriveConstants.TEST_DISTANCE_0))
                                .build());
            }

            currentPos = _drive.getPoseEstimate();
            Pose2d error_pose = _drive.follower.getLastError();
            RobotLogger.dd(TAG, "currentPos %s, errorPos %s",currentPos.toString(), error_pose.toString());
            //drive.turnSync(Math.toRadians(90));
            Path.sleep_millisec_opmode(2000, this);


            if (DriveConstants.USING_STRAFE_DIAGONAL == true) {
                //drive.resetFollowerWithParameters(true);
                _drive.followTrajectorySync(
                        _drive.trajectoryBuilder()
                                .strafeTo((new Vector2d(currentPos.getX() - DriveConstants.TEST_DISTANCE, currentPos.getY() - DriveConstants.TEST_DISTANCE_0)))
                                .build());
            }
            else {
                //drive.resetFollowerWithParameters(false);
                _drive.followTrajectorySync(
                        _drive.trajectoryBuilder()
                                .lineTo((new Vector2d(currentPos.getX() - DriveConstants.TEST_DISTANCE, currentPos.getY() - DriveConstants.TEST_DISTANCE_0)))
                                .build());
            }
            currentPos = _drive.getPoseEstimate();
            error_pose = _drive.follower.getLastError();
            RobotLogger.dd(TAG, "currentPos %s, errorPos %s",currentPos.toString(), error_pose.toString());

            Path.sleep_millisec_opmode(2000, this);

        }
    }
}
