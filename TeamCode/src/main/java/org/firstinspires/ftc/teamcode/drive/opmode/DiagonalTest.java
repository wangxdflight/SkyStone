package org.firstinspires.ftc.teamcode.drive.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Path;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotLogger;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.List;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(name = "DiagonalTest", group = "drive")
@Disabled
public class DiagonalTest extends LinearOpMode {
    public static double DISTANCE = 0; // update later;
    private String TAG = "DiagonalTest";
    SampleMecanumDrive _drive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.updateConstantsFromProperties();  // Transitional PID is used in base class;;
        DISTANCE = DriveConstants.TEST_DISTANCE;

        if (_drive == null) {
            if (DriveConstants.USING_BULK_READ == false)
                _drive = new SampleMecanumDrive(hardwareMap);
            else
                _drive = new SampleMecanumDrive(hardwareMap);

            _drive.setBrakeonZeroPower(DriveConstants.BRAKE_ON_ZERO);
            _drive.setPoseEstimate(new Pose2d(0, 0, _drive.getExternalHeading()));
        }

        waitForStart();

        while (!isStopRequested()) {
            //_drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, 0));

            if (DriveConstants.DIAGONAL_SPLIT)
                Path.StrafeDiagonalHelper(_drive, new Vector2d(DriveConstants.TEST_DISTANCE, DriveConstants.TEST_DISTANCE_0));
            else {
                Trajectory trajectory = _drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(DriveConstants.TEST_DISTANCE, DriveConstants.TEST_DISTANCE_0))
                        .build();
                _drive.followTrajectory(trajectory);
            }

            Localizer localizer = _drive.getLocalizer();
            if (DriveConstants.RUN_USING_ODOMETRY_WHEEL && (localizer != null)) {
                StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer) localizer; // @TODO
                List<Double> odo_positions = t.getWheelPositions();

                RobotLogger.dd(TAG, "odometry positions");
                _drive.print_list_double(odo_positions);
            }

            List<Double> positions = _drive.getWheelPositions();
            RobotLogger.dd(TAG, "wheel positions");
            _drive.print_list_double(positions);

            Pose2d currentPos = _drive.getPoseEstimate();
            Pose2d error_pose = _drive.follower.getLastError();
            RobotLogger.dd(TAG, "currentPos %s, errorPos %s", currentPos.toString(), error_pose.toString());
            //drive.turnSync(Math.toRadians(90));
            Path.sleep_millisec_opmode(2000, this);

            if (DriveConstants.RESET_FOLLOWER)
                _drive.resetFollowerWithParameters(DriveConstants.USING_STRAFE_DIAGONAL);

            if (DriveConstants.DIAGONAL_SPLIT)
                Path.StrafeDiagonalHelper(_drive, new Vector2d(0, 0));
            else {
                Trajectory trajectory = _drive.trajectoryBuilder()
                        .strafeTo(new Vector2d(0, 0))
                        .build();
                _drive.followTrajectory(trajectory);
            }

            currentPos = _drive.getPoseEstimate();
            error_pose = _drive.follower.getLastError();
            RobotLogger.dd(TAG, "currentPos %s, errorPos %s", currentPos.toString(), error_pose.toString());
            if (DriveConstants.RESET_FOLLOWER)
                _drive.resetFollowerWithParameters(DriveConstants.USING_STRAFE_DIAGONAL);

            Path.sleep_millisec_opmode(5000, this);
        }
    }
}
