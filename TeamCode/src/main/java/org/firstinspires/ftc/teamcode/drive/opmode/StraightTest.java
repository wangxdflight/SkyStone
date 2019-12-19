package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "StraightTest", group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = DriveConstants.getTestDistance();
	private String TAG = "StraightTest";
    @Override
    public void runOpMode() throws InterruptedException {
        DISTANCE = DriveConstants.getTestDistance();
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        RobotLog.dd(TAG, "trajectoryBuilder forward, DISTANCE: "+Double.toString(DISTANCE));
        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
        Localizer localizer = drive.getLocalizer();
        if (DriveConstants.getUseOdometryWheelorNot() && (localizer!=null)) {
            StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
            List<Double> odo_positions = t.getWheelPositions();

            RobotLog.dd(TAG, "odometry positions");
            drive.print_list_double(odo_positions);
        }

        List<Double> positions = drive.getWheelPositions();
        RobotLog.dd(TAG, "wheel positions");
        drive.print_list_double(positions);

    }
}
