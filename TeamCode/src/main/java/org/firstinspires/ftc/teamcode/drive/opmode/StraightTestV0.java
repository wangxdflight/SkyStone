
package org.firstinspires.ftc.teamcode.drive.calibration;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotLogger;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;


import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "StraightTestV0", group = "drive")
@Disabled
public class StraightTestV0 extends LinearOpMode {
    public static double DISTANCE = DriveConstants.TEST_DISTANCE;
    private String TAG = "StraightTest";
    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.updateConstantsFromProperties();
        DISTANCE = DriveConstants.TEST_DISTANCE;
        SampleMecanumDrive drive = null;
        if (DriveConstants.USING_BULK_READ == false)
            drive = new SampleMecanumDrive(hardwareMap);
        else
            drive = new SampleMecanumDrive(hardwareMap);
        drive.setBrakeonZeroPower(DriveConstants.BRAKE_ON_ZERO);
        RobotLogger.dd(TAG, "trajectoryBuilder forward, DISTANCE: "+Double.toString(DISTANCE));
        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);
        Localizer localizer = drive.getLocalizer();
        if (DriveConstants.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
            StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
            List<Double> odo_positions = t.getWheelPositions();

            RobotLogger.dd(TAG, "odometry positions");
            drive.print_list_double(odo_positions);
        }

        List<Double> positions = drive.getWheelPositions();
        RobotLogger.dd(TAG, "wheel positions");
        drive.print_list_double(positions);

    }
}
