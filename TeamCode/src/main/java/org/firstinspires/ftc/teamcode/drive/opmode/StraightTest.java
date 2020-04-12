package org.firstinspires.ftc.teamcode.drive.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;

import java.util.List;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.SafeSleep;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "StraightTest", group = "drive")
//@Disabled
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 72;
    private PIDCoefficients coefficients;
    private double kV;
    private String TAG = "StraightTest";
    @Override

    public void runOpMode() throws InterruptedException {
        DriveConstants.updateConstantsFromProperties();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

<<<<<<< HEAD

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();


        waitForStart();
=======
        while (! isStopRequested()) {
            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                    .forward(DISTANCE)
                    .build();

            waitForStart();
>>>>>>> 1bfdaffe627ecff62eee34726c32027b6d53928d

            if (isStopRequested()) return;

<<<<<<< HEAD
        drive.followTrajectory(trajectory);

        Localizer localizer = drive.getLocalizer();
        if (DriveConstants.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
            StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
            List<Double> odo_positions = t.getWheelPositions();

            RobotLog.dd(TAG, "odometry positions");
            //drive.print_list_double(odo_positions);
        }

        List<Double> positions = drive.getWheelPositions();
        RobotLog.dd(TAG, "wheel positions");
        //drive.print_list_double(positions);
=======
            SafeSleep.sleep_milliseconds(this, 500);

            drive.followTrajectory(trajectory);

            trajectory = drive.trajectoryBuilder(drive.getLocalizer().getPoseEstimate())
                    .back(DISTANCE)
                    .build();
            drive.followTrajectory(trajectory);
            SafeSleep.sleep_milliseconds(this,500);
        }
>>>>>>> 1bfdaffe627ecff62eee34726c32027b6d53928d
    }
}
