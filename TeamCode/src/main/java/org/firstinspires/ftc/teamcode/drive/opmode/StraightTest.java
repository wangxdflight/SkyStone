package org.firstinspires.ftc.teamcode.drive.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.localizer.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;

import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "StraightTest", group = "drive")
@Disabled
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 72;
    private PIDCoefficients coefficients;
    private double kV;
    private String TAG = "StraightTest";
    @Override
    public void runOpMode() {
        DriveConstants.updateConstantsFromProperties();
        SampleMecanumDriveBase drive = null;
        if (DriveConstants.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, false);
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);

        AllHardwareMap hwMap = new AllHardwareMap(hardwareMap);

        //FourWheelMecanumDrivetrain drivetrain = new FourWheelMecanumDrivetrain(hwMap);

        //drivetrain.setMotorZeroPower(DcMotor.ZeroPowerBehavior.BRAKE);
        //drivetrain.resetEncoders();
        //drivetrain.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Trajectory trajectory = drive.trajectoryBuilder()
                .forward(DISTANCE)
                .build();


        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
        Localizer localizer = drive.getLocalizer();
        if (DriveConstants.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
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
