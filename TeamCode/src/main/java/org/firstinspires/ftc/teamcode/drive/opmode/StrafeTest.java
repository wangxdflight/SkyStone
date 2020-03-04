
package org.firstinspires.ftc.teamcode.drive.calibration;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.RobotLogger;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;


import java.util.List;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.rear_ratio;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "StrafeTest", group = "drive")
@Disabled
public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = DriveConstants.TEST_DISTANCE;
    private AllHardwareMap hwMap;
    private String TAG = "StrafeTest";
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new AllHardwareMap(hardwareMap);
        DriveConstants.updateConstantsFromProperties();
        DISTANCE = DriveConstants.TEST_DISTANCE;
        SampleMecanumDrive drive = null;
        if (DriveConstants.USING_BULK_READ == false)
            drive = new SampleMecanumDrive(hardwareMap);
        else
            drive = new SampleMecanumDrive(hardwareMap);
        drive.setBrakeonZeroPower(DriveConstants.BRAKE_ON_ZERO);

/*
        DISTANCE = 72;  //Non-Bulk Read Code
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        drive.setBrakeonZeroPower(DriveConstants.BRAKE_ON_ZERO);

        DriveConstants.strafeDistance(hardwareMap, 24, false);
        //odometryStrafe(0.2, 24, false);

*/
        RobotLogger.dd(TAG, "trajectoryBuilder strafe, DISTANCE: "+Double.toString(DISTANCE));
        Trajectory trajectory = drive.trajectoryBuilder()
                .strafeLeft(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
        Localizer localizer = drive.getLocalizer();
        /*
        if (DriveConstants.RUN_USING_ODOMETRY_WHEEL && (localizer!=null)) {
            StandardTrackingWheelLocalizer t = (StandardTrackingWheelLocalizer)localizer; // @TODO
            List<Double> odo_positions = t.getWheelPositions();

            RobotLogger.dd(TAG, "odometry positions");
            drive.print_list_double(odo_positions);
        }
        */

        List<Double> positions = drive.getWheelPositions();
        RobotLogger.dd(TAG, "wheel positions");
        //drive.print_list_double(positions);

    }

    public void odometryStrafe(double power, double inches, boolean right){
        double counts = inches / (2 * PI * 1.25) * 1550.0;
        int sidewaysStart = hwMap.rightIntake.getCurrentPosition();

        if(!right)
            power = -power;

        hwMap.frontRight.setPower(-power);
        hwMap.frontLeft.setPower(power);
        hwMap.backRight.setPower(power * rear_ratio);
        hwMap.backLeft.setPower(-power * rear_ratio);

        while (opModeIsActive() || !isStarted()) {
            int sideways = hwMap.rightIntake.getCurrentPosition();

            int sidewaysDiff = Math.abs(sideways - sidewaysStart);
                telemetry.addData("Side", sidewaysDiff);
                telemetry.addData("Target", counts);
                telemetry.update();

            if (sidewaysDiff >= counts) {
                break;
            }
        }
        hwMap.frontRight.setPower(0);
        hwMap.frontLeft.setPower(0);
        hwMap.backRight.setPower(0);
        hwMap.backLeft.setPower(0);
    }
}
