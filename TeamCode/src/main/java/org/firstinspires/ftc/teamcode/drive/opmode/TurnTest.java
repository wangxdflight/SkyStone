package org.firstinspires.ftc.teamcode.drive.calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "TurnTest", group = "drive")
@Disabled
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 180; // deg
    private PIDCoefficients coefficients;
    private double TRACK_WIDTH;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.updateConstantsFromProperties();
        SampleMecanumDriveBase drive = null;

        if (DriveConstants.USING_BULK_READ == false)
            drive = new SampleMecanumDriveREV(hardwareMap, false);
        else
            drive = new SampleMecanumDriveREVOptimized(hardwareMap, false);

        int selected = 0;
        boolean blocker1 = false;
        boolean blocker2 = false;
        boolean blocker3 = false;

        PIDCoefficients oldPID = new PIDCoefficients(DriveConstants.MOTOR_VELO_PID.kP, DriveConstants.MOTOR_VELO_PID.kI,
                DriveConstants.MOTOR_VELO_PID.kD);
        double oldTrackWidth = DriveConstants.TRACK_WIDTH;

        coefficients = new PIDCoefficients(DriveConstants.MOTOR_VELO_PID.kP, DriveConstants.MOTOR_VELO_PID.kI,
                DriveConstants.MOTOR_VELO_PID.kD);
        TRACK_WIDTH = DriveConstants.TRACK_WIDTH;

        while (!isStarted()) {
            if (gamepad1.left_stick_y >= 0.5) {
                ANGLE -= 1;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            } else if (gamepad1.left_stick_y <= -0.5) {
                ANGLE += 1;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            }

            if (ANGLE < 0)
                ANGLE = 0;

            if (gamepad1.right_stick_y >= 0.5) {
                TRACK_WIDTH -= 0.1;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            } else if (gamepad1.right_stick_y <= -0.5) {
                TRACK_WIDTH += 0.1;
                try {
                    Thread.sleep(100);
                } catch (Exception e) {
                }
            }

            if (gamepad1.left_bumper) {
                DriveConstants.TRACK_WIDTH = TRACK_WIDTH;
                drive = new SampleMecanumDriveREV(hardwareMap, false);
            }

            telemetry.addData("Instructions", "L stick to change distance. R stick to " +
                    "change kV. L bumper to save values.");

            telemetry.addData("ANGLE", ANGLE);
            telemetry.addData("TrackWidth", TRACK_WIDTH);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;
        drive.turnSync(Math.toRadians(ANGLE));
    }
}
