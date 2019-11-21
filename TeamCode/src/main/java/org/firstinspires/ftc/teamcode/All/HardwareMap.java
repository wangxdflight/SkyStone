package org.firstinspires.ftc.teamcode.All;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

public class HardwareMap {
    public DcMotorEx leftRear, rightRear, leftFront, rightFront, firstJoint, secondJoint, intakeJoint, liftOne;
    public DcMotorEx leftEncoder, rightEncoder, frontEncoder;
    public Servo servo1, servo2, servo3;
    public BNO055IMU gyro;
    public IntegratingGyroscope imu;
    public static AnalogInput leftForward, rightForward, sideways;
    public static String TAG = "MainThread";

    public com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;

    public HardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {
        leftRear = (DcMotorEx) hwMap.get(DcMotor.class, "leftRear");
        rightRear = (DcMotorEx) hwMap.get(DcMotor.class, "rightRear");
        leftFront = (DcMotorEx) hwMap.get(DcMotor.class, "leftFront");
        rightFront = (DcMotorEx) hwMap.get(DcMotor.class, "rightFront");

        leftEncoder = (DcMotorEx)hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = (DcMotorEx)hardwareMap.dcMotor.get("rightEncoder");
        frontEncoder = (DcMotorEx)hardwareMap.dcMotor.get("frontEncoder");
        //------------------------===Intake & Lift Motors===------------------------

        //leftIntake = (DcMotorEx) hwMap.get(DcMotor.class, "leftIntake");
        //rightIntake = (DcMotorEx) hwMap.get(DcMotor.class, "rightIntake");
        liftOne = (DcMotorEx) hwMap.get(DcMotor.class, "liftOne");
        //liftTwo = (DcMotorEx) hwMap.get(DcMotor.class, "liftTwo");

        //---------------------------------------------------------------------------

        //------------------------===Servos===------------------------

        servo1 = hwMap.get(Servo.class, "clawServo1");
        servo2 = hwMap.get(Servo.class, "clasServo2");
        servo3 = hwMap.get(Servo.class, "servo3");

        //---------------------------------------------------------------------------

        leftForward = hwMap.get(AnalogInput.class, "leftForward");
        rightForward = hwMap.get(AnalogInput.class, "rightForward");
        sideways = hwMap.get(AnalogInput.class, "sideways");
        gyro = hwMap.get(BNO055IMU.class, "imu");
        imu = (IntegratingGyroscope) gyro;

        //firstJointVirtualServo = new MotorServo(firstJoint, MotorServo.MotorConfiguration.firstJoint);
        //secondJointVirtualServo = new MotorServo(secondJoint, MotorServo.MotorConfiguration.secondJoint);

        this.hardwareMap = hwMap;
    }

    public void gyroInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        gyro.initialize(parameters);
    }

    //--------------------------------==================\/\/Track Encoders\/\/==================---------------------------------

    public static class track extends Thread {  //Encoders: Min = 0.0, Max = 3.3
        static double totalLeft = 0;
        static double totalRight = 0;
        static double totalSide = 0;
        static boolean tracking = false;
        static double startTime = -1;
        static double elapsedTime = -1;
        static int delay = 25;
        static int readDelay = 15;
        static double leftDelta = 0;
        static double rightDelta = 0;
        static double sideDelta = 0;

        private static void beginTracking() {        //Right, Left, and Side--Encoders: Forward = 0 - 1 - 2 - 3 - 0 - 1..., Backward = 3 - 2 - 1 - 0 - 3 - 2...
            Thread update = new Thread() {
                public void run() {
                    if (tracking && startTime == -1)
                        startTime = System.currentTimeMillis();
                    while (tracking) {
                        double leftInit = leftForward.getVoltage();
                        double rightInit = rightForward.getVoltage();
                        double sideInit = sideways.getVoltage();

                        try {
                            Thread.sleep(readDelay);
                        } catch (Exception e) {
                        }

                        double leftFinal = leftForward.getVoltage();
                        double rightFinal = rightForward.getVoltage();
                        double sideFinal = sideways.getVoltage();

                        leftDelta = leftFinal - leftInit;
                        rightDelta = rightFinal - rightInit;
                        sideDelta = sideFinal - sideInit;

                        if (leftDelta >= 2.3) {
                            totalLeft -= leftInit + (3.3 - leftFinal);
                        } else if (leftDelta <= -2.3) {
                            totalLeft += (3.3 - leftInit) + leftFinal;
                        } else {
                            totalLeft += leftDelta;
                        }

                        if (rightDelta >= 2.3) {
                            totalRight -= rightInit + (3.3 - rightFinal);
                        } else if (rightDelta <= -2.3) {
                            totalRight += (3.3 - rightInit) + rightFinal;
                        } else {
                            totalRight += rightDelta;
                        }

                        if (sideDelta >= 2.3) {
                            totalSide += sideInit + (3.3 - sideFinal);
                        } else if (sideDelta <= -2.3) {
                            totalSide -= (3.3 - sideInit) + sideFinal;
                        } else {
                            totalSide += sideDelta;
                        }

                        try {
                            Thread.sleep(delay);
                        } catch (Exception e) {
                        }
                    }
                    elapsedTime = System.currentTimeMillis() - startTime;
                }
            };
            update.start();
        }

        public static ArrayList<Double> getEncoderTicks() {  //Returns ArrayList [leftTicks, rightTicks, sideTicks] in cm
            ArrayList<Double> totalTicks = new ArrayList<>();

            totalTicks.add(totalLeft /
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION * DriveConstant.LEFT_GEAR_RATIO /**
                    (2 * Math.PI * DriveConstant.ODOMETRY_RAD)*/ * (DriveConstant.ODOMETRY_RAD / DriveConstant.MECANUM_RAD));
            totalTicks.add(totalRight /
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION * DriveConstant.RIGHT_GEAR_RATIO /**
                    (2 * Math.PI * DriveConstant.ODOMETRY_RAD)*/ * (DriveConstant.ODOMETRY_RAD / DriveConstant.MECANUM_RAD));
            totalTicks.add(totalSide /
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION * DriveConstant.SIDE_GEAR_RATIO /**
                    (2 * Math.PI * DriveConstant.ODOMETRY_RAD)*/ * (DriveConstant.ODOMETRY_RAD / DriveConstant.MECANUM_RAD));
            return totalTicks;
        }

        public static ArrayList<String> getEncoderDebug() {  //Debugs the entire formula for calculating encoder ticks
            ArrayList<String> debug = new ArrayList<>();

            debug.add(/*"(ignoreLoopsAdditionalValue - getVoltage + 3.3 * numOfLoops)*/ "totalLeft / VOLTS_PER_REVOLUTION * GEAR_RATIO * " +
                    "(2 * PI * ODOMETRY_RADIUS) * (ODOMETRY_RADIUS / MECANUM_RADIUS)");
            debug.add("" + totalLeft + " / " +
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION + " * " + DriveConstant.LEFT_GEAR_RATIO /*+ " * " +
                    "(" + 2 + " * " + Math.PI + " * " + DriveConstant.ODOMETRY_RAD + ")"*/ + " * " +
                    "(" + DriveConstant.ODOMETRY_RAD + " / " + DriveConstant.MECANUM_RAD + ")");
            debug.add("" + totalRight + " / " +
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION + " * " + DriveConstant.RIGHT_GEAR_RATIO /*+ " * " +
                    "(" + 2 + " * " + Math.PI + " * " + DriveConstant.ODOMETRY_RAD + ")"*/ + " * " +
                    "(" + DriveConstant.ODOMETRY_RAD + " / " + DriveConstant.MECANUM_RAD + ")");
            debug.add("" + totalSide + " / " +
                    DriveConstant.ENCODER_COUNTS_PER_REVOLUTION + " * " + DriveConstant.SIDE_GEAR_RATIO /*+ " * " +
                    "(" + 2 + " * " + Math.PI + " * " + DriveConstant.ODOMETRY_RAD + ")"*/ + " * " +
                    "(" + DriveConstant.ODOMETRY_RAD + " / " + DriveConstant.MECANUM_RAD + ")");
            return debug;
        }

        public static ArrayList<String> getElapsedTime() {   //Returns [elapsedTime, CurrentlyTracking]
            ArrayList<String> time = new ArrayList<>();
            if (tracking) {
                time.add(Math.round((System.currentTimeMillis() - startTime) / 1000.0 * 100.0) / 100.0 + "s");
                time.add("Currently Tracking: YES");
            } else {
                time.add(Math.round(elapsedTime / 1000.0 * 100.0) / 100.0 + "s");
                time.add("Currently Tracking: NO");
            }
            return time;
        }

        public static void resetEncoders() {     //Resets everything to 0 including elapsedTime
            leftForward.resetDeviceConfigurationForOpMode();
            rightForward.resetDeviceConfigurationForOpMode();
            sideways.resetDeviceConfigurationForOpMode();
            totalLeft = 0;
            totalSide = 0;
            totalRight = 0;
            startTime = -1;
        }

        public static void encoders(boolean track, int readingDelay, int loopDelay) {
            tracking = track;

            if (readingDelay >= 0)
                readDelay = loopDelay;
            else
                readDelay = 15;

            if (loopDelay >= 0)
                delay = loopDelay;
            else
                delay = 25;

            if (track)
                beginTracking();
        }

        public static void encoders(boolean track) {
            tracking = track;
            readDelay = 15;
            delay = 25;
            if (track)
                beginTracking();
        }
    }

    //----------------------------==================/\/\End Track Encoders/\/\==================-----------------------------
}
