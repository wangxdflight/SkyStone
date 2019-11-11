/*
 * Copyright (c) 2018 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// [ref:] https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.util.RobotLog;
import com.acmerobotics.dashboard.config.Config;


//import org.firstinspires.ftc.teamcode.CSVWriter;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

// https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
@Config
@TeleOp(name = "Concept: ConceptEasyOpenCV", group = "Concept")
public class ConceptOpenCV extends LinearOpMode
{
    OpenCvCamera phoneCam;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new SamplePipeline());

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();
            RobotLog.i("Frame Count " + phoneCam.getFrameCount());
                    /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                phoneCam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * The "if" statements below will pause the viewport if the "X" button on gamepad1 is pressed,
             * and resume the viewport if the "Y" button on gamepad1 is pressed.
             */
            else if(gamepad1.x)
            {
                phoneCam.pauseViewport();
            }
            else if(gamepad1.y)
            {
                phoneCam.resumeViewport();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        // THIS DETECTOR RETURNS THE PIXEL LOCATION OF THE LEFT MOST BOUNDARY OF THE BLACK TARGET
        // YOU CAN EASILY MODIFY IT TO GET YOU THE CENTER
        // IT IS YOUR JOB TO DETERMINE WHERE YOU WANT TO GO BASED ON THIS VALUE

        // SOMETIMES THERE IS SOME RANDOM CRASH THAT HAPPENS IF UR CAMERA WOBBLES A LOT
        // I AM LOOKING INTO A FIX, BUT AS LONG AS UR CAMERA DOESN"T FLAIL WILDLY IN MATCH U R GOOD

        // IF YOU SEE BUGS PLEASE MESSAGE ME ON DISCORD at Epsilon#0036
        // THIS IS NO MEANS ROBUST, BUT IT WORKS WELL FOR ME

        // THIS DETECTOR SACRIFICES SPEED FOR A LOT OF VERSATILITY
        // IT FUNCTIONS WITH LOTS OF NOISE BY PERFORMING LOTS OF FILTERS
        // IF YOU FEEL THAT SOME PARTS OF THIS PIPELINE ARENT NEEDED, THEN REMOVE THEM
        // TO IMPROVE FRAMERATE


        private final Scalar minHSV = new Scalar(11.8, 161.7, 116.5);
        private final Scalar maxHSV = new Scalar(30.3, 255.0, 255.0);

        private final Point anchor = new Point(-1,-1);
        private final int erodeIterations = 10;

        private final int dilateIterations = 20;

        // THESE NEED TO BE TUNED BASED ON YOUR DISTANCE FROM THE BLOCKS
        private final double minContourArea = 300.0;
        private final double minContourPerimeter = 1000.0;
        private final double minContourWidth = 300.0;
        private final double minContourHeight = 0.0;

        private final double cbMin = 105;
        private final double cbMax = 140;

        private int minX, minY = Integer.MAX_VALUE;
        private int maxX, maxY = -1 * Integer.MAX_VALUE;

        // TUNE THESE THEY WILL VARY BASED ON WEBCAM PLACEMENT!!!!!

        private final int maxVumarkValue = 150;
        private final int valleyLength = 40;

        private int vumarkLeftBoundary = -1;

        private Mat mask = new Mat();
        private Mat kernel = new Mat();


        private Mat hierarchy = new Mat();

        // private CSVWriter csvWriter = new CSVWriter(new File("colsums.java"));

        private boolean shouldWrite = false;
        
        // https://github.com/Epsilon10/SKYSTONE-CV/blob/master/VisionPipeline.java
        public Mat processFrame(Mat input) {
            this.shouldWrite = true;
            RobotLog.i("frame, size: " + input.total());
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            Mat workingMat = input.clone();
            Imgproc.cvtColor(workingMat,workingMat,Imgproc.COLOR_RGB2HSV); // convert to HSV space

            Core.inRange(workingMat, minHSV, maxHSV, mask); // apply yellow filter
            Imgproc.erode(mask, mask, kernel, anchor, erodeIterations); // basically a faster blur
            Imgproc.dilate(mask, mask, kernel, anchor, dilateIterations); // remove noise

            List<MatOfPoint> stoneContours = new ArrayList<>();
            Imgproc.findContours(mask, stoneContours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE); // find block contours

            // remove noisy contours
            List<MatOfPoint> outputContours = new ArrayList<>();
            filterContours(stoneContours,outputContours, minContourArea, minContourPerimeter, minContourWidth, minContourHeight);

            workingMat.release();

            // draw stones (but not skystones)
            if (minX > 1e5 || maxX < 0 || minY > 1e5 || maxY < 0) return mask;

            Rect r = new Rect(new Point(minX, minY + 150), new Point(maxX,  maxY));
            Imgproc.rectangle(input, r, new Scalar(0,0,255));

            // crop vertically

            Mat cbMat = crop(input.clone(),new Point(0, minY + 150), new Point(input.width() - 1,  maxY));
            input = crop(input,new Point(0, minY + 150), new Point(input.width() - 3,  maxY));

            Imgproc.cvtColor(cbMat,cbMat,Imgproc.COLOR_RGB2YCrCb); // convert to ycrcb
            Core.extractChannel(cbMat, cbMat, 2); // extract cb channel

            Imgproc.threshold(cbMat, cbMat, cbMin, cbMax, Imgproc.THRESH_BINARY_INV); // binary mask to find the vumark

            List<MatOfPoint> stoneContours2 = new ArrayList<>();
            Imgproc.findContours(cbMat, stoneContours2, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE); // draw contours around the stones
            List<MatOfPoint> finalOutputContours = new ArrayList<>();
            resetRectangle();
            // build the max bounding rect
            filterContours(stoneContours2,finalOutputContours, minContourArea, minContourPerimeter, minContourWidth, minContourHeight);

            if (finalOutputContours.size() == 0) return input;

            // crop so we only see the stones
            Rect allStonesRect = getMaxRectangle();
            Mat cbCrop = crop(cbMat, allStonesRect);

            // crop so we only have the stones in the image
            Imgproc.drawContours(input, finalOutputContours, -1, new Scalar(0,0,255), 10);


            Imgproc.rectangle(input, getMaxRectangle(), new Scalar(0,255,0), 8);


            vumarkLeftBoundary = getMaxDropoff(cbCrop);

            // this just estimates where the marker is, dont rely on this
            Imgproc.circle(input,new Point(vumarkLeftBoundary + 200, cbMat.height() / 2), 150, new Scalar(255,0,0), 10);
            resetRectangle();


        if (vumarkLeftBoundary < (2 * input.width() / 7))
            RobotLog.i("DIR: ", "left");
        else if (vumarkLeftBoundary > ( 5 * input.width() / 7))
            RobotLog.i("DIR: ", "right");
        else
            RobotLog.i("DIR: ", "center");

            return input;
        }

        private void filterContours(List<MatOfPoint> contours, List<MatOfPoint> outputContours, double minContourArea, double minContourPerimeter, double minContourWidth,
                                    double minContourHeight) {
            //resetRectangle();
            RobotLog.d("NumContours " +  contours.size() + "");
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                int x = rect.x;
                int y = rect.y;
                int w = rect.width;
                int h = rect.height;

                if (w < minContourWidth)
                    continue;
                if (rect.area() < minContourArea)
                    continue;
                if ((2 * w + 2 * h) < minContourPerimeter)
                    continue;
                if (h < minContourHeight)
                    continue;
                outputContours.add(contour);

                if (x < minX) minX = x;
                if (y < minY) minY = y;
                if (x + w > maxX) maxX = x + w;
                if (y + h> maxY) maxY = y + h;
            }

        }

        private Mat crop(Mat image, Point topLeftCorner, Point bottomRightCorner) {
            Rect cropRect = new Rect(topLeftCorner, bottomRightCorner);
            return new Mat(image, cropRect);
        }

        private Mat crop(Mat image, Rect rect) {
            return new Mat(image, rect);
        }

        private int getMaxDropoff(Mat image) {

            Mat colsums = new Mat();
            Core.reduce(image, colsums, 0, Core.REDUCE_SUM, 4);

            colsums.convertTo(colsums, CvType.CV_32S);

            int[] colsumArray = new int[(int)(colsums.total()*colsums.channels())];
            colsums.get(0,0,colsumArray);
            for (int i = 0; i < colsumArray.length; i++) {
                colsumArray[i] /= 140;
            }


            for (int i = 0; i < colsumArray.length; i++) {

                if (colsumArray[i] < maxVumarkValue) {
                    // Log.d("colsum", i+"");
                    if (i + valleyLength > colsumArray.length - 1) return 101; // probably fix this lol
                    int[] slice = Arrays.copyOfRange(colsumArray, i, i+valleyLength);
                    if (isLargeValley(slice, 80, 25)) {

                        return i;
                    }
                }
            }
            return -1;
        }

        private boolean isLargeValley(int[] slice, double maxValue, double thresh) {
            List<Integer> differences = new ArrayList<>(slice.length - 1);
            for (int i : slice) {
                if (i > maxValue) return false;

            }
            for (int i = 0; i < slice.length - 1; i++)
                differences.add(Math.abs(slice[i+1] - slice[i]));

            return Collections.max(differences) < thresh;
        }

        public int getVumarkLeftBoundary() { return vumarkLeftBoundary; }

        private void resetRectangle() {
            maxX = maxY = -1 * (int) 1e8;
            minX = minY = (int) 1e8;
        }

        private Rect getMaxRectangle() { return new Rect(new Point(minX, minY), new Point(maxX,maxY)); }

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        public Mat processFrame_(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            RobotLog.i("frame, size: " + input.total());
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }
    }
}