package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcontroller.internal.HSCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.utilities.*;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;


/**
 * Created by David Austin on 10/27/2016.
 */

public abstract class FirstAuton extends OpMode {
    IO_SkyStone_Test io;
    static final int INIT = 0;
    static final int EXECUTE = 1;
    static final int STOP = 2;
    static final int FINISHED = 3;
    int state;
    int initState;
    ArrayList<BasicCommand> commands;
    /*ArrayList<BasicCommand> commandsInit;
    ArrayList<BasicCommand> commandsInitDOM1;
    ArrayList<BasicCommand> commandsInitDOM2;*/
    BasicCommand currentCommand;
    /*BasicCommand currentCommandInit;
    BasicCommand currentCommandInitDOM1;
    BasicCommand currentCommandInitDOM2;*/
    Iterator<BasicCommand> iterator;
    /*Iterator<BasicCommand> iteratorInit;
    Iterator<BasicCommand> iteratorInitDOM1;
    Iterator<BasicCommand> iteratorInitDOM2;*/
    //int allianceColor = IO.RED;



    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};

    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    //OpenCvCamera phoneCam;
    OpenCvCamera webcam;











    public FirstAuton() {
        super();
        this.msStuckDetectInit = 20000;
    }

    public void init() {
        //io = new IO_4WD_Test(hardwareMap, telemetry);
        io = new IO_SkyStone_Test(hardwareMap, telemetry);
        //io.setAllianceColor(allianceColor);
        BasicCommand.setIO(io);
        BasicCommand.setMap(hardwareMap);
        BasicCommand.setTelemetry(telemetry);
        io.gripperRotateStowed();
        //io.leftHookUp();
        //io.rightHookUp();

        io.gripperPincherStopped();
        io.gripperPincher2Stopped();
        //io.leftHookMid();
        //io.rightHookMid();
        //io.rightHookMidDown();
        io.rightHookMid();
        io.leftHookUp();
        //io.rightHookUp();
        io.capStoneUp();

        //io.retractHands();
        //io.openRelicHand();
        //io.jewelArmUp();
        //io.proximityArmUp();
        //io.hookStop();
        //io.markerBoxFlat();
        //telemetry.addData("Status", " Hook and Marker Box Initialized");
        io.calibrateGyroandIMU();
        //io.calibrateGyroandIMU1();
        //HSCamera.setHardwareMap(hardwareMap);

/*        //io.setAllianceColor(allianceColor);
        BasicCommand.setIO(io);
        BasicCommand.setTelemetry(telemetry);*/

        commands = new ArrayList<BasicCommand>();
        /*commandsInit = new ArrayList<BasicCommand>();
        commandsInitDOM1 = new ArrayList<BasicCommand>();
        commandsInitDOM2 = new ArrayList<BasicCommand>();*/
        //addInitCommands();
        //addInitDOM1Commands();
        //addInitDOM2Commands();
        addCommands();
        addFinalCommands();
        iterator = commands.iterator();
        /*iteratorInit = commandsInit.iterator();
        iteratorInitDOM1 = commandsInitDOM1.iterator();
        iteratorInitDOM2 = commandsInitDOM2.iterator();*/
        currentCommand = iterator.next();
        /*currentCommandInit = iteratorInit.next();
        currentCommandInitDOM1 = iteratorInitDOM1.next();
        currentCommandInitDOM2 = iteratorInitDOM2.next();*/
        state = INIT;
        //initState = INIT;

        if (io.getAllianceColor() == IO_SkyStone_Test.RED) {
            offsetX = -1f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
            offsetY = .4f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

            midPos = new float[] {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
            leftPos = new float[] {2f/8f+offsetX, 4f/8f+offsetY};
            rightPos = new float[] {6f/8f+offsetX, 4f/8f+offsetY};
        }


        if (io.getAllianceColor() == IO_SkyStone_Test.BLUE) {
            offsetX = .7f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
            offsetY = .4f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

            midPos = new float[] {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
            leftPos = new float[] {2f/8f+offsetX, 4f/8f+offsetY};
            rightPos = new float[] {6f/8f+offsetX, 4f/8f+offsetY};
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC



    }

    /*
 * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
 */
    @Override
    public void init_loop() {
        // make sure the gyro is calibrated.
        /*if (io.gyro.isCalibrating())  {
            telemetry.addData(">", "Gyro Calibrating. Do Not Move!");
        } else {
            telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        }
*/
        if (!io.imu.isGyroCalibrated()) {
            telemetry.addData(">", "IMU Calibrating. Do Not Move!");
        } else {
            telemetry.addData(">", "IMU Calibrated.");
        }

        telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
        telemetry.addData("Height", rows);
        telemetry.addData("Width", cols);

        if (io.getAllianceColor() == IO_SkyStone_Test.RED) {
            telemetry.addData("Alliance", "RED");
            if (valLeft == 0) {
                telemetry.addData("Skystone", "Left");
                io.skystoneDirection = -1;
                io.fbskystoneDirection = -1;
                io.distanceBetweenSkystones = 13;
                io.centerOffset = 0;
            }
            if (valMid == 0) {
                telemetry.addData("Skystone", "Middle");
                io.skystoneDirection = 0;
                io.fbskystoneDirection = 0;
                io.distanceBetweenSkystones = 24;
                io.centerOffset = 0;
            }
            if (valRight == 0) {
                telemetry.addData("Skystone", "Right");
                io.skystoneDirection = 1;
                io.fbskystoneDirection = 1;
                io.distanceBetweenSkystones = 24;
                io.centerOffset = 0;
            }
        }

        if (io.getAllianceColor() == IO_SkyStone_Test.BLUE) {
            telemetry.addData("Alliance", "BLUE");
            if (valLeft == 0) {
                telemetry.addData("Skystone", "Left");
                io.skystoneDirection = -1;
                io.fbskystoneDirection = 1;
                io.distanceBetweenSkystones = 24;
                io.centerOffset = 4;
            }
            if (valMid == 0) {
                telemetry.addData("Skystone", "Middle");
                io.skystoneDirection = 0;
                io.fbskystoneDirection = 0;
                io.distanceBetweenSkystones = 24;
                io.centerOffset = 3;
            }
            if (valRight == 0) {
                telemetry.addData("Skystone", "Right");
                io.skystoneDirection = 1;
                io.fbskystoneDirection = -1;
                io.distanceBetweenSkystones = 13;
                io.centerOffset = 3;
            }
        }

        /*if (!io.imu1.isGyroCalibrated()) {
            telemetry.addData(">", "IMU1 Calibrating. Do Not Move!");
        } else {
            telemetry.addData(">", "IMU1 Calibrated");
        }*/

        //telemetry.addData("VuMark from IdentifyVuMark from IO", io.getVuMark());

        //if (!currentCommandInit.isFinished()) {
        //if (io.getVuMark() == IO_4WD_Test.UNKNOWN) {
            /*switch (initState) {
                case INIT:
                    currentCommandInit.init();
                    initState = EXECUTE;
                    break;
                case EXECUTE:
                    if (currentCommandInit.isFinished()) {
                        currentCommandInit.stop();
                        if (iteratorInit.hasNext()) {
                            currentCommandInit = iteratorInit.next();
                            initState = INIT;
                        } else initState = EXECUTE; //FINISHED;
                        break;
                    }
                    currentCommandInit.execute();
                    break;
                case STOP:
                    currentCommandInit.stop();
                    if (iteratorInit.hasNext()) {
                        currentCommandInit = iteratorInit.next();
                        initState = INIT;
                    } else initState = FINISHED;
                    break;
                case FINISHED:
                    break;

            }*/
        //}

        /*if (io.getVuMark() == 0) {
            telemetry.addData("vuMark", "Unknown");
            telemetry.addData("vuMark", "not identified, try INIT again");
        } else if (io.getVuMark() == 1) {
            telemetry.addData("vuMark", "Left");
            telemetry.addData("vuMark", "Ready to START");
        } else if (io.getVuMark() == 2) {
            telemetry.addData("vuMark", "Center");
            telemetry.addData("vuMark", "Ready to START");
        } else if (io.getVuMark() == 3) {
            telemetry.addData("vuMark", "Right");
            telemetry.addData("vuMark", "Ready to START");
        }*/
    }

    public void start() {
        //io.setGyroOffset();
        io.setIMUOffset();
        io.resetDriveEncoders();
    }
    public abstract void addCommands();
    public abstract void addFinalCommands();

    /*public void addInitCommands() {
        commandsInit.add(new IdentifyVuMark());
    }*/

    /*public void addInitDOM1Commands() {
        commandsInitDOM1.add(new ArmAngleMovement(0, ArmAngleMovement.INCREASINGDIRECTION, .25));
    }*/

    /*public void addInitDOM2Commands() {
        commandsInitDOM2.add(new DOM2Movement(0, DOM2Movement.INCREASINGDIRECTION, .25));
    }*/

    public void loop() {
        //currentCommandInitDOM1.execute();
        //currentCommandInitDOM2.execute();
        io.updatePosition();
        switch(state){
            case INIT:
                currentCommand.init();
                state = EXECUTE;
                break;
            case EXECUTE:
                if(currentCommand.isFinished()){
                    currentCommand.stop();
                    if (iterator.hasNext()) {
                        currentCommand = iterator.next();
                        state = INIT;
                    } else state = FINISHED;
                    break;
                }
                currentCommand.execute();
                break;
            case STOP:
                currentCommand.stop();
                if(iterator.hasNext()){
                    currentCommand = iterator.next();
                    state = INIT;
                }else state = FINISHED;
                break;
            case FINISHED:
                break;

        }
    }

    public void stop() {
        io.setDrivePower(0,0, 0 ,0);
    }



    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private opencvSkystoneDetector.StageSwitchingPipeline.Stage stageToRenderToViewport = opencvSkystoneDetector.StageSwitchingPipeline.Stage.detection;
        private opencvSkystoneDetector.StageSwitchingPipeline.Stage[] stages = opencvSkystoneDetector.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }

}
