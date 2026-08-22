package org.firstinspires.ftc.teamcode.decode;

import android.graphics.Color;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "RedAutoNEAR", group = "Autonomous")
@Config
@Disabled

public class RedAutoNEAR extends LinearOpMode {
    /* HARDWARE */
    private DcMotorEx shooter = null;
    private DcMotor stage1 = null;
    // private DcMotor stage2 = null;
    private DcMotor stage3 = null;
    private Servo blockShooter = null;
    private Servo cameraServo = null;
    //private DistanceSensor leftDist = null;
    //private DistanceSensor rightDist = null;
    GoBildaPinpointDriver pinpoint = null;
    final private double OPENSHOOTER_OPEN = 0.8; //0.19 //0.3;
    final private double OPENSHOOTER_CLOSED = 1.0; // OPENSHOOTER_OPEN + 28//0.55
    final private double CAMERASERVO_HIGH = 0.55;
    final private double CAMERASERVO_LOW = 0.68;
    final private double SHOOTER_VELOCITY = 2100; //4000//4200;//4800;//5000;
    /* INIT */
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 24;//RED //20;//BLUE//24;// -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private Position cameraPosition =  new Position(DistanceUnit.INCH, 0, 5, 13.5, 0); //middle, 5 inch forward, 13.5 height
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -75, 0, 0);//-90

    private ColorBlobLocatorProcessor colorLocator;
    private ElapsedTime runtime = new ElapsedTime();

    /*
    /// 1. Define Action
    ///
    /// when Done, implement the Claw class
    public class Arm {
        private DcMotorEx Arm;

        public Arm(HardwareMap hardwareMap) {
            Arm = hardwareMap.get(DcMotorEx.class, "ArmP2");
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Arm.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ArmUP implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Arm.setPower(-0.8);
                    initialized = true;
                }

                double pos = Arm.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < ARM_UP) {
                    return true;
                } else {
                    Arm.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmUP() {
            return new ArmUP();
        }

        public class ArmDN implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Arm.setPower(0.8);
                    initialized = true;
                }

                double pos = Arm.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > ARM_DN) {
                    return true;
                } else {
                    Arm.setPower(0);
                    return false;
                }
            }
        }
        public Action ArmDN(){
            return new ArmDN();
        }
    }
    public class Extend {
        private DcMotorEx Extend;

        public Extend(HardwareMap hardwareMap) {
            Extend = hardwareMap.get(DcMotorEx.class, "ExtendP1");
            Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Extend.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ExtendOUT implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Extend.setPower(-0.8);
                    initialized = true;

                }

                double pos = Extend.getCurrentPosition();
                packet.put("ExtendPos", pos);
                if (pos > EXTEND_OUT) {
                    return true;
                } else {
                    Extend.setPower(0);
                    return false;
                }
            }
        }
        public Action ExtendOUT() {
            return new ExtendOUT();
        }
     */
    //SHOOTING CLASS
    public class ShootAllAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("ShootAllAction", "Firing 3 shots");
                // Fire three balls in sequence (blocking, similar to SleepAction(3))
                shootN(6);
                //sleep(500); //sleep before moving to next position

                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }

    // Convenience factory so you can just write shootAll() in your SequentialAction
    public Action shootAll() {
        return new ShootAllAction();
    }

    //START THE SHOOTER/POWER THE SHOOTER
    public class PowerShooter implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("PowerShooterAction", "Power shooter to power = 0.9");
                //set the shooter power to 0.9
                //shooter.setVelocity(5400);
                shooter.setVelocity(SHOOTER_VELOCITY);
                //shooter.setPower(0.90);
                //sleep(500);
                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }

    // Convenience factory so you can just write startShooter() in your SequentialAction
    public Action startShooter() {
        return new PowerShooter();
    }

    //closes the shooter gate
    public class moveGate implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("Action", "Closing gate");
                blockShooter.setPosition(OPENSHOOTER_CLOSED);
                //sleep(100);

                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }

    public Action closeGate() {
        return new moveGate();
    }
    //run the intake
    public class startIntakeAction implements Action {
        private boolean initialized = false;
        private double s1, s3;
        public startIntakeAction(double s1_in, double s3_in){
            s1 = s1_in;
            s3 = s3_in;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("Action", "Closing gate");
                //runIntake(0, 0.7, 1.0);
                runIntake(s1, s3);
                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }
    public Action startIntake(double s1, double s3) {
        return new startIntakeAction(s1, s3);
    }

    public class delay implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            long wait = 150;
            if (!initialized) {
                // Optionally log something
                packet.put("Delay", "");
                // Fire three balls in sequence (blocking, similar to SleepAction(3))
                shooter.setVelocity(SHOOTER_VELOCITY);
                sleep(wait);
                //sleep(500); //sleep before moving to next position

                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }

    // Convenience factory so you can just write shootAll() in your SequentialAction
    public Action shooterWait() {
        return new delay();
    }

    /// ***********************************************************************************
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        telemetry.update();

        Pose2d startPose = new Pose2d(-54, 54, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        initMotors();
        initAprilTagAndColorBlob();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Create telemetry thread
        Thread telemetryThread = new Thread(() -> {
            while (!isStopRequested()) {
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Runtime (seconds)", runtime.seconds());
                dashboard.sendTelemetryPacket(packet);
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });
        waitForStart();
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        runtime.reset();
        telemetryThread.start();
        double shootX = -28, shootY = 28; //30, 30
        Pose2d shootPose = new Pose2d(shootX, shootY, Math.toRadians(135));

        try {
            Actions.runBlocking(
                    new SequentialAction(
                            /* Preload */
                            startShooter(),
                            startIntake(1.0, 0.3),
                            //1. Go to shooting place
                            drive.actionBuilder(startPose)
                                    .strafeTo(new Vector2d(shootX, shootY))
                                    .build(),
                            shooterWait(),
                            shootAll(),
                            closeGate(),
                            startIntake(1.0, 0.3),

                            //3. get the inner most 3 balls
                            drive.actionBuilder(shootPose)
                                    .setTangent(Math.toRadians(45))
                                    .splineToLinearHeading(new Pose2d(-24, 38,  Math.toRadians(45)), Math.toRadians(45)) //go into
                                    .splineToLinearHeading(new Pose2d(-6, 60,  Math.toRadians(95)), Math.toRadians(90)) //go into
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(shootPose, Math.toRadians(225)) //go into
                                    .build(),
                            shootAll(),
                            closeGate(),
                            startIntake(1.0, 0.3),

                            //4. get the middle row balls
                            drive.actionBuilder(shootPose)
                                    .setTangent(Math.toRadians(15))
                                    .splineToLinearHeading(new Pose2d(2, 38,  Math.toRadians(45)), Math.toRadians(30)) //go into
                                    .splineToLinearHeading(new Pose2d(4, 62,  Math.toRadians(110)), Math.toRadians(95)) //go into
                                    //turn back to guide balls going out of the classifier
                                    // face 90 degrees forward
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(shootPose, Math.toRadians(200)) //go into
                                    .build(),
                            shootAll(),
                            closeGate(),
                            startIntake(1.0, 0.3),

                            //5. get the outermost row balls
                            drive.actionBuilder(shootPose)
                                    .setTangent(Math.toRadians(10))
                                    .splineToLinearHeading(new Pose2d(26, 35,  Math.toRadians(45)), Math.toRadians(0)) //go into
                                    .splineToLinearHeading(new Pose2d(33, 62,  Math.toRadians(135)), Math.toRadians(135)) //go into
                                    .setTangent(Math.toRadians(200))
                                    .splineToLinearHeading(shootPose, Math.toRadians(200)) //go into
                                    .build(),
                            shootAll(),
                            closeGate()
                    )
            );
            telemetry.addData("Trajectory", "Executed Successfully");
        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
        }
    }
    private void initMotors(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        // leftDist  = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        // rightDist = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        //1. need initial the shooter, stage1, 2, 3, servo
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        stage1 = hardwareMap.get(DcMotor.class, "stage1");
        // stage2 = hardwareMap.get(DcMotor.class, "stage2");
        stage3 = hardwareMap.get(DcMotor.class, "stage3");
        blockShooter = hardwareMap.get(Servo.class, "blockShooter");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //ALREADY set in the driver
//        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
//        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        stage1.setDirection(DcMotor.Direction.REVERSE);
        // stage2.setDirection(DcMotor.Direction.REVERSE);
        stage3.setDirection(DcMotor.Direction.REVERSE);
        blockShooter.setDirection(Servo.Direction.REVERSE); //Do we really need this?

        //shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    //Initialize the AprilTag processor.
    private void initAprilTagAndColorBlob() {
        /******************************************************************************************/
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(822.317, 822.317, 319.495, 242.502)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);
        /******************************************************************************************/
        /* Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
         * - Specify the color range you are looking for. Use a predefined color, or create your own
         *
         *   .setTargetColorRange(ColorRange.BLUE)     // use a predefined color match
         *     Available colors are: RED, BLUE, YELLOW, GREEN, ARTIFACT_GREEN, ARTIFACT_PURPLE
         *   .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,  // or define your own color match
         *                                       new Scalar( 32, 176,  0),
         *                                       new Scalar(255, 255, 132)))
         *
         * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
         *     This can be the entire frame, or a sub-region defined using:
         *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
         *     Use one form of the ImageRegion class to define the ROI.
         *       ImageRegion.entireFrame()
         *       ImageRegion.asImageCoordinates(50, 50,  150, 150)  100x100 pixels at upper left corner
         *       ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5)  50% width/height in center
         *
         * - Define which contours are included.
         *   You can get ALL the contours, ignore contours that are completely inside another contour.
         *     .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
         *     .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
         *     EXTERNAL_ONLY helps to avoid bright reflection spots from breaking up solid colors.
         *
         * - Turn the displays of contours ON or OFF.
         *     Turning these on helps debugging but takes up valuable CPU time.
         *        .setDrawContours(true)                Draws an outline of each contour.
         *        .setEnclosingCircleColor(int color)   Draws a circle around each contour. 0 to disable.
         *        .setBoxFitColor(int color)            Draws a rectangle around each contour. 0 to disable. ON by default.
         *
         *
         * - include any pre-processing of the image or mask before looking for Blobs.
         *     There are some extra processing you can include to improve the formation of blobs.
         *     Using these features requires an understanding of how they may effect the final
         *     blobs.  The "pixels" argument sets the NxN kernel size.
         *        .setBlurSize(int pixels)
         *        Blurring an image helps to provide a smooth color transition between objects,
         *        and smoother contours.  The higher the number, the more blurred the image becomes.
         *        Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
         *        Blurring too much may hide smaller features.  A size of 5 is good for a 320x240 image.
         *
         *     .setErodeSize(int pixels)
         *        Erosion removes floating pixels and thin lines so that only substantive objects remain.
         *        Erosion can grow holes inside regions, and also shrink objects.
         *        "pixels" in the range of 2-4 are suitable for low res images.
         *
         *     .setDilateSize(int pixels)
         *        Dilation makes objects and lines more visible by filling in small holes, and making
         *        filled shapes appear larger. Dilation is useful for joining broken parts of an
         *        object, such as when removing noise from an image.
         *        "pixels" in the range of 2-4 are suitable for low res images.
         *
         *        .setMorphOperationType(MorphOperationType morphOperationType)
         *        This defines the order in which the Erode/Dilate actions are performed.
         *        OPENING:    Will Erode and then Dilate which will make small noise blobs go away
         *        CLOSING:    Will Dilate and then Erode which will tend to fill in any small holes in blob edges.
         */
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
//                .setBlurSize(5)          // Smooth the transitions between different colors in image
//                // the following options have been added to fill in perimeter holes.
//                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
//                .setErodeSize(15)        // Shrink blobs back to original size
                //.setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .setBlurSize(3)             // smaller blur
                .setDilateSize(3)           // much smaller than 15
                .setErodeSize(3)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.OPENING)

                .build();

        /******************************************************************************************/
        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(aprilTag)
                    .addProcessor(colorLocator)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(aprilTag)
                    .addProcessor(colorLocator)
                    .build();
        }
    }
    /*Manually set the camera gain and exposure.  This can only be called AFTER calling initAprilTag(), and only works for Webcams; */
    private void  setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    private void setBlobExposureAuto() {
        if (visionPortal == null) return;

        ExposureControl exposureControl =
                visionPortal.getCameraControl(ExposureControl.class);
        GainControl gainControl =
                visionPortal.getCameraControl(GainControl.class);

        if (exposureControl != null) {
            exposureControl.setMode(ExposureControl.Mode.Auto);
        }

        // Optional: some drivers like to cap gain to reduce noise, if supported
        // if (gainControl != null) { gainControl.setGain(someMaxValue); }
    }

    public void shootOnce(){
        //1. make sure the gate is closed
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        //2. start the shooter
        shooter.setVelocity(SHOOTER_VELOCITY); //max RPM * 0.9
        //shooter.setPower(0.90);
        //sleep(200);

        //3. set stage power
        stage1.setPower(0.6); //1.0 //keep stage1 as intake
        sleep(100);
        // stage2.setPower(-0.4); //use stage 2 as the second gate
        stage3.setPower(-0.3);
        sleep(110);
        stage3.setPower(1); //accelate stage3
        //open the gate so that the ball can go through
        blockShooter.setPosition(OPENSHOOTER_OPEN);
        sleep(200); //250//300
        //4. close the gate
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        stage3.setPower(0);
        // stage2.setPower(0.8);
        sleep(200);//300 //150
    }

    public void shootN(int count) {
        final double targetVel = SHOOTER_VELOCITY + 100; //close = 2200. far = 2500.   // same units you use in setVelocity/getVelocity
        final double dropMargin = 100;         // tune
        final double recoverMargin = 100; //100;      // tune (smaller than dropMargin)
        final double stage3FeedPower = 0.6;    // tune down if multiple balls sneak
        final double stage3HoldPower = 0.0;

        final double GATE_HOLD = OPENSHOOTER_CLOSED;   // you may want a slightly-open "hold" instead
        final double GATE_PULSE_OPEN = OPENSHOOTER_OPEN; // tune so 1 ball passes, not 2

        final int pulseMs = 200;//130;              // tune: shorter = fewer double-feeds
        final int stableMs = 120;             // require speed stable before feeding next ball
        final int loopSleepMs = 15;

        // Spin up
        blockShooter.setPosition(GATE_HOLD);
        shooter.setVelocity(targetVel);

        //stage1.setPower(0.6);  //0.6, 1.0       // intake
        stage3.setPower(stage3HoldPower);

        // sleep(600);
        ElapsedTime time_pass = new ElapsedTime();
        time_pass.reset();

        while(time_pass.milliseconds() <= 1500){
            stage3.setPower(stage3FeedPower);
            blockShooter.setPosition(GATE_PULSE_OPEN);
            sleep(pulseMs);

            // 3) Immediately block the next ball
            blockShooter.setPosition(GATE_HOLD);
            //stage3.setPower(stage3HoldPower);

            // 4) Wait for recovery enough to avoid weak 2nd/3rd shots
            while (opModeIsActive() && shooter.getVelocity() < targetVel - recoverMargin) {
                telemetry.addData("Shooter Vel", "%5.2f", shooter.getVelocity());
                telemetry.update();
                sleep(loopSleepMs);
                idle();
            }
        }

        // Stop / reset
        stage3.setPower(0);
        blockShooter.setPosition(GATE_HOLD);
    }

    //running intake
    public void runIntake(double s1, double s3) {
        stage1.setPower(s1);
        // stage2.setPower(s2);
        stage3.setPower(s3);
    }
}
