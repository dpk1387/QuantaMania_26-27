package org.firstinspires.ftc.teamcode.decode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

@Autonomous(name = "RED NEAR AUTONOMOUS", group = "Autonomous")
//@Config
@Disabled
public class RedGateNear_Version2 extends LinearOpMode {
    /* HARDWARE */
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    final private double SHOOTER_GEAR_RATIO = 17.0/18.0;

    private DcMotor stage1 = null;
    // private DcMotor stage2 = null;
    private DcMotor stage3 = null;
    private Servo blockShooter = null;
    private Servo cameraServo = null;
    GoBildaPinpointDriver pinpoint = null;
    final private double OPENSHOOTER_OPEN = 0.8; //0.19 //0.3;
    final private double OPENSHOOTER_CLOSED = 1.0; // OPENSHOOTER_OPEN + 28//0.55
    final private double SHOOTER_VELOCITY = 1900; //2100 //2200 //2150
    /* INIT */
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 24;//RED //20;//BLUE//24;// -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private Position cameraPosition =  new Position(DistanceUnit.INCH, 0, 5, 13.5, 0); //middle, 5 inch forward, 13.5 height
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -75, 0, 0);//-90

    private ColorBlobLocatorProcessor colorLocator;
    double shootx2, shooty2;

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

    public class intakeDelay1 implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("intake Delay", "");
                // Fire three balls in sequence (blocking, similar to SleepAction(3))
                sleep(500-200); //1000 //1500

                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }

    // Convenience factory so you can just write shootAll() in your SequentialAction
    public Action intakeWait1() {
        return new intakeDelay1();
    }

    public class intakeDelay2 implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Optionally log something
                packet.put("intake Delay", "");
                // Fire three balls in sequence (blocking, similar to SleepAction(3))
                sleep(400-150); //1500

                initialized = true;
            }
            // Returning false tells Road Runner this action is finished
            return false;
        }
    }

    // Convenience factory so you can just write shootAll() in your SequentialAction
    public Action intakeWait2() {
        return new intakeDelay2();
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
        // initAprilTagAndColorBlob();

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
        double shootX = -23, shootY = 23; //-29, 29
        //-25, 25 //-28, 28  //30, 30
        double newShootX = -11, newShootY = 20; //-23, 23
        //-26, 26 //-24, 24 //-25, 25 //-23, 23 //-21, 21 //-16, 16 //-13, 13
        Pose2d shootPose = new Pose2d(shootX, shootY, Math.toRadians(135));
        Pose2d newShootPose = new Pose2d(newShootX, newShootY, Math.toRadians(135-8));

        double shootx2 = -30, shooty2 = 20; //-23, 23
        Pose2d newShootPose2 = new Pose2d(shootx2, shooty2, Math.toRadians(115));


        Pose2d classifierPose = new Pose2d(7.5+0.2+0.5, 64+2,  Math.toRadians(120-5)); //120-6 //120
        Pose2d backClassifierPose = new Pose2d(7.5+0.5, 64+1,  Math.toRadians(120-5-3)); //120-6 //120


        while (opModeIsActive()){
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.update();

            try {
                Actions.runBlocking(
                        new SequentialAction(
                                startShooter(),
                                startIntake(0.9, 0.3),
                                //go to shooting place
                                drive.actionBuilder(startPose)
                                        .strafeTo(new Vector2d(newShootX, newShootY))
                                        .build(),
//                                shooterWait(), //let shooter accelerate
                                shootAll(), //shoot 3 balls
//                                closeGate(),
                                startIntake(0.9, 0.3), //start intake

                                //get the middle row balls
                                drive.actionBuilder(newShootPose)
                                        .setTangent(Math.toRadians(-3)) // -5
                                        .splineToSplineHeading(new Pose2d(7.5, 29, Math.toRadians(80)), Math.toRadians(50))

                                        .splineToLinearHeading(new Pose2d(5.5, 55-5, Math.toRadians(110)), Math.toRadians(108-6))
                                        .setTangent(Math.toRadians(-90))
                                        //.setTangent(Math.toRadians(32)) //15
                                        //go to intake balls
                                        //.splineToSplineHeading(new Pose2d(12, 42, Math.toRadians(90)), Math.toRadians(90)) //_, _,_, 95

                                        //go to shoot

                                        //.setTangent(Math.toRadians(-100))
                                        .splineToSplineHeading(new Pose2d(0, 31, Math.toRadians(120)), Math.toRadians(-135)) //200
                                        .splineToLinearHeading(newShootPose, Math.toRadians(-170)) //-160, -200
                                        .build(),
                                shootAll(), //shoot balls
//                                closeGate(),
                                startIntake(0.9, 0.3),

                                //INTAKE FROM CLASSIFIER
                                //***********FIRST TIME***********
                                //get balls from classifier
                                drive.actionBuilder(newShootPose)
                                        .setTangent(Math.toRadians(15))

                                        //.setTangent(Math.toRadians(25)) //15
                                        .splineToLinearHeading(classifierPose, Math.toRadians(95)) //80 //85 //95 //go into
                                        //Pose2d classifierPose = new Pose2d(7.5+0.2+0.5, 64+2,  Math.toRadians(120)); //120-6 //120
//                                        .splineToLinearHeading(backClassifierPose, Math.toRadians(95))
                                        .build(),
                                //wait at the classifier to intake balls
                                intakeWait1(),
                                //go to shoot
                                drive.actionBuilder(classifierPose)
                                        .setTangent(Math.toRadians(-95)) //-100 //-90
                                        .splineToLinearHeading(newShootPose, Math.toRadians(-175)) //-160 //-155 //200 //go into
                                        .build(),
                                shootAll(), //shoot all
//                                closeGate(),
                                startIntake(0.9, 0.3), //intake again if time

                                //***********SECOND TIME***********
                                drive.actionBuilder(newShootPose)
                                        .setTangent(Math.toRadians(15)) //25 //15
                                        .splineToLinearHeading(classifierPose, Math.toRadians(95)) //85 //95 //go into
                                        .splineToLinearHeading(backClassifierPose, Math.toRadians(95))
                                        .build(),
                                //wait at the classifier to intake balls
                                intakeWait2(),
                                //go to shoot
                                drive.actionBuilder(classifierPose)
                                        .setTangent(Math.toRadians(-95)) //-95 //-90
                                        .splineToLinearHeading(newShootPose, Math.toRadians(-175)) //-155 //200 //go into
                                        .build(),
                                shootAll(), //shoot all
//                                closeGate(),
                                startIntake(0.9, 0.3), //intake again if time

                                //***********THIRD***********
                                drive.actionBuilder(newShootPose)
                                        .setTangent(Math.toRadians(15)) //25 //15
                                        .splineToLinearHeading(classifierPose, Math.toRadians(95)) //85 //95 //go into
                                        .splineToLinearHeading(backClassifierPose, Math.toRadians(95))
                                        .build(),
                                //wait at the classifier to intake balls
                                intakeWait2(),
                                //go to shoot
                                drive.actionBuilder(classifierPose)
                                        .setTangent(Math.toRadians(-95)) //-95 //-90
                                        .splineToLinearHeading(newShootPose, Math.toRadians(-175)) //-155 //200 //go into
                                        .build(),
                                shootAll(), //shoot all
//                                closeGate(),
                                startIntake(0.9, 0.3), //intake again if time

                                //get the inner most 3 balls
                                drive.actionBuilder(newShootPose)
                                        .setTangent(Math.toRadians(90)) //45 //60
                                        .splineToSplineHeading(new Pose2d(-12, 36,  Math.toRadians(85)), Math.toRadians(85))
                                        .splineToLinearHeading(new Pose2d(-13.5, 53+4, Math.toRadians(100)), Math.toRadians(115))

                                        //go back to shooting
                                        .splineToSplineHeading(newShootPose2, Math.toRadians(100)) //-135 //go into
                                        .build(),

                                shootAll()
                                //shoot 3 balls
//                                closeGate(),

//                                //get out of launch zone
//                                drive.actionBuilder(shootPose)
//                                        .strafeTo(new Vector2d(-5, 32))
//                                        .build()
                        )
                );
                telemetry.addData("Trajectory", "Executed Successfully");
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }
            blockShooter.setPosition(OPENSHOOTER_CLOSED);
            break; ///quite the opmode loop

        }
    }
    private void initMotors(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");

        //1. need initial the shooter, stage1, 2, 3, servo
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "topShooterMotor");

        stage1 = hardwareMap.get(DcMotor.class, "stage1");
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
        stage3.setDirection(DcMotor.Direction.REVERSE);
        blockShooter.setDirection(Servo.Direction.REVERSE); //Do we really need this?

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shootN(int count) {
        final double targetVel = SHOOTER_VELOCITY; //close = 2200. far = 2500.   // same units you use in setVelocity/getVelocity
        final double dropMargin = 100;         // tune
        final double lowRecoverMargin = 100; //100;      // tune (smaller than dropMargin)
        final double highRecoverMargin = 150; //100 //75
        final double stage3FeedPower = 0.6;    //tune down if multiple balls sneak
        final double stage3HoldPower = 0.0;

        startIntake(0.9, 0.3); //start intake

        final int pulseMs = 250; //180;//400;//130;              // tune: shorter = fewer double-feeds
        final int loopSleepMs = 15;

        // Spin up
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        shootVelocity(targetVel);
        stage3.setPower(stage3HoldPower);

        // sleep(600);
        ElapsedTime time_pass = new ElapsedTime();
        time_pass.reset();

        while(time_pass.milliseconds() <= 1000-300){ //1000-200
            stage3.setPower(stage3FeedPower);
            blockShooter.setPosition(OPENSHOOTER_OPEN);
            sleep(pulseMs);

            // 3) Immediately block the next ball
            blockShooter.setPosition(OPENSHOOTER_CLOSED);

            // 4) Wait for recovery enough to avoid weak/overpowered 2nd/3rd shots.getVelocity()
            while (opModeIsActive() && shooter.getVelocity() < targetVel - lowRecoverMargin) { //shooter.getVelocity() < targetVel - lowRecoverMargin && shooter.getVelocity() > targetVel + highRecoverMargin
                telemetry.addData("Shooter Vel", "%5.2f", shooter.getVelocity());
                telemetry.update();
                sleep(loopSleepMs);
                idle();
            }
        }

        // Stop / reset
//        stage3.setPower(0);
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        startIntake(0.9, 0.3); //start intake
    }

    //running intake
    public void runIntake(double s1, double s3) {
        stage1.setPower(s1);
        // stage2.setPower(s2);
        stage3.setPower(s3);
    }

    public void shootVelocity(double base){
        shooter.setVelocity(base);
        shooter2.setVelocity((double) (base * SHOOTER_GEAR_RATIO));

    }
}
