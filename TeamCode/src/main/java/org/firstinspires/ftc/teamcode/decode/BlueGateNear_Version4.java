package org.firstinspires.ftc.teamcode.decode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BLUE NEAR 18", group = "Autonomous")
@Config
public class BlueGateNear_Version4 extends LinearOpMode {
    /* HARDWARE */
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private DcMotor stage1 = null;
    // private DcMotor stage2 = null;
    private DcMotor stage3 = null;
    private Servo blockShooter = null;
    final private double OPENSHOOTER_OPEN = 0.3; //0.19 //0.3;
    final private double OPENSHOOTER_CLOSED = 0.5; // OPENSHOOTER_OPEN + 28//0.55
    final private double SHOOTER_VELOCITY = 1925+25; //1950; //1900; //2100 //2200 //2150

    final private double SHOOTER_GEAR_RATIO = 17.0/18.0;
    /* INIT */
    private ElapsedTime runtime = new ElapsedTime();

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
                shootVelocity(SHOOTER_VELOCITY);
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

    //run the intake
    public class startIntakeAction implements Action {
        private boolean initialized = false;
        private double s1, s3;
        public startIntakeAction(double s1_in, double s3_in){
            s1 = s1_in;
            s3 = s3_in;
            blockShooter.setPosition(OPENSHOOTER_CLOSED);
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
                sleep(400-75); //550 //750 //1000 //1500

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
                sleep(350);//300 //1500

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

        Pose2d startPose = new Pose2d(-54, -54, Math.toRadians(-135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        initMotors();
        // initAprilTagAndColorBlob();

        // -------------------------------------------------------------------------
        // FIX 1: Define all poses and variables BEFORE waitForStart() so trajectories
        //         can be pre-built during the init phase. This eliminates the per-segment
        //         build-time pause that occurred at every actionBuilder boundary.
        // -------------------------------------------------------------------------
        double shootX = -14-3, shootY = -14-3; //-29, 29
        double newShootX = -11, newShootY = -20; // -20, -20//-19, -19 // -23, -23
        Pose2d shootPose = new Pose2d(shootX, shootY, Math.toRadians(-135));
        Pose2d newShootPose = new Pose2d(newShootX, newShootY, Math.toRadians(-143)); //-150 //-135

        double lastShootX = -32-3, lastShootY = -13-3; //-30, -11
        Pose2d lastShootPose = new Pose2d(lastShootX, lastShootY, Math.toRadians(-126));

        // FROM RED:  Pose2d classifierPose = new Pose2d(7.5+0.2+0.5, 64+2,  Math.toRadians(120)); //120-6 //120
        Pose2d classifierPose = new Pose2d(7.7, -64-2,  Math.toRadians(240)); //235d //-120
        double newClassifierX = 7.7+2, newClassifierY = -64;
        Pose2d newClassifierPose = new Pose2d(newClassifierX, newClassifierY, Math.toRadians(235+10+10)); //235 + 5 + 12

        double stage1power = 1.0;
        double stage3power = 0.4;

        // -------------------------------------------------------------------------
        // FIX 1 (continued): Pre-build all trajectories here, during init, so that
        //         no trajectory math runs between actions at match time.
        //
        // FIX 2: Merged separate consecutive actionBuilder calls into one builder
        //         wherever no shooting/waiting happens between them. Each merge
        //         eliminates a stop-and-restart at that segment boundary.
        //
        // FIX 3: Replaced strafeTo(8,-22) -> splineToLinearHeading pattern with
        //         setTangent() + a direct spline, avoiding the tangent discontinuity
        //         that forced velocity=0 at that join.
        //
        // FIX 4: Used afterDisp(999, ...) on trajectories that end at the shoot pose
        //         so that shootAll() fires as the robot arrives, overlapping the
        //         final deceleration with the shooter gate sequence.
        // -------------------------------------------------------------------------

        // Traj 1: start -> first shoot position
        // (strafeTo is fine here since it's the opening move with no prior tangent)
        Action traj1 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(shootX, shootY))
                .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, stage3power)))
                .build();

        Action traj2 = drive.actionBuilder(shootPose)//
                .splineToSplineHeading(new Pose2d(11, -22, Math.toRadians(-110)), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(11, -55, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToSplineHeading(newShootPose, Math.toRadians(170)) //-160, -200
                .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, stage3power))) // FIX 4
                .build();

        /*****CYCLE 1*****/
        // Traj 3: shoot pose -> classifier (FIRST TIME)
        Action traj3_toClassifier1 = drive.actionBuilder(newShootPose)
                .setTangent(Math.toRadians(15))
                .splineToLinearHeading(classifierPose, Math.toRadians(-95)) //80 //85 //95 //go into
                .strafeTo(new Vector2d(newClassifierX, newClassifierY))
                .afterDisp(999, new SequentialAction(intakeWait2()))
                .build();

        // Traj 4: classifier -> shoot pose, with shootAll() overlapping arrival (FIRST TIME)
        Action traj4_toShoot1 = drive.actionBuilder(newClassifierPose)
                .setTangent(Math.toRadians(95)) //-100 //-90
                .splineToLinearHeading(newShootPose, Math.toRadians(175)) //-160 //-155 //200 //go into
                .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, stage3power)))
                .build();

        /*****CYCLE 2*****/
        // Traj 3: shoot pose -> classifier (FIRST TIME)
        Action traj3_toClassifier2 = drive.actionBuilder(newShootPose)
                .setTangent(Math.toRadians(15))
                .splineToLinearHeading(classifierPose, Math.toRadians(-95)) //80 //85 //95 //go into
                .strafeTo(new Vector2d(newClassifierX, newClassifierY))
                .afterDisp(999, new SequentialAction(intakeWait2()))
                .build();

        // Traj 4: classifier -> shoot pose, with shootAll() overlapping arrival (FIRST TIME)
        Action traj4_toShoot2 = drive.actionBuilder(newClassifierPose)
                .setTangent(Math.toRadians(95)) //-100 //-90
                .splineToLinearHeading(newShootPose, Math.toRadians(175)) //-160 //-155 //200 //go into
                .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, stage3power))) // FIX 4
                .build();

        /*****CYCLE 3*****/
        // Traj 3: shoot pose -> classifier (FIRST TIME)
        Action traj3_toClassifier3 = drive.actionBuilder(newShootPose)
                .setTangent(Math.toRadians(15))
                .splineToLinearHeading(classifierPose, Math.toRadians(-95)) //80 //85 //95 //go into
                .strafeTo(new Vector2d(newClassifierX, newClassifierY))
                .afterDisp(999, new SequentialAction(intakeWait2()))
                .build();

        // Traj 4: classifier -> shoot pose, with shootAll() overlapping arrival (FIRST TIME)
        Action traj4_toShoot3 = drive.actionBuilder(newClassifierPose)
                .setTangent(Math.toRadians(95)) //-100 //-90
                .splineToLinearHeading(newShootPose, Math.toRadians(175)) //-160 //-155 //200 //go into
                .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, stage3power)))
                .build();

//        // Traj 5: shoot pose -> classifier (SECOND TIME)
//        Action traj5_toClassifier2 = drive.actionBuilder(newShootPose)
//                .setTangent(Math.toRadians(15)) //25 //15
//                .splineToLinearHeading(classifierPose, Math.toRadians(-95)) //85 //95 //go into
//                //Pose2d classifierPose = new Pose2d(7.5+0.2+0.5, -64-2,  Math.toRadians(240)); //240 //-120
//                //.strafeTo(new Vector2d(7.5+0.2+1.2, -64-2+2))
//                .strafeTo(new Vector2d(newClassifierX, newClassifierY))
//                .build();
//
//        // Traj 6: classifier -> shoot pose (SECOND TIME)
//        // FIX 4: afterDisp(999) fires shootAll() + startIntake as robot reaches newShootPose
//        Action traj6_toShoot2 = drive.actionBuilder(newClassifierPose)
//                .setTangent(Math.toRadians(95)) //-95 //-90
//                .splineToLinearHeading(newShootPose, Math.toRadians(175)) //-155 //200 //go into
//                .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, stage3power))) // FIX 4
//                .build();
//
//        // Traj 7: shoot pose -> classifier (THIRD TIME)
//        Action traj7_toClassifier3 = drive.actionBuilder(newShootPose)
//                .setTangent(Math.toRadians(15)) //25 //15
//                .splineToLinearHeading(classifierPose, Math.toRadians(-95)) //85 //95 //go into
//                //Pose2d classifierPose = new Pose2d(7.5+0.2+0.5, -64-2,  Math.toRadians(240)); //240 //-120
//                //.strafeTo(new Vector2d(7.5+0.2+1.2, -64-2+2))
//                .strafeTo(new Vector2d(newClassifierX, newClassifierY))
//                .build();
//
//        // Traj 8: classifier -> shoot pose (THIRD TIME)
//        // FIX 4: afterDisp(999) fires shootAll() + startIntake as robot reaches newShootPose
//        Action traj8_toShoot3 = drive.actionBuilder(newClassifierPose)
//                .setTangent(Math.toRadians(95)) //-95 //-90
//                .splineToLinearHeading(newShootPose, Math.toRadians(175)) //-155 //200 //go into
//                .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, stage3power))) // FIX 4
//                .build();


        // Traj 5: shoot pose -> inner balls -> last shoot pose
        Action traj5_innerBalls = drive.actionBuilder(newShootPose)
                .setTangent(Math.toRadians(-145))//-100)) //45 //60
                .splineToSplineHeading(new Pose2d(-17, -30,  Math.toRadians(-90)), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-17, -52, Math.toRadians(-90)), Math.toRadians(-90))
                //go back to shooting
                .splineToSplineHeading(lastShootPose, Math.toRadians(-225)) //-135 //go into
                .afterDisp(999, shootAll()) // FIX 4
                .build();

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

        while (opModeIsActive()){
            telemetry.addData("Shooter Velocity", shooter.getVelocity());
            telemetry.update();
            //*
            try {
                Actions.runBlocking(
                        new SequentialAction(
                                startShooter(),
                                startIntake(stage1power, stage3power),

                                //shoot preloads
                                //go back, shoot
                                traj1,

                                //intake middle row, go back, and shoot
                                traj2,

                                //CLASSIFER CYCLES
                                //cycle 1
                                traj3_toClassifier1,
                                traj4_toShoot1,

                                //cycle 2
                                traj3_toClassifier2,
                                traj4_toShoot2,

                                //cycle 3
                                traj3_toClassifier3,
                                traj4_toShoot3,

                                //get first row, shoot
                                traj5_innerBalls

                        )
                );
                telemetry.addData("Trajectory", "Executed Successfully");
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }
           /*
           //no trajectory code

            try {
                Actions.runBlocking(
                        new SequentialAction(
                                startShooter(),
                                startIntake(stage1power, 0.3),
                                //go to shooting place
                                drive.actionBuilder(startPose)
                                        .strafeTo(new Vector2d(shootX, shootY))
                                        .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, 0.3)))
                                        .build(),

                                //get the middle row balls
                                drive.actionBuilder(newShootPose)
                                        .setTangent(Math.toRadians(3))
                                        .splineToSplineHeading(new Pose2d(7.5, -29, Math.toRadians(-80)), Math.toRadians(-50))
                                        .splineToLinearHeading(new Pose2d(5.0, -55, Math.toRadians(-110)), Math.toRadians(-102))
                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(newShootPose, Math.toRadians(170))
                                        .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, 0.3)))
                                        .build(),

                                //----------FIRST TIME at classifier
                                drive.actionBuilder(newShootPose)
                                        .setTangent(Math.toRadians(15))
                                        .splineToLinearHeading(classifierPose, Math.toRadians(-95))
                                        .strafeTo(new Vector2d(newClassifierX, newClassifierY))
                                        .build(),
                                intakeWait1(),
                                drive.actionBuilder(newClassifierPose)
                                        .setTangent(Math.toRadians(95))
                                        .splineToLinearHeading(newShootPose, Math.toRadians(175))
                                        .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, 0.3)))
                                        .build(),

                                //--------SECOND TIME at classifier
                                drive.actionBuilder(newShootPose)
                                        .setTangent(Math.toRadians(15))
                                        .splineToLinearHeading(classifierPose, Math.toRadians(-95))
                                        .strafeTo(new Vector2d(newClassifierX, newClassifierY))
                                        .build(),
                                intakeWait2(),
                                drive.actionBuilder(newClassifierPose)
                                        .setTangent(Math.toRadians(95))
                                        .splineToLinearHeading(newShootPose, Math.toRadians(175))
                                        .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, 0.3)))
                                        .build(),

                                //--------THIRD TIME at classifier
                                drive.actionBuilder(newShootPose)
                                        .setTangent(Math.toRadians(15))
                                        .splineToLinearHeading(classifierPose, Math.toRadians(-95))
                                        .strafeTo(new Vector2d(newClassifierX, newClassifierY))
                                        .build(),
                                intakeWait2(),
                                drive.actionBuilder(newClassifierPose)
                                        .setTangent(Math.toRadians(95))
                                        .splineToLinearHeading(newShootPose, Math.toRadians(175))
                                        .afterDisp(999, new SequentialAction(shootAll(), startIntake(stage1power, 0.3)))
                                        .build(),

                                //get the inner most 3 balls
                                drive.actionBuilder(newShootPose)
                                        .setTangent(Math.toRadians(-100))
                                        .splineToSplineHeading(new Pose2d(-12, -36, Math.toRadians(-85)), Math.toRadians(-85))
                                        .splineToLinearHeading(new Pose2d(-22, -53-4, Math.toRadians(-100)), Math.toRadians(-115))
                                        .splineToSplineHeading(lastShootPose, Math.toRadians(-225))
                                        .afterDisp(999, shootAll())
                                        .build()
                        )
                );
                telemetry.addData("Trajectory", "Executed Successfully");
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }
            //*/
            blockShooter.setPosition(OPENSHOOTER_CLOSED);
            break; ///quit the Opmode loop --> it won't go back to the top
        }
    }
    private void initMotors(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).

        //cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        // leftDist  = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        // rightDist = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        //1. need initial the shooter, stage1, 2, 3, servo
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "topShooterMotor");
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
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        stage1.setDirection(DcMotor.Direction.REVERSE);
        stage3.setDirection(DcMotor.Direction.REVERSE);
        blockShooter.setDirection(Servo.Direction.REVERSE); //Do we really need this?
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shootN(int count) {
        //*
        final double targetVel = SHOOTER_VELOCITY + 100; //close = 2200. far = 2500.   // same units you use in setVelocity/getVelocity
        final double lowRecoverMargin = 100; //100;
        final double stage3FeedPower = 0.6; //tune down if multiple balls sneak
        final double stage3HoldPower = 0.0;

        startIntake(1.0, 0.5); //start intake to move thing up

        final int pulseMs = 250; //180;//400;//130;
        final int loopSleepMs = 15;

        //spin up
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        shootVelocity(targetVel);
        stage3.setPower(stage3HoldPower);

        // sleep(600);
        ElapsedTime time_pass = new ElapsedTime();
        time_pass.reset();

        while(time_pass.milliseconds() <= 1000-200){ //1000-300
            // 4) Wait for recovery enough to avoid weak/overpowered 2nd/3rd shots.getVelocity()
            while (opModeIsActive() && shooter.getVelocity() < targetVel - lowRecoverMargin) { //shooter.getVelocity() < targetVel - lowRecoverMargin && shooter.getVelocity() > targetVel + highRecoverMargin
                sleep(loopSleepMs);
                idle();
            }

            stage3.setPower(stage3FeedPower);
            blockShooter.setPosition(OPENSHOOTER_OPEN);
            sleep(pulseMs);

            // 3) Immediately block the next ball
            blockShooter.setPosition(OPENSHOOTER_CLOSED);
        }

        // Stop / reset
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        startIntake(1.0, 0.5); //start intake
        /*
        final double targetVel = SHOOTER_VELOCITY + 200; //close = 2200. far = 2500.   // same units you use in setVelocity/getVelocity
        final double stage3FeedPower = 0.8;    //tune down if multiple balls sneak
        startIntake(0.9, 0.5); //start intake
        final int pulseMs = 700; //180;//400;//130;
        // Spin up
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        shootVelocity(targetVel);

        stage3.setPower(stage3FeedPower);
        blockShooter.setPosition(OPENSHOOTER_OPEN);
        sleep(pulseMs);


        startIntake(1.0, 0.5); //start intake
        //blockShooter.setPosition(OPENSHOOTER_CLOSED);
        //*/

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