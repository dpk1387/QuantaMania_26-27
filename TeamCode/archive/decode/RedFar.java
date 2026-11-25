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

@Autonomous(name = "RED FAR - BOTTOM FIRST", group = "Autonomous")
@Config
public class RedFar extends LinearOpMode {
    /* HARDWARE */
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private DcMotor stage1 = null;
    // private DcMotor stage2 = null;
    private DcMotor stage3 = null;
    private Servo blockShooter = null;
    final private double OPENSHOOTER_OPEN = 0.3; //0.19 //0.3;
    final private double OPENSHOOTER_CLOSED = 0.5; // OPENSHOOTER_OPEN + 28//0.55
    final private double SHOOTER_VELOCITY = 2500; //2000 //2100 //2200 //2150
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
    // Convenience factor so you can just write startShooter() in your SequentialAction
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

    /// ***********************************************************************************
    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry.addData("Status", "Initializing...");
        telemetry.update();
        telemetry.update();

        Pose2d startPose = new Pose2d(62, 11, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        initMotors();

        double shootX = 54, shootY = 15;
        Pose2d shootPose = new Pose2d(shootX, shootY, Math.toRadians(156));

        Pose2d loadZonePose = new Pose2d(64, 64, Math.toRadians(45));
        Pose2d initialLoadZonePose = new Pose2d(44, 40, Math.toRadians(60));

        int approachingTangent = 90; //45; //0; //180

        double stage1power = 0.8;
        double stage3power = 0.4;

        // Traj 1: start -> shoot position
        Action traj1 = drive.actionBuilder(startPose)
                .splineToLinearHeading(shootPose, Math.toRadians(170))
                .afterDisp(999, new SequentialAction(shootAll()))
                .build();

        /***** CYCLE 1 *****/
        // Traj 2: shoot pose -> loading zone
        Action traj2_toLoadingZone1 = drive.actionBuilder(shootPose)
                .setTangent(Math.toRadians(90))
                //wiggle to intake more artifacts
                .splineToSplineHeading(initialLoadZonePose, Math.toRadians(170))
                .splineToLinearHeading(loadZonePose, Math.toRadians(90))
                .build();

        // Traj 3: loading zone -> shoot pose
        Action traj3_loadZoneToShoot1 = drive.actionBuilder(loadZonePose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose, Math.toRadians(90))
                .afterDisp(999, new SequentialAction(shootAll()))
                .build();

        /***** CYCLE 2 *****/
        // Traj 2: shoot pose -> loading zone
        Action traj2_toLoadingZone2 = drive.actionBuilder(shootPose)
                .setTangent(Math.toRadians(90))
                //wiggle to intake more artifacts
                .splineToSplineHeading(initialLoadZonePose, Math.toRadians(170)) //90
                .splineToLinearHeading(loadZonePose, Math.toRadians(90))
                .build();

        // Traj 3: loading zone -> shoot pose
        Action traj3_loadZoneToShoot2 = drive.actionBuilder(loadZonePose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose, Math.toRadians(90))
                .afterDisp(999, new SequentialAction(shootAll()))
                .build();

        /***** CYCLE 3 *****/
        // Traj 2: shoot pose -> loading zone
        Action traj2_toLoadingZone3 = drive.actionBuilder(shootPose)
                .setTangent(Math.toRadians(90))
                //wiggle to intake more artifacts
                .splineToSplineHeading(initialLoadZonePose, Math.toRadians(170))
                .splineToLinearHeading(loadZonePose, Math.toRadians(90))
                .build();

        // Traj 3: loading zone -> shoot pose
        Action traj3_loadZoneToShoot3 = drive.actionBuilder(loadZonePose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(shootPose, Math.toRadians(90))
                .afterDisp(999, new SequentialAction(shootAll()))
                .build();
//        // Traj 4: shoot pose -> loading zone
//        Action traj4_toLoadingZone = drive.actionBuilder(shootPose)//
//                .splineToLinearHeading(loadZonePose, Math.toRadians(approachingTangent))
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(55, 55, Math.toRadians(90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(64, 64, Math.toRadians(45)), Math.toRadians(90))
//
//                .build();


//        // Traj 5: loading zone -> shoot pose
//        Action traj5 = drive.actionBuilder(loadZonePose)
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(shootPose, Math.toRadians(90))
//                .build();

//        // Traj 6: shoot pose -> loading zone
//        Action traj6_toLoadingZone = drive.actionBuilder(shootPose)//
//                .splineToLinearHeading(loadZonePose, Math.toRadians(approachingTangent))
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(55, 55, Math.toRadians(90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(64, 64, Math.toRadians(45)), Math.toRadians(90))
//
//                .build();


//        // Traj 7: loading zone -> shoot pose
//        Action traj7 = drive.actionBuilder(loadZonePose)
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(shootPose, Math.toRadians(90))
//                .build();

        // Traj 4: shoot pose -> 3rd row -> shoot pose
        //get the artifacts from third row, go to shoot
        Action traj4_thirdRow = drive.actionBuilder(shootPose)
                //go in front of row
                .splineToSplineHeading(new Pose2d(40-4, 28-8, Math.toRadians(90)), Math.toRadians(90))
                //intake row
                .splineToLinearHeading(new Pose2d(40-4, 52 /*56*/, Math.toRadians(90)), Math.toRadians(90))
                //go back to shoot
                .splineToSplineHeading(shootPose, Math.toRadians(110))
                //shoot
                .afterDisp(999, new SequentialAction(shootAll()))
                .build();

        // Traj 5: leave launch zone for ranking point
        Action traj5_leaveLaunchZone = drive.actionBuilder(shootPose)
                .strafeTo(new Vector2d(46, 24))
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

                                //move up to shoot, go shoot, start intake
                                traj1,

                                //go to third row, intake, come back, and shoot
                                traj4_thirdRow,
                                //shootAll(),

                                //CYCLE 1
                                traj2_toLoadingZone1,
                                traj3_loadZoneToShoot1,

                                //CYCLE 2
                                traj2_toLoadingZone2,
                                traj3_loadZoneToShoot2,

                                //CYCLE 3
                                traj2_toLoadingZone3,
                                traj3_loadZoneToShoot3,

                                //leave zone
                                traj5_leaveLaunchZone
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

        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        stage1.setDirection(DcMotor.Direction.REVERSE);
        // stage2.setDirection(DcMotor.Direction.REVERSE);
        stage3.setDirection(DcMotor.Direction.REVERSE);
        blockShooter.setDirection(Servo.Direction.REVERSE); //Do we really need this?

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shootN(int count) {
        //*
        final double targetVel = SHOOTER_VELOCITY; //=2500.
        final double lowRecoverMargin = 100+150; //100;
        final double stage3FeedPower = 0.4;    //tune down if multiple balls sneak

        startIntake(0.7, 0.4); //start intake to move thing up

        final int pulseMs = 250; //250 //180;//400;//130;
        final int loopSleepMs = 15;

        // Spin up
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        shootVelocity(targetVel);
        stage3.setPower(stage3FeedPower);

        ElapsedTime time_pass = new ElapsedTime();
        time_pass.reset();

        while(time_pass.milliseconds() <=1800 /*2200*/){
            // 4) Wait for recovery enough to avoid weak/overpowered 2nd/3rd shots.getVelocity()
            while (opModeIsActive() && shooter.getVelocity() < targetVel - lowRecoverMargin) {
                stage3.setPower(0.5); //0.2
                sleep(loopSleepMs);
                idle();
            }

            stage3.setPower(stage3FeedPower);
            blockShooter.setPosition(OPENSHOOTER_OPEN);
            sleep(pulseMs);

            // 3) Immediately block the next ball
            blockShooter.setPosition(OPENSHOOTER_CLOSED);
        }

        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        startIntake(1.0, 0.3); //start intake

        /*
        final double targetVel = SHOOTER_VELOCITY + 200; //close = 2200. far = 2500.   // same units you use in setVelocity/getVelocity
        final double stage3FeedPower = 0.8;    //tune down if multiple balls sneak
        startIntake(0.9, 0.5); //start intake
        final int pulseMs = 700; //180;//400;//130;              // tune: shorter = fewer double-feeds
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