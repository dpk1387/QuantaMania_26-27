package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

@TeleOp(name="TeleOp Red")

public class TeleOpRed extends LinearOpMode
{
    private final double SPEED_GAIN = 0;
    private final double STRAFE_GAIN = 0;
    private final double TURN_GAIN = 0;

    private DcMotor frontLeftDrive = null;  //  Used to control the left front drive wheel
    private DcMotor frontRightDrive = null;  //  Used to control the right front drive wheel
    private DcMotor backLeftDrive = null;  //  Used to control the left back drive wheel
    private DcMotor backRightDrive = null;  //  Used to control the right back drive wheel

    private DcMotorEx intake1 = null;
    private DcMotorEx intake2 = null;
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private Servo turret = null;

    private Servo flowerIntake = null;
    private Servo camera = null;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private static final String RED_AUDIENCE = "RED AUDIENCE";
    private static final String RED_SCORING = "RED SCORING";

    private double SHOOTER_VELOCITY;

    // target positions
    private static double audienceX; double audienceY; double audienceYaw;

    private AprilTagClusterDetection desiredCluster = null;

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        initPinpoint();
        initDriveMotors();

        boolean targetFound = false;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Shooter Velocity", shooter.getVelocity());

            double intake1_power = 0.8;
            double intake2_power = 0.4;

            if (gamepad1.left_bumper) {

            }

            if (gamepad1.right_bumper) {
                shoot();
            }

            if (gamepad1.right_trigger_pressed) {
                runIntake(intake1_power, 0);
            }
            else if (gamepad1.left_trigger_pressed) {
                runIntake(-intake1_power,0);
            }

            if (gamepad1.a) {

            }

            if (gamepad1.b) {

            }

            if (gamepad1.x) {

            }

            if (gamepad1.y) {

            }

            double drive = gamepad1.left_stick_y / 1.25;
            double strafe = gamepad1.left_stick_x / 1.25;
            double turn = gamepad1.right_stick_x / 1.5;

            /*List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {

                if (detection instanceof AprilTagClusterDetection) {

                    AprilTagClusterDetection clusterDet =
                            (AprilTagClusterDetection) detection;

                    String clusterName = clusterDet.metadata.name;

                    if (Math.abs(detection.ftcPose.roll) < 90) {

                        targetFound = true;
                        desiredCluster = clusterDet;

                        telemetry.addData("Found Cluster", clusterName);
                        telemetry.addData("% Found",
                                clusterDet.percentClusterFound);

                        break;
                    }
                }
            }*/

            moveRobot(drive, strafe, turn);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    public void shoot() {

    }

    private static class DriveCommand {
        public final double drive;
        public final double strafe;
        public final double turn;
        public DriveCommand(double drive, double strafe, double turn) {
            this.drive  = drive;
            this.strafe = strafe;
            this.turn   = turn;
        }
    }

    /*private DriveCommand pinpoint(double targetX, double targetY, double targetYaw) {
        double drive = 0;
        double strafe = 0;
        double turn = 0;

    }*/

    private void initDriveMotors(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftWheel"); //frontLeftWheel
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightWheel");//frontRightWheel
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftWheel"); //backLeftWheel
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightWheel");//backRightWheel

        //1. need initial the shooter, intake1, 2, turret
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        turret = hardwareMap.get(Servo.class, "turret");

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        intake1.setDirection(DcMotor.Direction.REVERSE);
        intake2.setDirection(DcMotor.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initPinpoint() {
        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Configure the sensor
        configurePinpoint();
        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }
    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        //pinpoint.setOffsets( , ,DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                //GoBildaPinpointDriver.EncoderDirection.FORWARD);
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

    public void runIntake(double intake1_power, double intake2_power){
        intake1.setPower(intake1_power);
        intake2.setPower(intake2_power);
    }

    public void shootVelocity(double base){
        shooter.setVelocity(base);
        shooter2.setVelocity(base);
        //shooter2.setVelocity((double) (base * SHOOTER_GEAR_RATIO));

    }
}