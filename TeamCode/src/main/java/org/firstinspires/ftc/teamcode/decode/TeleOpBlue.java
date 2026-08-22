/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.decode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * V2 - THIS VERSION ADD THE PINPOINT
 * V3 - This version do the yaw turn first
 * V4 - add ColorBlob
 * V5 - Add B to breakaway from obstacles
 * V6 - Add Intake
 */

@TeleOp(name="TeleOp Blue", group = "Concept")
//@Disabled
public class TeleOpBlue extends LinearOpMode
{
    // Adjust these numbers to suit your robot. Should be from 30 - 55 inches
    final double DESIRED_DISTANCE = 40;//35;//45;//12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.4;//0.2;//0.10;//0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.06;//0.03; //0.03;//0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   = 0.06;// 0.03;//0.04;//0.02  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.9;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.9;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.9;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor frontLeftDrive = null;  //  Used to control the left front drive wheel
    private DcMotor frontRightDrive = null;  //  Used to control the right front drive wheel
    private DcMotor backLeftDrive = null;  //  Used to control the left back drive wheel
    private DcMotor backRightDrive = null;  //  Used to control the right back drive wheel
    private DcMotorEx shooter = null;
    private DcMotorEx shooter2 = null;
    private DcMotor stage1 = null;
    //    private DcMotor stage2 = null;
    private DcMotorEx stage3 = null;
    private Servo blockShooter = null;
    final private double OPENSHOOTER_OPEN = 0.3; // 0.8
    final private double OPENSHOOTER_CLOSED = 0.5; //1.0;//OPENSHOOTER_OPEN + 28;//0.55

    private Servo cameraServo = null;
    final private double CAMERASERVO_HIGH = 0.49;//0.55;
    //final private double CAMERASERVO_LOW = 0.68;
    private double SHOOTER_VELOCITY = 2500;//4800;//5000;

    final private double SHOOTER_GEAR_RATIO = 17.0/18.0; //bottom / top
    boolean intakeMode = false;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 20;//20;//24;// -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private Position cameraPosition = //new Position(DistanceUnit.INCH, 0, 0, 0, 0);
            //new Position(DistanceUnit.INCH, -4, -7, 15, 0);
            new Position(DistanceUnit.INCH, 0, 5, 13.5, 0); //middle, 5 inch forward, 13.5 height
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -75, 0, 0);//-90
    GoBildaPinpointDriver pinpoint;
    ///
    enum LbState { IDLE, ALIGN, TRANSLATE }
    private LbState lbState = LbState.IDLE;
    private int yawStableCount = 0;
    ///
    private ColorBlobLocatorProcessor colorLocator;
    //obstacle avoidance
    private double rangeError = 0; //desiredTag.ftcPose.range;
    private double headingError =0 ;// desiredTag.ftcPose.bearing;
    private double yawError = 0; //= desiredTag.ftcPose.yaw;


    @Override public void runOpMode()
    {
        /*MORE PARAMETER SETTINGS*/
        //desired shoot location, when tag view is not available - ie. robot will blindly aim to return to this location
        double desired_x = 0, desired_y, desired_yaw; // FIELD: x=right, y=forward, deg (0°=+Y)
        double far_x, far_y, far_yaw;
        double latch_x = 0, latch_y, latch_yaw; //0, 50
        double park_x, park_y, park_yaw;
        if (DESIRED_TAG_ID == 20) {
            //desired_x = -30; desired_y =  30; desired_yaw =  45;
            //shoot close
            desired_x = -23; desired_y =  -23; desired_yaw =  -135; //corresponding to DESIRED DISTANCE 50 -- NEED TO CHeck the yaw
            //clear classifier
            latch_x = -8; latch_y = -66; latch_yaw = -120; //0, 50, 90
            //parking position
            park_x = -38.5; park_y = -35; park_yaw = -90;
            //shooting from far
            far_x = -48; far_y = 0; far_yaw = -150; //far shooting locations
            //yaw okay around 155, just a little close to the left side of the goal
        }

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        double stage1_power = 0.8;//0.5
        double stage3_power = 0.3;

        // Initialize the Apriltag Detection process
        initAprilTagAndColorBlob();
        //initialize pinpoint device
        initPinpoint();
        //init wheel
        initDriveMotors();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();
        boolean aprilTagMode = false;
        boolean blobMode     = false;

        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        boolean lastYState = false;  // The previous state of the Y button
        boolean shooting = false;

        cameraServo.setPosition(CAMERASERVO_HIGH);

        //double pos= CAMERASERVO_LOW;//TEST

        shootVelocity(SHOOTER_VELOCITY);
        while (opModeIsActive())
        {
            telemetry.addData("velocity (top)", shooter2.getVelocity());
            telemetry.addData("velocity (bottom)", shooter.getVelocity());


            // set camera exposure
            if (gamepad1.left_bumper && !aprilTagMode) {
                aprilTagMode = true;
                blobMode = false;
                setManualExposure(6, 250);
            }
            if ((gamepad1.right_trigger > 0.5) && !blobMode) {
                aprilTagMode = false;
                blobMode = true;
                setBlobExposureAuto();
            }
            //set camera position
            if (gamepad1.left_bumper) {
                cameraServo.setPosition(CAMERASERVO_HIGH);
            }
            /**************************************************************************************/
            targetFound = false;
            desiredTag  = null;
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        targetFound = true;                         // Yes, we want to use this tag.
                        desiredTag = detection;
                        break;  // don't look any further.
                    }else{
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        desiredTag = null;
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", desiredTag.robotPose.getPosition().x, desiredTag.robotPose.getPosition().y,desiredTag.robotPose.getPosition().z));

                telemetry.addData("yawError", yawError);
                telemetry.addData("headingError", headingError);
                telemetry.addData("rangeError", rangeError);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }
            /**************************************************************************************/
            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper) {
                if (targetFound) {

                    final double[] yawRange = new double[] {0,15};// 0, 25degrees
                    final double[] distanceRange = new double[] {55,70/*65*/}; //50, 55 //45, 65 inches
                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    rangeError   = (desiredTag.ftcPose.range);
                    headingError = desiredTag.ftcPose.bearing;
                    yawError     = desiredTag.ftcPose.yaw;
                    // Use the speed and turn "gains" to calculate how we want the robot to move.

                    //stop driving if in range
                    if ((rangeError>distanceRange[0] && rangeError<distanceRange[1])) {
                        drive = 0;
                    } else {
                        drive   = Range.clip((rangeError - (distanceRange[0] + distanceRange[1])/2)*SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    }

                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                    //angle flexibility
                    if (!(yawError>yawRange[0] && yawError<yawRange[1])) {
                        strafe = Range.clip(-(yawError - (yawRange[0] + yawRange[1])/2) * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    } else strafe = 0;

                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                    // Update robot location according to tag - reset pinpoint position
                    if ((yawError < 15) && (rangeError < 70)) {
                        // update the position according to localization, based on tag -- this is field coordinate
                        Position rp = desiredTag.robotPose.getPosition();
                        YawPitchRollAngles ro = desiredTag.robotPose.getOrientation();
                        double x_fwd    = fieldX_to_pfwd(rp.x, rp.y);
                        double y_left   = fieldY_to_pleft(rp.x, rp.y);
                        double h_rr     = ro.getYaw(AngleUnit.DEGREES);
                        //double h_rr  = heading_field_to_rr(ro.getYaw(AngleUnit.DEGREES));
                        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x_fwd, y_left, AngleUnit.DEGREES, h_rr));
                        telemetry.addData("UPDATE pinpoint from", "rp_x %5.2f, rp_y %5.2f, ro_yaw %5.2f ", rp.x, rp.y, ro.getYaw(AngleUnit.DEGREES));
                    }
                } else {
                    // if left bumper is pressed, but no tag in sight -- then move according to localization
                    DriveCommand cmd = drivePinpoint( desired_x, desired_y, desired_yaw); //result always valid
                    drive = cmd.drive;
                    strafe = cmd.strafe;
                    turn = cmd.turn;
                }
            } else
                //if right bumper is pressed -> go to the location to release the latch
                if (gamepad1.right_bumper){
                    DriveCommand cmd = drivePinpoint( latch_x, latch_y, latch_yaw); //result always valid
                    drive = cmd.drive;
                    strafe = cmd.strafe;
                    turn = cmd.turn;
                } else {
                    //if left trigger is press and final 20 second
                    if (gamepad1.left_trigger > 0.5) {
                        DriveCommand cmd = drivePinpoint(park_x, park_y, park_yaw); //result always valid
                        drive = cmd.drive;
                        strafe = cmd.strafe;
                        turn = cmd.turn;
                    }
                    else if (gamepad1.right_trigger > 0.5) {
                            DriveCommand cmd = drivePinpoint(far_x, far_y, far_yaw); //result always valid
                            drive = cmd.drive;
                            strafe = cmd.strafe;
                            turn = cmd.turn;
                    } else {
                        // LB released → reset for next run
                        lbState = LbState.IDLE;
                        yawStableCount = 0;
                        // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                        drive = -gamepad1.left_stick_y / 1.5;  // Reduce drive rate to 50%.
                        strafe = -gamepad1.left_stick_x / 1.5;  // Reduce strafe rate to 50%.
                        turn = -gamepad1.right_stick_x / 2.0;  // Reduce turn rate to 33%.


                        telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                    }
                }
            // if right bumper is press -> and there is purple ball in sights -> turn and drive toward it
            if (gamepad1.right_trigger > 0.5) {
                DriveCommand cmd = autoAcquirePurple();
                if (cmd.validBlob) {
                    drive = cmd.drive;
                    strafe = cmd.strafe;
                    turn = cmd.turn;
                }
            }

            telemetry.update();

            if (gamepad2.left_bumper && !shooting){
                moveRobot(0, 0, 0);
                shootBasedOnDistance();
                moveRobot(0, 0, 0);
                shooting = false;
            }

            if (gamepad2.right_bumper && !shooting){
                moveRobot(0, 0, 0);
                shootBasedOnDistance();
                moveRobot(0, 0, 0);
                shooting = false;

            }

            /**************************************************************************************/
            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            //Apply power to stage1, stage2, stage3
            runIntake(stage1_power, stage3_power);
            sleep(10);
        }
    }
    /**************************************************************************************/
    //Move robot according to desired axes motions: Positive X is forward,  Positive Y is strafe left, Positive Yaw is counter-clockwise

    //OLD SHOOTING FUNCTION
    public void shoot(int totalMs) {
        moveRobot(0, 0, 0); // stops robot in place
        double targetVel = SHOOTER_VELOCITY; //close = 2200. far = 2500.   // same units you use in setVelocity/getVelocity
        final double recoverMargin = 100+100; //100;      // tune (smaller than dropMargin)
        final double stage3FeedPower = 0.6; //0.6 // tune down if multiple balls sneak
        final double stage3HoldPower = 0.0;

        intakeMode = true;

        final double GATE_HOLD = OPENSHOOTER_CLOSED;   // you may want a slightly-open "hold" instead
        final double GATE_PULSE_OPEN = OPENSHOOTER_OPEN; // tune so 1 ball passes, not 2


        final int pulseMs = 130;
        final int stableizeMs = 300;             // startup time to accelorate before shooting
        final int loopSleepMs = 15;

        // Spin up
        blockShooter.setPosition(GATE_HOLD);
        shooter.setVelocity(targetVel); //target Velocity
        stage3.setPower(stage3HoldPower);

        ElapsedTime time_pass = new ElapsedTime();
        time_pass.reset();
        boolean high = true;
        //1500 - stabilize Ms
        while(time_pass.milliseconds() <= 1800 ){

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

        telemetry.addData("Ewan", "End Shooting");
        telemetry.update();

        // Stop / reset
//        stage3.setPower(0);
        blockShooter.setPosition(GATE_HOLD);
//        shooter.setVelocity(0);
        intakeMode = true;
    }
    //calculate how much vel based on distance
    public double distanceToVel(double d) {
        // Convert from distance to velocity in a linear manner
        final double DMIN = 36.0;
        final double DMAX = 110;//110.0;
        final double VEL_MIN = 1900;//1700.0;
        final double VEL_MAX = 2400;//2500.0; //2550.0

        // Clamp distance to [DMIN, DMAX]
        if (d <= DMIN) {
            return VEL_MIN;
        }
        if (d >= DMAX){
            return VEL_MAX;
        }

        // Linear interpolation
        //t is given distance
        double t = (d - DMIN) / (DMAX - DMIN);          // t in [0, 1]
        double vel = VEL_MIN + t * (VEL_MAX - VEL_MIN); // vel in [VEL_MIN, VEL_MAX]

        return vel;
    }
    public void shootBasedOnDistance() {
        /*
        1. read camera distance + pose
        2. move robot a lit bit to aim exactly at the April TAG -
        [REFINE LATER]
        3. set velocity based on distance
        4. shoot
        * */
        double targetVel = SHOOTER_VELOCITY;

        //check for desired tag
        if (desiredTag != null) {
            rangeError = desiredTag.ftcPose.range;
            headingError = desiredTag.ftcPose.bearing;
            yawError = desiredTag.ftcPose.yaw;
            quickTurnToTagBearing(DESIRED_TAG_ID);
            targetVel = distanceToVel(rangeError);
        }
        /**SHOOTING**/
        double    stage3FeedPower = 0.6;
        double    lowRecoverMargin = 150; //100; //200
        long      loopSleepMs = 15;
        double    totalShootingTime = 700; //1500-600;//1000 - 300; //1000-200
        long      pulseMs = 250; //250
        // ADJUST THE Stage3 powemanually
        //*
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        double cur_left = p.getY(DistanceUnit.INCH);           // +Y left
        if (cur_left < -23){
            stage3FeedPower = 0.2;
            lowRecoverMargin = 200;
            runIntake(1.0, 0.3); //1.0, 0.2
            totalShootingTime = 2200;
            targetVel = 2500;
        }
        //*/

        //prepare all the shooter stuff
        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        shootVelocity(targetVel);
        stage3.setPower(stage3FeedPower);

        ElapsedTime time_pass = new ElapsedTime();
        time_pass.reset();
        while(time_pass.milliseconds() <= totalShootingTime){
            //wait
            while (opModeIsActive() && (shooter.getVelocity() < targetVel - lowRecoverMargin)
                    //&& (time_pass.milliseconds() <= totalShootingTime)
            ) {
                stage3.setPower(0); //don't prematurely kick ball
                sleep(loopSleepMs);
                idle();
            }
            stage3.setPower(stage3FeedPower);
            blockShooter.setPosition(OPENSHOOTER_OPEN);
            sleep(pulseMs);   // 3) Immediately block the next ball -- but open right away if the speed is ok

            blockShooter.setPosition(OPENSHOOTER_CLOSED);
        }

        blockShooter.setPosition(OPENSHOOTER_CLOSED);
        runIntake(0.9, 0.5); //0.8, 0.6
    }
    /**
     * Quick turn-to-yaw using AprilTag. Assumes tag is usually visible.
     * If tag is not seen at any iteration, this function exits immediately (so you can move on).
     * Returns true if aligned, false otherwise.
     */
    //correction while shooting method
    public boolean quickTurnToTagBearing(int DESIRED_TAG_ID) {
        final double TOLERANCE_DEG = 2.5; // 1.5;//3.0;   // within +/- 3 degrees
        final double kP = 0.02;             // tune
        final double MIN_POWER = 0.12;      // tune
        final double MAX_POWER = 0.6;      // tune
        final double TIMEOUT_S = 1.5;       // quick timeout

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < TIMEOUT_S) {
            // Find desired tag
            desiredTag = null;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null &&
                        ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                    desiredTag = detection;
                    break;
                }
            }
            // If we don't see the tag, stop and move on
            if (desiredTag == null) {
                setTurnPower(0.0);
                return false;
            }
            //recalculate errors
            rangeError   = desiredTag.ftcPose.range;
            headingError = desiredTag.ftcPose.bearing;
            yawError     = desiredTag.ftcPose.yaw;
            // Done, permission to shoot now
            if (Math.abs(headingError) <= TOLERANCE_DEG) {
                setTurnPower(0.0);
                return true;
            }
            // P turn
            double turnPower = kP * headingError;
            // minimum power
            if (Math.abs(turnPower) < MIN_POWER) {
                turnPower = Math.copySign(MIN_POWER, turnPower);
            }
            // cap
            turnPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, turnPower));
            setTurnPower(turnPower);
            idle();
        }

        // timeout
        setTurnPower(0.0);
        return false;
    }
    /** In-place turn motor powers. If direction is wrong, flip signs. */
    private void setTurnPower(double turnPower) {
        frontLeftDrive.setPower(-turnPower);
        backLeftDrive.setPower(-turnPower);
        frontRightDrive.setPower(turnPower);
        backRightDrive.setPower(turnPower);
    }
    public void moveRobot(double x, double y, double yaw) {
        //mecanum drive
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
    //Init motors
    private void initDriveMotors(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftWheel"); //frontLeftWheel
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightWheel");//frontRightWheel
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftWheel"); //backLeftWheel
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightWheel");//backRightWheel

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
//        leftDist  = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
//        rightDist = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        //1. need initial the shooter, stage1, 2, 3, servo
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "topShooterMotor");

        stage1 = hardwareMap.get(DcMotor.class, "stage1");
//        stage2 = hardwareMap.get(DcMotor.class, "stage2");
        stage3 = hardwareMap.get(DcMotorEx.class, "stage3");
        blockShooter = hardwareMap.get(Servo.class, "blockShooter");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.setDirection(DcMotor.Direction.REVERSE);
        stage1.setDirection(DcMotor.Direction.REVERSE);
//        stage2.setDirection(DcMotor.Direction.REVERSE);
        stage3.setDirection(DcMotor.Direction.REVERSE);
        blockShooter.setDirection(Servo.Direction.REVERSE); //Do we really need this?

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        //pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setOffsets(180, -160.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        //pinpoint.setOffsets(-160, -180.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

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
    /**************************************************************************************/
    // HELPER
    // Field (X right, Y fwd, 0° = +Y)  ->  Pinpoint/RR (X fwd, Y left, 0° = +X)
    static double fieldX_to_pfwd(double x_field, double y_field) { return y_field; }   // forward
    static double fieldY_to_pleft(double x_field, double y_field) { return -x_field; } // left
    static double heading_field_to_rr(double heading_field_deg) {  // wrap to (-180,180]
        return wrapDeg(heading_field_deg - 90.0);
        //return heading_field_deg;
    }
    static double wrapDeg(double a) {
        a = (a + 180.0) % 360.0;
        if (a < 0) a += 360.0;
        return a - 180.0;
    }
    // BLOB related
    // Put this inside your OpMode class (but outside any methods)
    private static class DriveCommand {
        public final double drive;
        public final double strafe;
        public final double turn;
        public final boolean validBlob;
        public DriveCommand(double drive, double strafe, double turn, boolean v) {
            this.drive  = drive;
            this.strafe = strafe;
            this.turn   = turn;
            this.validBlob = v;
        }
    }
    private DriveCommand drivePinpoint(double desired_x, double desired_y, double desired_yaw){
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        double goal_fwd = fieldX_to_pfwd(desired_x, desired_y);   // = y_field
        double goal_left = fieldY_to_pleft(desired_x, desired_y);  // = -x_field
        double goal_headRR = heading_field_to_rr(desired_yaw);       // = field - 90

        // --- 2) Current pose from Pinpoint (RR frame) ---
        pinpoint.update();
        Pose2D p = pinpoint.getPosition();
        double cur_fwd = p.getX(DistanceUnit.INCH);           // +X forward
        double cur_left = p.getY(DistanceUnit.INCH);           // +Y left
        double cur_head = p.getHeading(AngleUnit.DEGREES);     // 0° = +X

        //telemetry.addData("goal: ","goal_fwd %.2f /goal_left %.2f /goal_headRR %+.2f/ ", goal_fwd, goal_left, goal_headRR);
        //telemetry.addData("cur: ","cur_fwd %.2f /cur_left %.2f /cur_head %+.2f/ ", cur_fwd, cur_left, cur_head);
        // --- 3) Errors / geometry ---
        double dF = goal_fwd - cur_fwd;
        double dL = goal_left - cur_left;
        double range = Math.hypot(dF, dL);

        // Bearing of goal relative to robot forward; + = left (matches ftcPose.bearing)
        double bearingDeg = Math.toDegrees(Math.atan2(dL, dF));
        double yawErr = wrapDeg(goal_headRR - cur_head);         // align-to-desired heading
        double bearingRel = wrapDeg(bearingDeg - cur_head);          // bearing in robot frame

        // --- 4) Params ---
        final double YAW_TOL_DEG = 3.0;
        final int YAW_STABLE_LOOPS = 3;     // debounce: hold tol for N loops
        final double MIN_SPIN = 0.14;   // <-- new: tune ~0.10–0.18
        // --- 5) State machine (ALIGN → TRANSLATE) while LB is held ---
        if (lbState == LbState.IDLE) {
            lbState = LbState.ALIGN;
            yawStableCount = 0;
        }
        if (lbState == LbState.ALIGN) {
            // Spin in place to desired yaw


            turn = Range.clip(TURN_GAIN * yawErr, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            // gate translation during alignment
            drive = 0;
            strafe = 0;
            //g little bit nudge
            if (Math.abs(yawErr) > YAW_TOL_DEG && Math.abs(turn) < MIN_SPIN) {
                turn = Math.copySign(MIN_SPIN, yawErr);
            }
            if (Math.abs(yawErr) <= YAW_TOL_DEG) {
                if (++yawStableCount >= YAW_STABLE_LOOPS) {
                    lbState = LbState.TRANSLATE;  // yaw locked
                }
            } else {
                yawStableCount = 0;
            }
        } else { // TRANSLATE
            // Hold yaw fixed: no more turning
            turn = 0;
            // ---- Option B: decompose motion along robot axes toward the target ----
            double speed = Range.clip(SPEED_GAIN * range, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double brad = Math.toRadians(bearingRel);
            drive = speed * Math.cos(brad);  // forward
            strafe = speed * Math.sin(brad);  // left
            // (Optional tiny steering to keep path tight: leave at 0 if you truly want to freeze yaw)
            //turn = Range.clip(0.02 * wrapDeg(bearingDeg - cur_head), -0.2, 0.2);
            turn = Range.clip(TURN_GAIN * yawErr, -0.28, 0.8);
        }
        telemetry.addData("LB", lbState);
        telemetry.addData("Yaw cur/goal/err/yawStableCount", "%.2f / %.2f / %+.2f/ %d", cur_head, goal_headRR, yawErr, yawStableCount);
        telemetry.addData("Range / bearing", "%.2f in / %+.2f°", range, bearingDeg);
        telemetry.addData("Command drive/strife/turn ", "%.2f / %.2f / %.2f", drive, strafe, turn);

        return new DriveCommand(drive, strafe, turn, true);
        //return new DriveCommand(0, 0, 0, true);
    }
    private DriveCommand autoAcquirePurple() {

        // --- Blob-based constants (kept local so it's easy to reuse elsewhere) ---

        // Image size (change if your stream changes)
        final int IMG_WIDTH  = 640;
        final int IMG_HEIGHT = 480;

        // Camera intrinsics for your config (C270 @ 640x480 in this example)
        final double FX = 822.317;     // focal length in pixels (x)
        final double CX = 319.495;     // principal point x in pixels

        // Artifact geometry (DECODE ball)
        final double ARTIFACT_DIAMETER_IN = 5.0;              // adjust if you want
        final double ARTIFACT_RADIUS_IN   = ARTIFACT_DIAMETER_IN / 2.0;

        // Desired distance
        final double TARGET_RANGE_IN = 12;//24.0;                  // 2 ft

        // Control gains
        final double K_DRIVE = 0.03;                          // power per inch of range error
        final double K_TURN  = 0.02;                          // power per degree of angle error

        final double MAX_DRIVE = 0.5;
        final double MAX_TURN  = 0.4;

        // Deadbands
        final double RANGE_TOL_IN  = 1.0;                     // stop driving when |error| < 1"
        final double ANGLE_TOL_DEG = 1.0;                     // stop turning when |error| < 1°

        // --- Default command: do nothing ---
        double driveCmd  = 0.0;
        double strafeCmd = 0.0;
        double turnCmd   = 0.0;

        // --- Get blobs & pick largest ---
        /**************************************************************************************/
        //Detect Color Blob
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        /*
         * The list of Blobs can be filtered to remove unwanted Blobs.
         *   Note:  All contours will be still displayed on the Stream Preview, but only those
         *          that satisfy the filter conditions will remain in the current list of
         *          "blobs".  Multiple filters may be used.
         *
         * To perform a filter
         *   ColorBlobLocatorProcessor.Util.filterByCriteria(criteria, minValue, maxValue, blobs);
         *
         * The following criteria are currently supported.
         *
         * ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA
         *   A Blob's area is the number of pixels contained within the Contour.  Filter out any
         *   that are too big or small. Start with a large range and then refine the range based
         *   on the likely size of the desired object in the viewfinder.
         *
         * ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY
         *   A blob's density is an indication of how "full" the contour is.
         *   If you put a rubber band around the contour you would get the "Convex Hull" of the
         *   contour. The density is the ratio of Contour-area to Convex Hull-area.
         *
         * ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO
         *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
         *   A perfect Square has an aspect ratio of 1.  All others are > 1
         *
         * ColorBlobLocatorProcessor.BlobCriteria.BY_ARC_LENGTH
         *   A blob's arc length is the perimeter of the blob.
         *   This can be used in conjunction with an area filter to detect oddly shaped blobs.
         *
         * ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY
         *   A blob's circularity is how circular it is based on the known area and arc length.
         *   A perfect circle has a circularity of 1.  All others are < 1
         */
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                //50, 20000, blobs);  // filter out very small blobs.
                50, 200000, blobs);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                //0.6, 1, blobs);     /* filter out non-circular blobs.
                0.5, 1, blobs);     /* filter out non-circular blobs.
         * NOTE: You may want to adjust the minimum value depending on your use case.
         * Circularity values will be affected by shadows, and will therefore vary based
         * on the location of the camera on your robot and venue lighting. It is strongly
         * encouraged to test your vision on the competition field if your event allows
         * sensor calibration time.
         */
        /*
         * The list of Blobs can be sorted using the same Blob attributes as listed above.
         * No more than one sort call should be made.  Sorting can use ascending or descending order.
         * Here is an example.:
         *   ColorBlobLocatorProcessor.Util.sortByCriteria(
         *      ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
         */
        ColorBlobLocatorProcessor.Util.sortByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                SortOrder.DESCENDING, blobs);

        telemetry.addLine("Circularity Radius Center");
        // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            Circle circleFit = b.getCircle();
            telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
                    b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
        }

        ColorBlobLocatorProcessor.Blob bestBlob = null;
        if (!blobs.isEmpty()) {
            bestBlob = blobs.get(0);
        }

        if (bestBlob != null) {
            Circle circle = bestBlob.getCircle();

            if (circle != null) {
                double centerX_px = circle.getX();
                double centerY_px = circle.getY();
                double radius_px  = circle.getRadius();
                // --- Check if blob is partially outside image (clipped) ---
                boolean isPartial = (centerX_px - radius_px <= 0) ||
                        (centerX_px + radius_px >= IMG_WIDTH) ||
                        (centerY_px - radius_px <= 0) ||
                        (centerY_px + radius_px >= IMG_HEIGHT);
                // --- Horizontal alignment: compute angle error from image center ---
                double dx_px = centerX_px - CX;              // + if blob is to the right
                double angleErrRad = Math.atan2(dx_px, FX);  // small-angle approx
                double angleErrDeg = Math.toDegrees(angleErrRad);
                turnCmd = Range.clip(-angleErrDeg * K_TURN, -MAX_TURN, MAX_TURN);

                if (Math.abs(angleErrDeg) < ANGLE_TOL_DEG) {
                    turnCmd = 0;
                }
                if (isPartial) {
                    // Partial blob -> ONLY turn to center, do not trust radius for distance
                    driveCmd  = 0;
                    strafeCmd = 0;

                    telemetry.addLine("Partial blob: turning only");
                    telemetry.addData("AngleError", "%.1f", angleErrDeg);
                } else {
                    // Full blob in view -> estimate distance from radius
                    double rangeIn = FX * ARTIFACT_RADIUS_IN / radius_px;
                    double rangeError = rangeIn - TARGET_RANGE_IN;
                    driveCmd = Range.clip(rangeError * K_DRIVE, -MAX_DRIVE, MAX_DRIVE);

                    if (Math.abs(rangeError) < RANGE_TOL_IN) {
                        driveCmd = 0;
                    }

                    strafeCmd = 0; // still only drive + turn in this behavior

                    telemetry.addLine("Full blob: centering + range control");
                    telemetry.addData("RangeIn",    "%.1f", rangeIn);
                    telemetry.addData("RangeError", "%.1f", rangeError);
                    telemetry.addData("AngleError", "%.1f", angleErrDeg);
                }
            } else {
                telemetry.addLine("Blob found, but no circle fit");
            }
        } else {
            telemetry.addLine("No purple blob detected");
        }

        // Return the command vector for caller to use
        return new DriveCommand(driveCmd, strafeCmd, turnCmd, bestBlob != null);
    }

    /**
     * Computes a strafe command to dodge sideways away from closer obstacle.
     * Uses two REV 2M distance sensors at 45° left/right.
     * Returns DriveCommand(drive=0, strafe, turn=0).
     * When both sides are clear, returns near-zero strafe so caller can resume “home”.
     /**
     * Decide initial dodge direction based on current distances.
     * Returns -1 for strafe left, +1 for strafe right, or 0 for no dodge.
     */
    /*
    //NO NEED ANYMORE --> no more distance sensors
    private double chooseDodgeDirectionOnce() {
        // LOGIC HERE IS REVERSED!!!!!
        final double NEAR_THRESHOLD_IN = 18.0;  // tune as needed
//        double leftIn  = leftDist.getDistance(DistanceUnit.INCH);
//        double rightIn = rightDist.getDistance(DistanceUnit.INCH);

        if (Double.isNaN(leftIn) || Double.isInfinite(leftIn))  leftIn  = 1000;
        if (Double.isNaN(rightIn) || Double.isInfinite(rightIn)) rightIn = 1000;

        telemetry.addData("Dodge leftIn",  "%.1f", leftIn);
        telemetry.addData("Dodge rightIn", "%.1f", rightIn);

        boolean leftNear  = leftIn  < NEAR_THRESHOLD_IN;
        boolean rightNear = rightIn < NEAR_THRESHOLD_IN;

        // Simple rules:
        // - If only left is near → slide right (+1)
        // - If only right is near → slide left (-1)
        // - If both near → slide toward side with more room
        // - If neither near → no dodge (0)
        if (leftNear && !rightNear) {
            telemetry.addLine("Dodge dir chosen: RIGHT (obstacle left)");
            return +1.0;
        } else if (rightNear && !leftNear) {
            telemetry.addLine("Dodge dir chosen: LEFT (obstacle right)");
            return -1.0;
        } else if (leftNear && rightNear) {
            if (leftIn > rightIn) {
                telemetry.addLine("Dodge dir chosen: LEFT (more room left)");
                return -1.0;
            } else {
                telemetry.addLine("Dodge dir chosen: RIGHT (more room right)");
                return +1.0;
            }
        } else {
            telemetry.addLine("Dodge dir chosen: NONE (no close obstacle)");
            return 0.0;
        }
    }
    */
    /**************************************************************************************/
    //INTAKE
    public void runIntake(double stage1_power, double stage3_power){
        stage1.setPower(stage1_power);
//        stage2.setPower(stage2_power);
        stage3.setPower(stage3_power);
    }
    public void shootVelocity(double base){   //run shooters with gear ratio
        shooter.setVelocity(base);
        shooter2.setVelocity((double) (base * SHOOTER_GEAR_RATIO));

    }
}

