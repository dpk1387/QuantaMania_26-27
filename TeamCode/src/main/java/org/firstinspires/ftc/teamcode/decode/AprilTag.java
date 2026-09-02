package org.firstinspires.ftc.teamcode.decode;

import android.util.Size;
import java.util.Locale;

//import necessary android and ftc sdk classes
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//imports ftc vision and AprilTag detection systems
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List; //used to store lists of detections
//int num = 0;
@TeleOp //marks program as teleop
@Disabled
public class AprilTag extends LinearOpMode {

    @Override //main method that runs once the OpMode started
    public void runOpMode() throws InterruptedException {
        double tagSize = 6.4375; //defines physical size of each AprilTag (inches)

        //this gets the april tags that are gonna be used on field
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "Tag20", tagSize, DistanceUnit.INCH)
                .addTag(21, "Tag21", tagSize, DistanceUnit.INCH)
                .addTag(22, "Tag22", tagSize, DistanceUnit.INCH)
                .addTag(23, "Tag23", tagSize, DistanceUnit.INCH)
                .addTag(24, "Tag24", tagSize, DistanceUnit.INCH)
                .build(); //finalize the tag library

        //this sets how the tags are detected and what visualizations are drawn on screen
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true) //draw xyz coord axes on detected tags
                .setDrawCubeProjection(true) //shows a cube overlay that represents 3d orientation
                .setDrawTagID(true) //displays the tag's numerical ID on the screen
                .setDrawTagOutline(true) //draws tag's rectangular border
                .setTagLibrary(tagLibrary) //links this processor to the custom tag library created above
                .setLensIntrinsics(629.0, 629.0, 320.0, 240.0) //lense calibration
                .build(); //build tag processor

        //the vision portal connects the camera and processors together for real-time detection
        //add apriltag processor to the vision pipeline
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor) //add created tag processor
                .setCamera(hardwareMap.get(WebcamName.class,"Webcam 1")) //name camera
                .setCameraResolution(new Size(640,480)) //resolution
                .enableLiveView(true) //enables real-time camera preview on Driver Station
                .build(); //finalize visionportal setup

        waitForStart();//wait for driver to press play

        //MAIN OPERATION LOOP
        while (!isStopRequested() && opModeIsActive()) {

            //get list of all detected AprilTags from processor
            List<AprilTagDetection> detections = tagProcessor.getDetections();

            //if there's no tags, show a message
            if (detections.isEmpty()) {
                telemetry.addLine("No tags detected");
            }

            //process all detected tags
            else {
                //AprilTagDetection tag = tagProcessor.getDetections().get(0);
                for (AprilTagDetection tag : detections) {
                    //tag is first detected tag
                    telemetry.addData("Tag ID", tag.id); //display tag number

                    //if pose (position/orientation) isn't available, show a warning
                    if (tag.ftcPose == null) {
                        telemetry.addLine("Pose not available for this tag");
                    }

                    //otherwise, continue and extract and display pose data
                    else {
                        //x, y, z
                        double x = tag.ftcPose.x;
                        double y = tag.ftcPose.y;
                        double z = tag.ftcPose.z;

                        //extract orientation
                        double roll = tag.ftcPose.roll;
                        double pitch = tag.ftcPose.pitch;
                        double yaw = tag.ftcPose.yaw;

                        //extract relative angles and range to camera
                        double bearing = tag.ftcPose.bearing;
                        double elevation = tag.ftcPose.elevation;
                        double range = tag.ftcPose.range;

                        //display on driver hub
                        telemetry.addData("XYZ", String.format(Locale.US, "%.2f %.2f %.2f", x, y, z));
                        telemetry.addData("RPY", String.format(Locale.US, "%.2f %.2f %.2f", roll, pitch, yaw));
                        telemetry.addData("RBE", String.format(Locale.US, "%.2f %.2f %.2f", range, bearing, elevation));
                        telemetry.addLine(""); //readability
                    }
                }
            }

            telemetry.update(); //update telemetry
            sleep(20); //brief pause to avoid overloading the control loop
        }
    }
}
