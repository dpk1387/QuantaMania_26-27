package org.firstinspires.ftc.teamcode.analysis;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

@Autonomous(name = "JSON Drive")
public class json_drive extends LinearOpMode {
    // set up variables and motors

    DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight;

    ElapsedTime runtime = new ElapsedTime();

    JSONArray logArray = new JSONArray();

    double log_interval = 0.5; // time interval in seconds between collecting data points
    double switch_interval = 2.0; // seconds between switching direction
    double lastLog = 0; // time in seconds of last data collection

    @Override
    public void runOpMode() throws InterruptedException {
        FrontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftWheel");
        FrontRight = hardwareMap.get(DcMotorEx.class, "frontRightWheel");
        BackLeft = hardwareMap.get(DcMotorEx.class, "backLeftWheel");
        BackRight = hardwareMap.get(DcMotorEx.class, "backRightWheel");

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        BackLeft.setDirection(DcMotorEx.Direction.REVERSE);

        boolean forward = true;


        waitForStart();

        runtime.reset();

        double switchTime = 0;

        while (opModeIsActive() && runtime.seconds() < 6) { // run robot for 6 seconds

            // drives the robot foward and backward for fixed duration while collecting data about the motor current and power
            if (runtime.seconds() - switchTime >= switch_interval) {
                forward = !forward;
                switchTime = runtime.seconds();
            }

            double power = forward ? 0.3 : -0.3; // drive forward, then reverse after each switch interval
            FrontLeft.setPower(power);
            FrontRight.setPower(power);
            BackLeft.setPower(power);
            BackRight.setPower(power);

            if (runtime.seconds() - lastLog >= log_interval) { // adding data samples into a json file
                try {
                    // create new JSON objects for each new data sample. fl, fr, bl, br contain data for each motor
                    JSONObject sample = new JSONObject();
                    JSONObject fl = new JSONObject();
                    JSONObject fr = new JSONObject();
                    JSONObject bl = new JSONObject();
                    JSONObject br = new JSONObject();

                    long timestamp_ms = System.currentTimeMillis(); // milliseconds since January 1, 1970, midnight UTC
                    sample.put("run time", runtime.seconds()); // seconds since program started running
                    sample.put("timestamp", new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS").format(new Date(timestamp_ms))); // converting timestamp_ms to date and time

                    fl.put("power", Math.round(FrontLeft.getPower() * 1000.0) / 1000.0);
                    //fl.put("position", FrontLeft.getCurrentPosition());
                    fl.put("current", Math.round(FrontLeft.getCurrent(CurrentUnit.AMPS) * 1000.0) / 1000.0);

                    fr.put("power", Math.round(FrontRight.getPower() * 1000.0) / 1000.0);
                    //fr.put("position", FrontRight.getCurrentPosition());
                    fr.put("current", Math.round(FrontRight.getCurrent(CurrentUnit.AMPS) * 1000.0) / 1000.0);

                    bl.put("power", Math.round(BackLeft.getPower() * 1000.0) / 1000.0);
                    //bl.put("position", BackLeft.getCurrentPosition());
                    bl.put("current", Math.round(BackLeft.getCurrent(CurrentUnit.AMPS) * 1000.0) / 1000.0);

                    br.put("power", Math.round(BackRight.getPower() * 1000.0) / 1000.0);
                    //br.put("position", BackRight.getCurrentPosition());
                    br.put("current", Math.round(BackRight.getCurrent(CurrentUnit.AMPS) * 1000.0) / 1000.0);

                    sample.put("FrontLeft", fl);
                    sample.put("FrontRight", fr);
                    sample.put("BackLeft", bl);
                    sample.put("BackRight", br);

                    logArray.put(sample); // adding completed sample to the in-memory log

                    lastLog = runtime.seconds(); // update logging timer so next sample is collected after another log_interval seconds

                } catch (JSONException e) {
                    telemetry.addLine("JSON Logging Error");
                    telemetry.addData("Error", e.getMessage());
                    telemetry.update();
                }
            }
        }

        // stop motors
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        try {
            String path = "/storage/emulated/0/Download/BLACKJACK.json"; // path of the json file
            // this file exists on the control hub

            File file = new File(path);

            JSONArray existingArray = new JSONArray(); // create a new json array

            if (file.exists() && file.length() > 0) {
                try {
                    // reads entire existing json file into memory so new samples can be appended with no invalid json
                    BufferedReader reader = new BufferedReader(new FileReader(file));
                    StringBuilder content = new StringBuilder();

                    String line;
                    while ((line = reader.readLine()) != null) {
                        content.append(line);
                    }

                    reader.close();

                    existingArray = new JSONArray(content.toString());

                } catch (Exception e) {
                    telemetry.addLine("Old JSON invalid, starting new file");
                }
            }

            // append samples from current run to the existing dataset
            for (int i = 0; i < logArray.length(); i++) {
                existingArray.put(logArray.get(i));
            }

            // rewrite the file with new combined dataset
            FileWriter writer = new FileWriter(file);
            writer.write(existingArray.toString(4));
            writer.close();

            telemetry.addData("File length", file.length());
            telemetry.addData("New samples", logArray.length());
            telemetry.addData("Existing samples", existingArray.length());
            telemetry.update();
            sleep(5000);

        } catch (Exception e) {
            telemetry.addLine("Old JSON invalid, starting new file");
            telemetry.addData("Error", e.toString());
            telemetry.update();

            sleep(7000);
        }

        sleep(13000);
    }
}
