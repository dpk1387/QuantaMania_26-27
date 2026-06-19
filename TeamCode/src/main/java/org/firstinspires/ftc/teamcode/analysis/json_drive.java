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
    DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight;

    ElapsedTime runtime = new ElapsedTime();

    JSONArray logArray = new JSONArray();

    double log_interval = 0.5;
    double switch_interval = 2.0;
    double lastLog = 0;

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

        while (opModeIsActive() && runtime.seconds() < 6) {
            if (runtime.seconds() - switchTime >= switch_interval) {
                forward = !forward;
                switchTime = runtime.seconds();
            }

            double power = forward ? 0.3 : -0.3;
            FrontLeft.setPower(power);
            FrontRight.setPower(power);
            BackLeft.setPower(power);
            BackRight.setPower(power);

            if (runtime.seconds() - lastLog >= log_interval) {
                try {
                    JSONObject sample = new JSONObject();
                    JSONObject fl = new JSONObject();
                    JSONObject fr = new JSONObject();
                    JSONObject bl = new JSONObject();
                    JSONObject br = new JSONObject();

                    long timestamp_ms = System.currentTimeMillis();
                    sample.put("run time", runtime.seconds());
                    sample.put("timestamp", new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS").format(new Date(timestamp_ms)));

                    fl.put("power", FrontLeft.getPower() * 1000.0 / 1000.0);
                    //fl.put("position", FrontLeft.getCurrentPosition());
                    fl.put("current", FrontLeft.getCurrent(CurrentUnit.AMPS) * 1000.0 / 1000.0);

                    fr.put("power", FrontRight.getPower() * 1000.0 / 1000.0);
                    //fr.put("position", FrontRight.getCurrentPosition());
                    fr.put("current", FrontRight.getCurrent(CurrentUnit.AMPS) * 1000.0 / 1000.0);

                    bl.put("power", BackLeft.getPower() * 1000.0 / 1000.0);
                    //bl.put("position", BackLeft.getCurrentPosition());
                    bl.put("current", BackLeft.getCurrent(CurrentUnit.AMPS) * 1000.0 / 1000.0);

                    br.put("power", BackRight.getPower() * 1000.0 / 1000.0);
                    //br.put("position", BackRight.getCurrentPosition());
                    br.put("current", BackRight.getCurrent(CurrentUnit.AMPS) * 1000.0 / 1000.0);

                    sample.put("FrontLeft", fl);
                    sample.put("FrontRight", fr);
                    sample.put("BackLeft", bl);
                    sample.put("BackRight", br);

                    logArray.put(sample);

                    lastLog = runtime.seconds();

                } catch (JSONException e) {
                    telemetry.addLine("JSON Logging Error");
                    telemetry.addData("Error", e.getMessage());
                    telemetry.update();
                }
            }
        }

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        try {
            String path = "/storage/emulated/0/Download/BLACKJACK.json";

            File file = new File(path);

            JSONArray existingArray = new JSONArray();

            if (file.exists() && file.length() > 0) {
                try {
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

            for (int i = 0; i < logArray.length(); i++) {
                existingArray.put(logArray.get(i));
            }

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
