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

@Autonomous(name = "JSON Drive")
public class json_drive extends LinearOpMode {
    DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight;

    ElapsedTime runtime = new ElapsedTime();

    JSONArray logArray = new JSONArray();

    double log_interval = 2.0;
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

        while (opModeIsActive() && runtime.seconds() < 9) {
            if (runtime.seconds() - switchTime >= 2) {
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

                    sample.put("time", runtime.seconds());

                    fl.put("power", FrontLeft.getPower());
                    fl.put("position", FrontLeft.getCurrentPosition());
                    fl.put("current", FrontLeft.getCurrent(CurrentUnit.AMPS));

                    fr.put("power", FrontRight.getPower());
                    fr.put("position", FrontRight.getCurrentPosition());
                    fr.put("current", FrontRight.getCurrent(CurrentUnit.AMPS));

                    bl.put("power", BackLeft.getPower());
                    bl.put("position", BackLeft.getCurrentPosition());
                    bl.put("current", BackLeft.getCurrent(CurrentUnit.AMPS));

                    br.put("power", BackRight.getPower());
                    br.put("position", BackRight.getCurrentPosition());
                    br.put("current", BackRight.getCurrent(CurrentUnit.AMPS));

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
            //String path = Environment.getExternalStorageDirectory().getPath() + "/FIRST/drive_log.json";
            String path = "/storage/emulated/0/Download/POKER.json";

            File file = new File(path);
            //File file = new File(hardwareMap.appContext.getExternalFilesDir(null),
            //        "drive_log.json");

            FileWriter writer = new FileWriter(file);

            telemetry.addData("Samples", logArray.length());

            sleep(500);

            writer.write(logArray.toString(4));

            writer.flush();
            //writer.close();

            BufferedReader reader = new BufferedReader(new FileReader(file));
            String line;
            StringBuilder content = new StringBuilder();

            while ((line = reader.readLine()) != null) {
                content.append(line);
            }
            reader.close();

            /*
            telemetry.addData("Exists", file.exists());
            telemetry.addData("Length", file.length());
            telemetry.addLine("Saved JSON file");
            telemetry.addData("File Path", path);
            telemetry.addData("Can Read", file.canRead());
            telemetry.addData("Can Write", file.canWrite());
            telemetry.addData("Parent Exists", file.getParentFile().exists());
            telemetry.addData("Directory", file.getParentFile().list().length);
            telemetry.addData("Parent path", file.getParentFile().getAbsolutePath());

            File parent = file.getParentFile();
            File[] files = parent.listFiles();

            if (files != null) {
                for (File f : files) {
                    telemetry.addLine(f.getName());
                }
            }*/

            telemetry.addData("File Content", content.toString());

            telemetry.update();

            sleep(18000);

        } catch (Exception e) {
            telemetry.addLine("File save failed");
            telemetry.addData("Error", e.toString());
            telemetry.update();

            sleep(7000);
        }

        sleep(13000);
    }
}
