package org.firstinspires.ftc.teamcode.opmode;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color/Distnace Test (low)", group = "Sensor")
@Config
public class REVcolorV3Test2 extends LinearOpMode {
    // Declare the sensor
    private int select;
    public double grabDistance = 2.5;
    float[] hsv = new float[3];
    public double distance;
    public int red, blue, green;


    @Override
    public void runOpMode() {

        // Initialize the sensor - use the name you set in the Robot Configuration
        ColorRangeSensor colorAndDistanceSensor = hardwareMap.get(RevColorSensorV3.class, "highColor");

        waitForStart();

        while (opModeIsActive()) {
            red = colorAndDistanceSensor.red();
            green = colorAndDistanceSensor.green();
            blue = colorAndDistanceSensor.blue();

            distance = colorAndDistanceSensor.getDistance(DistanceUnit.CM);


            Color.RGBToHSV(
                    (int) (colorAndDistanceSensor.red() * 255.0 / 1023),
                    (int) (colorAndDistanceSensor.green() * 255.0 / 1023),
                    (int) (colorAndDistanceSensor.blue() * 255.0 / 1023),
                    hsv
            );


            telemetry.addData("hue", hsv[0]);
            telemetry.addData("sat", hsv[1]);
            telemetry.addData("val", hsv[2]);
            telemetry.addData("dist", distance);

            telemetry.update();
        }
    }
}
