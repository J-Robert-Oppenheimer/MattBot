package org.firstinspires.ftc.teamcode.opmode;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color/Distnace Test", group = "Sensor")
@Config
public class REVcolorV3Test extends LinearOpMode {
    // Declare the sensor
    private int select;
    public double grabDistance = 2.5;
    public double distance;
    public int red, blue, green;

    float[] hsv;



    @Override
    public void runOpMode() {
        Servo claw = hardwareMap.get(Servo.class, "highClaw");
        claw.scaleRange(0.22, 0.5); //open, close

        // Initialize the sensor - use the name you set in the Robot Configuration
        RevColorSensorV3 colorAndDistanceSensor = hardwareMap.get(RevColorSensorV3.class, "lowColor");
        //ColorRangeSensor colorRangeSensor = hardwareMap.get(RevColorSensorV3.class, "Sensor"); //?

        hsv = new float[3];

        waitForStart();

        while (opModeIsActive()) {
            // Get distance
            distance = colorAndDistanceSensor.getDistance(DistanceUnit.CM);

            // Get color values
            red = colorAndDistanceSensor.red();
            green = colorAndDistanceSensor.green();
            blue = colorAndDistanceSensor.blue();

            Color.RGBToHSV(
                    (int) (colorAndDistanceSensor.red() * 255.0 / 1023),
                    (int) (colorAndDistanceSensor.green() * 255.0 / 1023),
                    (int) (colorAndDistanceSensor.blue() * 255.0 / 1023),
                    hsv
            );

            telemetry.addData("hue", hsv[0]);
            telemetry.addData("sat", hsv[1]);
            telemetry.addData("val", hsv[2]);

            //yellow light is red light and blue light
            //red is just red
            //blue is just blue

            // Example of using both


            telemetry.update();
        }
    }
}
