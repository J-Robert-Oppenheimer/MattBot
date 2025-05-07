package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "test")
public class servotesterdouble extends LinearOpMode {

    public static double pos = 0;

    public Servo servo;
    public Servo servo2;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = hardwareMap.get(Servo.class, "servo1" );
        servo2 = hardwareMap.get(Servo.class, "servo2" );
//reverse: lowTurn
        servo2.setDirection(Servo.Direction.REVERSE);
        servo.setPosition(0);
        servo2.setPosition(0);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                pos += 0.01;
                sleep(50);
            }
            if (gamepad1.b) {
                pos -= 0.01;
                sleep(50);
            }
            if (gamepad1.x) {
                pos += 0.1;
                sleep(50);
            }
            if (gamepad1.y) {
                pos -= 0.1;
                sleep(50);
            }

            servo.setPosition(pos);
            servo2.setPosition(pos);



            telemetry.addData("servoPos: ", servo.getPosition());
            telemetry.update();

        }
    }
}


