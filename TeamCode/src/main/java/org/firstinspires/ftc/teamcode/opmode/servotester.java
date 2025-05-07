package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servotester (HB test)", group = "test")
public class servotester extends LinearOpMode {

    public Servo servo6, servo7;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo6= hardwareMap.get(Servo.class, "highBonkL" );
        servo7= hardwareMap.get(Servo.class, "highBonkR" );

        servo7.setDirection(Servo.Direction.REVERSE);


        servo6.setPosition(0.1);
        servo7.setPosition(0.1);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                servo6.setPosition(servo6.getPosition() + 0.01);
                servo7.setPosition(servo7.getPosition() + 0.01);
            }
            if (gamepad1.b) {
                servo6.setPosition(servo6.getPosition() - 0.01);
                servo7.setPosition(servo7.getPosition() - 0.01);
            }
            if (gamepad1.x) {
                servo6.setPosition(servo6.getPosition() + 0.1);
                servo7.setPosition(servo7.getPosition() + 0.1);
            }
            if (gamepad1.y) {
                servo6.setPosition(servo6.getPosition() - 0.1);
                servo7.setPosition(servo7.getPosition() - 0.1);
            }
//            servo7.setPosition(1-servo6.getPosition());
//            servo7.setPosition(servo6.getPosition());
            sleep(50);



            telemetry.addData("servoPos: ", servo6.getPosition());
            telemetry.addData("servoPos2: ", servo7.getPosition());
            telemetry.update();

        }
    }
}


