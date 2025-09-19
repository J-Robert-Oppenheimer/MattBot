package org.firstinspires.ftc.teamcode.KishenPractice;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp

public class ServoToZeroPractice extends LinearOpMode {

    public static double cat = 0.5;

    public Servo dog;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dog = hardwareMap.get(Servo.class, "thingie");
        dog.setPosition(0.5);
        waitForStart();
        while (opModeIsActive()){

            if(gamepad1.a)
            {
                cat += 0.1;
                sleep(50);
            }

            if(gamepad1.b)
            {
                cat -= 0.1;
                sleep(50);
            }
            dog.setPosition(cat);

            telemetry.addData("servo postion is ", dog.getPosition());

            telemetry.update();
        }



    }

}
