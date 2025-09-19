package org.firstinspires.ftc.teamcode.KishenPractice;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
    public class MotorTo1000Practice extends LinearOpMode {
        DcMotor bird;
        int desiredPosition = 1000;
        @Override
        public void runOpMode() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            DcMotor bird = hardwareMap.dcMotor.get("bird");
            int fella = bird.getCurrentPosition();
            waitForStart();

            while (opModeIsActive()){
                bird.setPower(0.7);
                bird.setTargetPosition(desiredPosition);
                bird.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                telemetry.update();
                telemetry.addData("Encoder Position",fella );

            }







        }
    }


