package org.firstinspires.ftc.teamcode.KishenPractice;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class MotorTester extends LinearOpMode {
    DcMotor leftFront;
    int desiredPosition = 2000;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        waitForStart();

        while (opModeIsActive()){
            leftFront.setPower(0.7);
            leftFront.setTargetPosition(desiredPosition);
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }







    }
}


