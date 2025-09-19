package org.firstinspires.ftc.teamcode.KishenPractice;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.ExampleOpmodes.dumbMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config

public class ResetEncoderssss extends OpMode {
    dumbMap dumbBot = new dumbMap(this);

    DcMotor bird;

    @Override
    public void init() {

        bird = hardwareMap.dcMotor.get("bird");
        bird.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bird.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bird.setTargetPosition(0);
        bird.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop() {
        if (gamepad2.x && gamepad2.y) {
            bird.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }

    }

}
