package org.firstinspires.ftc.teamcode.opmode;

//luh Teleop
//default position pos = 0.9, vpos = 1

/*flat positions:
pos = 0 (70 deg), vpos = 0.205, slidePos = 0.299
pos = 0.05 (72 deg), vpos = 0.21, slidePos = 0.421
pos = 0.1 (75 deg), vpos = 0.214, slidePos = 0.573
*/

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.dumbMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config

public class ResetEncoders extends OpMode {

    dumbMap dumbBot = new dumbMap(this);
    DcMotor slide, slide2;
    DcMotor slide3;
    @Override
    public void init() {
        slide = hardwareMap.dcMotor.get("lefthammer");
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setTargetPosition(0);

        slide2 = hardwareMap.dcMotor.get("righthammer");
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setTargetPosition(0);

        slide3 = hardwareMap.dcMotor.get("anvilslide");
        slide3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide3.setDirection(DcMotorSimple.Direction.REVERSE);
        slide3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide3.setTargetPosition(0);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    @Override
    public void loop() {
        if(gamepad2.x && gamepad2.y) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if(gamepad2.a && gamepad2.b) {
            slide3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

}