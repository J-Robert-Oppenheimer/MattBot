package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "motorthing2", group = "test")
public class motorTest2 extends LinearOpMode {
    DcMotor slide1, slide2, slide3;
    int upslidePos= 10, lowslidePos = 0;

    @Override
    public void runOpMode() {

        slide1 = hardwareMap.dcMotor.get("lefthammer"); //
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setTargetPosition(0);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide2 = hardwareMap.dcMotor.get("righthammer"); //
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setTargetPosition(0);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide3 = hardwareMap.dcMotor.get("anvilslide"); //0 - -400
        slide3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide3.setDirection(DcMotorSimple.Direction.REVERSE);
        slide3.setTargetPosition(0);
        slide3.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        waitForStart();

        while(opModeIsActive()){


            if(gamepad2.right_trigger>0){
                upslidePos+= (int)(gamepad2.right_trigger*75);
                upslidePos = Math.min(upslidePos,2050);
            }
            else if(gamepad2.left_trigger>0){
                upslidePos-= (int)(gamepad2.left_trigger*75);
                upslidePos = Math.max(upslidePos,0);
            }

            slide1.setTargetPosition(upslidePos);
            slide2.setTargetPosition(upslidePos);
            slide1.setPower(1);
            slide2.setPower(1);





            if(gamepad2.left_stick_y>0){
                lowslidePos+= (int)(gamepad2.left_stick_y*50);
                lowslidePos = Math.min(0,lowslidePos);
            }
            else if(gamepad2.left_stick_y<0){
                lowslidePos+= (int)(gamepad2.left_stick_y*50);
                lowslidePos = Math.max(lowslidePos,-400);
            }
            slide3.setTargetPosition(lowslidePos);
            slide3.setPower(1);


            telemetry.addData("slidePos1", slide1.getCurrentPosition());
            telemetry.addData("slidePos2", slide2.getCurrentPosition());
            telemetry.addData("slidePos1-2other", upslidePos);
            telemetry.addData("slidePos1-2other2", slide1.getTargetPosition());



            telemetry.addData("slidePos3", slide3.getCurrentPosition());
            telemetry.addData("slidePos3other", slide3.getTargetPosition());
            telemetry.update();
        }
    }
}
