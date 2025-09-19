package org.firstinspires.ftc.teamcode.KishenPractice;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

public class ArcadePractice {
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    public void init(HardwareMap hwMap){
        frontLeft = hwMap.get(DcMotor.class, "front_left");
        backLeft = hwMap.get(DcMotor.class, "back_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backRight = hwMap.get(DcMotor.class, "back_right");


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);









    }
    public void drive(double throttle, double spin){
        double leftPower = throttle + spin;
        double rightPower = throttle - spin;
        double largest = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (largest>1.0){
            leftPower /= largest;
            rightPower /= largest;

        }
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);
        frontRight.setPower(rightPower);





    }
}
