package org.firstinspires.ftc.teamcode.auton.dumber;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class dumbDrive extends OpMode {
    public DcMotor leftFront, rightFront, leftBack, rightBack;
    MecanumDrive drive;
    @Override
    public void init() {
        leftFront = this.hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront = this.hardwareMap.dcMotor.get("rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack = this.hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack = this.hardwareMap.dcMotor.get("rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
    }

    @Override
    public void loop() {
        drive.moveInTeleop(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, Math.pow(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.right_stick_x, 2), 0.5));

    }
}
