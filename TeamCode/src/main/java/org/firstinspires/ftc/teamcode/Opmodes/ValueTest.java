package org.firstinspires.ftc.teamcode.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ValueTest extends LinearOpMode {

    public DcMotor leftFront, rightFront, leftRear, rightRear;
    public DcMotor slide1, slide2, spinner;

    public static int anvilSlidePos, hammerSlidePos;
    public static double outtakeRotatePos, outtakeLiftPos, outtakeOpenPos, outtakeTurnPos, intakeLiftPos, outtakeSlideLiftRightPos, outtakeSlideLiftLeftPos;

    public Servo outtakeTurn, outtakeOpen, outtakeLift, outtakeRotate, outtakeSlideLiftLeft, outtakeSlideLiftRight,intakeLiftRight,intakeLiftLeft;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //DRIVE MOTOR INITIALIZATION
        leftFront = this.hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront = this.hardwareMap.dcMotor.get("rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear = this.hardwareMap.dcMotor.get("leftRear");
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = this.hardwareMap.dcMotor.get("rightRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //ROBOT ACTUATION

        slide1 = this.hardwareMap.dcMotor.get("slide1"); // control 2
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setTargetPosition(0);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide2 = this.hardwareMap.dcMotor.get("slide2"); // expansion 1 , 0 is empty
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide2.setTargetPosition(0);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spinner = this.hardwareMap.dcMotor.get("spinner"); // control 1
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        
        slide1.setTargetPosition(0);
        slide2.setTargetPosition(0);
        slide1.setPower(1);
        slide2.setPower(1);

        outtakeTurn = this.hardwareMap.get(Servo.class, "outtakeTurn" ); // 0.4 (close) 0.6 (open)    control 4
        outtakeOpen = this.hardwareMap.get(Servo.class, "outtakeOpen" ); // 0.2 (right) 0.53 (perp) 0.86 (left)    control 0
        outtakeLift = this.hardwareMap.get(Servo.class, "outtakeLift" ); // 0.75 (down) 0.45 (up) 0.2  (back) 0.6 (searching)    control 3
        outtakeRotate = this.hardwareMap.get(Servo.class, "outtakeRotate" ); // 0.35 max right 0.54 mid // 0.81 left    control 4
        outtakeSlideLiftLeft = this.hardwareMap.get(Servo.class, "outtakeSlideLiftLeft" ); // 0.8 out 0.25 in    control 5


        outtakeSlideLiftRight = this.hardwareMap.get(Servo.class, "outtakeSlideLiftRight" ); // 0.78 (close) 0.99 (open)      expansion 1
        intakeLiftRight = this.hardwareMap.get(Servo.class, "intakeLiftRight" );// spec grab  0.105 spec place 0.135(little more?) blockplace 0.2 transfer 0.26 up 0.185
        intakeLiftLeft = this.hardwareMap.get(Servo.class, "intakeLiftLeft" );// expansion 2L and 3R
        intakeLiftLeft.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {

            slide1.setTargetPosition(hammerSlidePos); // spec grab 700 spec place 900
            slide2.setTargetPosition(hammerSlidePos);

            spinner.setTargetPosition(anvilSlidePos);

            if (outtakeRotatePos != 0) outtakeRotate.setPosition(outtakeRotatePos);
            if (outtakeLiftPos != 0) outtakeLift.setPosition(outtakeLiftPos);
            if (outtakeOpenPos != 0) outtakeOpen.setPosition(outtakeOpenPos);
            if (outtakeTurnPos != 0) outtakeTurn.setPosition(outtakeTurnPos);
            if (outtakeSlideLiftLeftPos != 0) outtakeSlideLiftLeft.setPosition(outtakeSlideLiftLeftPos);
            if (intakeLiftPos != 0) {
                intakeLiftLeft.setPosition(intakeLiftPos);
                intakeLiftRight.setPosition(intakeLiftPos);
            }
            if (outtakeSlideLiftRightPos != 0) outtakeSlideLiftRight.setPosition(outtakeSlideLiftRightPos);

            telemetry.update();

        }
    }
}