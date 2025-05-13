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
    public static double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos, llServoPos;

    public Servo outtakeTurn, outtakeOpen, outtakeLift, outtakeRotate, outtakeSlideLiftLeft, outtakeSideLiftRight,intakeLiftRight,intakeliftLeft;

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

        spinner = this.hardwareMap.dcMotor.get("anvilslide"); // control 1
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);
        
        slide1.setTargetPosition(0);
        slide2.setTargetPosition(0);
        slide1.setPower(1);
        slide2.setPower(1);

        lowClaw = this.hardwareMap.get(Servo.class, "lowClaw" ); // 0.4 (close) 0.6 (open)    control 4
        lowRot = this.hardwareMap.get(Servo.class, "lowRot" ); // 0.2 (right) 0.53 (perp) 0.86 (left)    control 0
        lowBonk = this.hardwareMap.get(Servo.class, "lowBonk" ); // 0.75 (down) 0.45 (up) 0.2  (back) 0.6 (searching)    control 3
        lowTurn = this.hardwareMap.get(Servo.class, "lowTurn" ); // 0.35 max right 0.54 mid // 0.81 left    control 4
        llServo = this.hardwareMap.get(Servo.class, "llServo" ); // 0.8 out 0.25 in    control 5


        highClaw = this.hardwareMap.get(Servo.class, "highClaw" ); // 0.78 (close) 0.99 (open)      expansion 1
        highBonkL = this.hardwareMap.get(Servo.class, "highBonkL" );// spec grab  0.105 spec place 0.135(little more?) blockplace 0.2 transfer 0.26 up 0.185
        highBonkR = this.hardwareMap.get(Servo.class, "highBonkR" );// expansion 2L and 3R
        highBonkR.setDirection(Servo.Direction.REVERSE);
        highTurnB = this.hardwareMap.get(Servo.class, "highTurnB" ); // spec grab 0.185 spec place 0.75 blockplace 0.8// transfer 0.17
        highTurnT = this.hardwareMap.get(Servo.class, "highTurnT" );// expansion 4B and 5T
        highTurnT.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {

            slide1.setTargetPosition(hammerSlidePos); // spec grab 700 spec place 900
            slide2.setTargetPosition(hammerSlidePos);

            spinner.setTargetPosition(anvilSlidePos);

            if (lowTurnPos != 0) lowTurn.setPosition(lowTurnPos);
            if (lowBonkPos != 0) lowBonk.setPosition(lowBonkPos);
            if (lowRotPos != 0) lowRot.setPosition(lowRotPos);
            if (lowClawPos != 0) lowClaw.setPosition(lowClawPos);
            if (llServoPos != 0) llServo.setPosition(llServoPos);
            if (highTurnPos != 0) {
                highTurnB.setPosition(highTurnPos);
                highTurnT.setPosition(highTurnPos);
            }
            if (highBonkPos != 0) {
                highBonkR.setPosition(highBonkPos);
                highBonkL.setPosition(highBonkPos);
            }
            if (highClawPos != 0) highClaw.setPosition(highClawPos);

            telemetry.update();

        }
    }
}