package org.firstinspires.ftc.teamcode.KishenPractice;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ExampleOpmodes.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@TeleOp
public class DrivePractice extends LinearOpMode {
    private Follower follower;
    DcMotor leftFront,leftBack,rightFront,rightBack;
    private final Pose startPose = new Pose(0,0,0);
    dumbMap dumbBot = new dumbMap(this);


    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        dumbBot.drivePower = 1.0;
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD );
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //right back is 3
        //left front wheel is 0
        //right front wheel is 1
        //left back wheel is 2
        waitForStart();
        follower.startTeleopDrive();

        while(opModeIsActive()){

            follower.setTeleOpMovementVectors(gamepad1.left_stick_y* dumbBot.drivePower, -gamepad1.left_stick_x * dumbBot.drivePower,
                    -gamepad1.right_stick_x *Math.abs(dumbBot.drivePower), true);
            follower.update();
        }












    }



}
