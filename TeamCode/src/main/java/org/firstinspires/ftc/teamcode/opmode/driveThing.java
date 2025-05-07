package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DriveThing")
@Config
public class driveThing extends LinearOpMode {
    dumbMap dumbBot = new dumbMap(this);
    private Follower follower;

    DcMotor leftFront, leftBack, rightFront, rightBack;
    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
//        leftFront = hardwareMap.dcMotor.get("leftFront");
//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        rightFront = hardwareMap.dcMotor.get("rightFront");
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftBack = hardwareMap.dcMotor.get("leftRear");
//        leftBack.setDirection(DcMotor.Direction.REVERSE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        rightBack = hardwareMap.dcMotor.get("rightRear");
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//I need a change to push
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);


        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()) {
            //movement
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * dumbBot.drivePower,
                    -gamepad1.left_stick_x * dumbBot.drivePower,
                    -gamepad1.right_stick_x * Math.abs(dumbBot.drivePower), true);
            follower.update();

            if (!dumbBot.xLast && gamepad1.x) {
                dumbBot.halfSpeedToggle = !dumbBot.halfSpeedToggle;
                dumbBot.qtrSpeedToggle = false;
            }
            if (!dumbBot.bLast && gamepad1.b) {
                dumbBot.qtrSpeedToggle = !dumbBot.qtrSpeedToggle;
                dumbBot.halfSpeedToggle = false;
            }
            if (!dumbBot.yLast && gamepad1.y) {
                dumbBot.drivingReverse = !dumbBot.drivingReverse;
            }
            if (dumbBot.drivingReverse) {
                if (dumbBot.halfSpeedToggle) {
                    dumbBot.drivePower = -0.5;
                } else if (dumbBot.qtrSpeedToggle) {
                    dumbBot.drivePower = -0.25;
                } else {
                    dumbBot.drivePower = -1;
                }
            } else {
                if (dumbBot.halfSpeedToggle) {
                    dumbBot.drivePower = 0.5;
                } else if (dumbBot.qtrSpeedToggle) {
                    dumbBot.drivePower = 0.25;
                } else {
                    dumbBot.drivePower = 1;
                }
            }
            dumbBot.xLast = gamepad1.x;
            dumbBot.yLast = gamepad1.y;
            dumbBot.bLast = gamepad1.b;
        }
    }
}
