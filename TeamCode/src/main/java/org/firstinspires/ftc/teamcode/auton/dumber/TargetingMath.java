package org.firstinspires.ftc.teamcode.auton.dumber;


// RR-specific imports

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Drawing;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.dumbMap;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

//chris was here
@Config
@TeleOp(name = "bean", group = "Autonomous")


public class TargetingMath extends LinearOpMode {

    public Limelight3A limelight;
    double mode;
    LLResult result;
    double[] array;
    String stage;
    int iterCount;
    boolean wasLast;
    Timer pathTimer;
    Servo lowClaw, lowRot, lowBonk, lowTurn, sniffer;
    double desiredY;
    double biProductY;
    double desiredX;
    double lowTurnAngle;
    double lowRotAngle;
    double bonkLength = 7.5;
    double limeLength = 4;
    double limeLengthX = -0.5;
    int anvilTarget = 0;
    double lowTurnTarget;
    double lowRotTarget;
    double inchPerAnvil = -145.1 / (0.7121771654*2* Math.PI) ;
    boolean firstTouch = true;
    DcMotor slide3;
    ArrayList<Double> LimelightXOffsets = new ArrayList<>();
    ArrayList<Double> LimelightYOffsets = new ArrayList<>();
    ArrayList<Double> LimelightThetas = new ArrayList<>();

    dumbMap map = new dumbMap(this);

    //Get Position and Angle
    //Calculate needed turn to reach desired x offset
    //Store the amount of biProduct y offset
    //Move the slide within its range

    @Override //this is where set up is done
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // instantiate your Mecanum Drive at a particular pose.

        lowClaw = this.hardwareMap.get(Servo.class, "lowClaw" ); // 0.04 (close) 0.28 (open)
        lowRot = this.hardwareMap.get(Servo.class, "lowRot" ); // 0.5 (par) 0.83 (perp)
        lowBonk = this.hardwareMap.get(Servo.class, "lowBonk" ); // 0.77 (down) 0.36 (up) 0.  (back)
        lowTurn = this.hardwareMap.get(Servo.class, "lowTurn" ); // 0.48 straight 0.84 left
        sniffer = this.hardwareMap.get(Servo.class, "llServo" ); // 0.48 straight 0.84 left

        slide3 = hardwareMap.dcMotor.get("anvilslide"); //0 - -400
        slide3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide3.setDirection(DcMotorSimple.Direction.REVERSE);
        slide3.setTargetPosition(0);
        slide3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide3.setTargetPosition(0);
        slide3.setPower(1);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.updatePythonInputs(mode, 0,0,0,0,0,0,0);
        limelight.start();
        
        mode = 0;

        lowClaw.scaleRange(0.41, 0.6); //open, close
        lowClaw.setPosition(0.5);
        lowRot.setPosition(0.84);
        lowBonk.setPosition(0.6);
        sniffer.setPosition(0.8);
        lowTurn.setPosition(0.81);

        stage = "auto searching";

        wasLast = false;
        pathTimer = new Timer();
        pathTimer.resetTimer();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            //Determine Number of Reds
            telemetry.addLine("mode: " + (mode == 0 ? "red" : (mode == 1 ? "yellow" : "blue")));
            if (gamepad1.dpad_left) {
                mode = 0;
                limelight.updatePythonInputs(mode, 0,0,0,0,0,0,0);
            } else if (gamepad1.dpad_up) {
                mode = 1;
                limelight.updatePythonInputs(mode, 0,0,0,0,0,0,0);
            } else if (gamepad1.dpad_right) {
                mode = 2;
                limelight.updatePythonInputs(mode, 0,0,0,0,0,0,0);
            }
            if (mode == 0) {
                gamepad1.setLedColor(255, 0, 0, 10000);
            } else if (mode == 1) {
                gamepad1.setLedColor(255, 255, 0, 10000);
            } else if (mode == 2) {
                gamepad1.setLedColor(0, 0, 255, 10000);
            } else {
                gamepad1.setLedColor(255, 255, 255, 10000);
            }

            if (stage == "dormant") {
                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    LimelightYOffsets.add(array[4]);
                    LimelightXOffsets.add(array[3]);
                    LimelightThetas.add(array[1]);

                    desiredY = map.averageLastContents(LimelightYOffsets, 5) + limeLength;
                    desiredX = map.averageLastContents(LimelightXOffsets, 5) + limeLengthX;
                    telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                    telemetry.addData("LimelightXOffset", array[3]);
                    telemetry.addData("LimelightYOffset", array[4]);
                    telemetry.addData("LimelightTheta", array[1]);
                    telemetry.addData("limelight on", limelight.isRunning());

                    biProductY = Math.sqrt(bonkLength * bonkLength - Math.pow(desiredX, 2));
                    lowTurnAngle = Math.toDegrees(Math.asin(desiredX / bonkLength));
                    lowRotAngle = lowTurnAngle + map.averageLastContents(LimelightThetas, 5);
                    anvilTarget = (int) ((desiredY - biProductY) * inchPerAnvil + slide3.getTargetPosition());
                    telemetry.addData("DesiredY", desiredY);
                    telemetry.addData("DesiredX", desiredX);
                    telemetry.addData("BiProduct", biProductY);
                    telemetry.addData("LowTurnAngle", lowTurnAngle);
                    telemetry.addData("LowRotAngle", lowRotAngle);
                    telemetry.addData("AnvilTarget", anvilTarget);
                    telemetry.addData("Extension Needed", desiredY - biProductY);
                }
                wasLast = true;
            }

            if (stage == "auto searching") {
                if (slide3.getCurrentPosition() > -400) slide3.setTargetPosition(slide3.getTargetPosition() - 2);
                else slide3.setTargetPosition(0);
                wasLast = true;
                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    LimelightYOffsets.add(array[4]);
                    LimelightXOffsets.add(array[3]);
                    LimelightThetas.add(array[1]);
                    if (map.averageLastContents(LimelightXOffsets, 10) == array[3] && array[3] != 0) {
                        stage = "found";
                        wasLast = false;
                    }
                }
                telemetry.addData("slide3", slide3.getCurrentPosition());
            }

            if (stage == "found") {
                if (!wasLast) iterCount = 0;
                else iterCount++;
                result = limelight.getLatestResult();
                if (result.getPythonOutput()[0] != 1.0) stage = "auto searching";
                if (result != null) {
                    array = result.getPythonOutput();
                    LimelightYOffsets.add(array[4]);
                    LimelightXOffsets.add(array[3]);
                    LimelightThetas.add(array[1]);

                    desiredY = map.averageLastContents(LimelightYOffsets, 3) + limeLength;
                    desiredX = map.averageLastContents(LimelightXOffsets, 3) + limeLengthX;
                    telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                    telemetry.addData("LimelightXOffset", array[3]);
                    telemetry.addData("LimelightYOffset", array[4]);
                    telemetry.addData("LimelightTheta", array[1]);
                    telemetry.addData("limelight on", limelight.isRunning());

                    biProductY = Math.sqrt(bonkLength * bonkLength - Math.pow(desiredX, 2));
                    lowTurnAngle = Math.toDegrees(Math.asin(desiredX / bonkLength));
                    lowRotAngle = lowTurnAngle + map.averageLastContents(LimelightThetas, 3);
                    anvilTarget = (int) ((desiredY - biProductY) * inchPerAnvil + slide3.getTargetPosition());
                    telemetry.addData("DesiredY", desiredY);
                    telemetry.addData("DesiredX", desiredX);
                    telemetry.addData("BiProduct", biProductY);
                    telemetry.addData("LowTurnAngle", lowTurnAngle);
                    telemetry.addData("LowRotAngle", lowRotAngle);
                    telemetry.addData("AnvilTarget", anvilTarget);
                    telemetry.addData("Extension Needed", desiredY - biProductY);
                }
                wasLast = true;
                if (iterCount > 50) {
                    stage = "pre bite";
                    wasLast = false;
                    pathTimer.resetTimer();
                }
            }

            if (stage == "pre bite") {
                sniffer.setPosition(0.25);
                lowClaw.setPosition(1);
                if (anvilTarget < 0 && anvilTarget > -500){
                    slide3.setTargetPosition(anvilTarget);
                }
                lowTurnTarget = 0.54 - lowTurnAngle / 180 * 0.54;
                lowTurn.setPosition(lowTurnTarget);
                lowRotTarget = 0.18 + lowRotAngle / 180 * 0.67;
                lowRot.setPosition(lowRotTarget);
                if (pathTimer.getElapsedTime() > 600) {
                    stage = "bite";
                    pathTimer.resetTimer();
                }
            }

            if (stage == "bite") {
                lowBonk.setPosition(0.73);
                if (pathTimer.getElapsedTime() > 150) {
                    lowClaw.setPosition(0);
                }
                telemetry.addData("DesiredY", desiredY);
                telemetry.addData("DesiredX", desiredX);
                telemetry.addData("BiProduct", biProductY);
                telemetry.addData("LowTurnAngle", lowTurnAngle);
                telemetry.addData("LowRotAngle", lowRotAngle);
                telemetry.addData("AnvilTarget", anvilTarget);
                telemetry.addData("Extension Needed", desiredY - biProductY);
            }

            if (stage == "human operated searching") {
                if (gamepad1.dpad_up && slide3.getTargetPosition() > -500) {
                    slide3.setTargetPosition(slide3.getTargetPosition() - 2);
                } else if (gamepad1.dpad_down && slide3.getTargetPosition() < 0) {
                    slide3.setTargetPosition(slide3.getTargetPosition() + 2);
                }
                wasLast = true;
                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    LimelightYOffsets.add(array[4]);
                    LimelightXOffsets.add(array[3]);
                    LimelightThetas.add(array[1]);
                    if (result.getPythonOutput()[0] == 1.0) {
                        stage = "found";
                        wasLast = false;
                    }
                }
            }




            telemetry.addData("stage", stage);
            telemetry.update();
        }//While OpMode Active
        limelight.stop();
    }//While Running
}