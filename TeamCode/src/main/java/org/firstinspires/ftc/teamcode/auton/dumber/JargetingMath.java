package org.firstinspires.ftc.teamcode.auton.dumber;


// RR-specific imports

import android.graphics.Color;

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
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.dumbMap;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

//chris was here
@Config
@TeleOp(name = "beanTuah", group = "Autonomous")


public class JargetingMath extends LinearOpMode {

    Limelight3A limelight;
    RevColorSensorV3 lowColor;
    float[] hsv;
    double hue;
    double val;
    double mode;
    LLResult result;
    double[] array;
    String stage;
    int iterCount;
    boolean wasLast;
    Timer pathTimer;
    Servo lowClaw, lowRot, lowBonk, lowTurn, sniffer;
    double desiredY;
    double desiredX;
    double lowTurnAngle;
    double lowRotAngle;
    double bonkLength = 8;
    double limeLength = 4;
    double limeLengthX = -0.5;
    int anvilTarget = 0;
    double inchPerAnvil = -145.1 / (0.7121771654*2* Math.PI) ;
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
        limelight.pipelineSwitch(0);
        limelight.updatePythonInputs(mode,0,0,0,0,0,0,0);
        
        lowColor = hardwareMap.get(RevColorSensorV3.class, "lowColor");
        hsv = new float[3];

        mode = 3;

        lowClaw.scaleRange(0.54, 0.71); //close, open
        lowClaw.setPosition(1);
        lowRot.setPosition(0.84);
        lowBonk.setPosition(0.55);
        sniffer.setPosition(0.85);
        lowTurn.setPosition(0.81);

        stage = "dormant";

        wasLast = false;
        pathTimer = new Timer();
        pathTimer.resetTimer();

        waitForStart();
        limelight.start();
        limelight.pipelineSwitch(0);
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (stage == "dormant") {
                if (!wasLast) {
                    wasLast = true;
                }

                mode = 0;
                limelight.updatePythonInputs(mode, 0,0,0,0,0,0,0);
                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    if (array[0] == 1) {
                        LimelightYOffsets.add(array[4]);
                        LimelightXOffsets.add(array[3]);
                        LimelightThetas.add(array[1]);
                        desiredY = map.averageLastContents(LimelightYOffsets, 3) + limeLength;
                        desiredX = map.averageLastContents(LimelightXOffsets, 3) + limeLengthX;
                        lowTurnAngle = Math.toDegrees(Math.asin(desiredX / bonkLength));
                        lowRotAngle = lowTurnAngle + map.averageLastContents(LimelightThetas, 3);
                        lowRot.setPosition(0.17 + lowRotAngle / 180 * 0.68);
                    }
                }

                sniffer.setPosition(0.85);
                lowClaw.setPosition(1);
                lowBonk.setPosition(0.55);
                lowTurn.setPosition(0.81);
                slide3.setTargetPosition(0);
                mode = 3;

                if (gamepad1.dpad_left) {
                    stage = "auto searching";
                    wasLast = false;
                    pathTimer.resetTimer();
                    mode = 0;
                } else if (gamepad1.dpad_up) {
                    stage = "auto searching";
                    wasLast = false;
                    pathTimer.resetTimer();
                    mode = 1;
                } else if (gamepad1.dpad_right) {
                    stage = "auto searching";
                    wasLast = false;
                    pathTimer.resetTimer();
                    mode = 2;
                }
            }

            if (stage == "auto searching") {
                if (!wasLast) {
                    limelight.updatePythonInputs(mode, 0,0,0,0,0,0,0);
                    wasLast = true;
                }

                sniffer.setPosition(0.85);
                lowClaw.setPosition(1);
                lowRot.setPosition(0.84);
                lowBonk.setPosition(0.55);
                lowTurn.setPosition(0.81);

                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    if (array[0] == 1) {
                        stage = "limeCheck";
                        wasLast = false;
                        pathTimer.resetTimer();
                    } else {
                        if (slide3.getCurrentPosition() > -400) slide3.setTargetPosition(slide3.getTargetPosition() - 2);
                        else slide3.setTargetPosition(0);
                    }
                }

                if (gamepad1.dpad_down) {
                    stage = "dormant";
                    wasLast = false;
                    pathTimer.resetTimer();
                    mode = 3;
                }
            }

            if (stage == "limeCheck") {
                if (!wasLast) {
                    wasLast = true;
                }

                sniffer.setPosition(0.85);
                lowClaw.setPosition(1);
                lowRot.setPosition(0.84);
                lowBonk.setPosition(0.55);
                lowTurn.setPosition(0.81);
                slide3.setTargetPosition(slide3.getCurrentPosition());

                if (pathTimer.getElapsedTime() > 50) {
                    stage = "found";
                    wasLast = false;
                    pathTimer.resetTimer();
                }
                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    if (array[0] == 1) {
                        LimelightYOffsets.add(array[4]);
                        LimelightXOffsets.add(array[3]);
                        LimelightThetas.add(array[1]);
                    } else {
                        stage = "auto searching";
                        wasLast = false;
                        pathTimer.resetTimer();
                    }
                }
            }

            if (stage == "found") {
                if (!wasLast) {
                    wasLast = true;

                    desiredY = map.averageLastContents(LimelightYOffsets, 3) + limeLength;
                    desiredX = map.averageLastContents(LimelightXOffsets, 3) + limeLengthX;
                    lowTurnAngle = Math.toDegrees(Math.asin(desiredX / bonkLength));
                    lowRotAngle = lowTurnAngle + map.averageLastContents(LimelightThetas, 3);
                    anvilTarget = (int) ((desiredY - Math.sqrt(bonkLength * bonkLength - Math.pow(desiredX, 2))) * inchPerAnvil + slide3.getCurrentPosition());
                }

                sniffer.setPosition(0.3);
                lowClaw.setPosition(1);
                lowRot.setPosition(0.18 + lowRotAngle / 180 * 0.67);
                lowBonk.setPosition(0.55);
                lowTurn.setPosition(0.54 - lowTurnAngle / 180 * 0.54);
                if (anvilTarget < 0 && anvilTarget > -500){
                    slide3.setTargetPosition(anvilTarget);
                }

                if (pathTimer.getElapsedTime() > 350) {
                    stage = "pre bite";
                    wasLast = false;
                    pathTimer.resetTimer();
                }
            }

            if (stage == "pre bite") {
                if (!wasLast) {
                    wasLast = true;
                }

                sniffer.setPosition(0.3);
                lowClaw.setPosition(1);
                lowRot.setPosition(0.18 + lowRotAngle / 180 * 0.67);
                lowBonk.setPosition(0.73);
                lowTurn.setPosition(0.54 - lowTurnAngle / 180 * 0.54);
                if (anvilTarget < 0 && anvilTarget > -500){
                    slide3.setTargetPosition(anvilTarget);
                }

                if (pathTimer.getElapsedTime() > 200) {
                    stage = "bite";
                    wasLast = false;
                    pathTimer.resetTimer();
                }
            }

            if (stage == "bite") {
                if (!wasLast) {
                    wasLast = true;
                }

                sniffer.setPosition(0.3);
                lowClaw.setPosition(0);
                lowRot.setPosition(0.18 + lowRotAngle / 180 * 0.67);
                lowBonk.setPosition(0.73);
                lowTurn.setPosition(0.54 - lowTurnAngle / 180 * 0.54);
                if (anvilTarget < 0 && anvilTarget > -500){
                    slide3.setTargetPosition(anvilTarget);
                }

                if (gamepad1.dpad_down) {
                    stage = "dormant";
                    wasLast = false;
                    pathTimer.resetTimer();
                    mode = 3;
                }
                
                if (pathTimer.getElapsedTime() > 200) {
                    stage = "revCheck";
                    wasLast = false;
                    pathTimer.resetTimer();
                }
            }
            
            if (stage == "revCheck") {
                if (!wasLast) {
                    wasLast = true;
                }
                
                sniffer.setPosition(0.85);
                lowClaw.setPosition(0);
                lowRot.setPosition(0.84);
                lowBonk.setPosition(0.65);
                lowTurn.setPosition(0.54);
                if (anvilTarget < 0 && anvilTarget > -500){
                    slide3.setTargetPosition(anvilTarget);
                }

                Color.RGBToHSV(
                        (int) (lowColor.red() * 255.0 / 1023),
                        (int) (lowColor.green() * 255.0 / 1023),
                        (int) (lowColor.blue() * 255.0 / 1023),
                        hsv
                );
                hue = hsv[0];
                if (mode == 0 && (hue < 30 || hue > 330)) {
                    telemetry.addLine("success");
                } else if (mode == 1 && (hue < 85 && hue > 50)) {
                    telemetry.addLine("success");
                } else if (mode == 2 && (hue < 230 && hue > 210)) {
                    telemetry.addLine("success");
                } else {
                    telemetry.addLine("fail");
                    if (pathTimer.getElapsedTime() > 600) {
                        stage = "auto searching";
                        wasLast = false;
                        pathTimer.resetTimer();
                    }
                }

                if (gamepad1.dpad_down) {
                    stage = "dormant";
                    wasLast = false;
                    pathTimer.resetTimer();
                    mode = 3;
                }
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

            telemetry.update();
        }//While OpMode Active
        limelight.stop();
    }//While Running
}