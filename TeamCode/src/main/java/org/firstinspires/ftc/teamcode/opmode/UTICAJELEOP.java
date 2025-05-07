package org.firstinspires.ftc.teamcode.opmode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auton.dumber.MecanumDrive;
import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "UTICAJELEOP", group = "oldteles")
@Config

public class UTICAJELEOP extends LinearOpMode {
    public String state;
    public int mode;
    dumbMap dumbBot = new dumbMap(this);
    public int anvilSlidePos, hammerSlidePos;
    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos, snifferPos;
    private Limelight3A limelight;
    ArrayList<Double> LimelightXOffsets = new ArrayList<>();
    ArrayList<Double> LimelightYOffsets = new ArrayList<>();
    ArrayList<Double> LimelightThetas = new ArrayList<>();
    double[] array;
    RevColorSensorV3 highColor, lowColor;
    float[] lowhsv;
    double lowHue;
    LLResult result;
    boolean wasLast;
    int iterCount;
    public boolean down1Last = false, up1Last = false, right1Last = false, left1Last = false, lowClawRot = true, psLast = false, b1Last;
    double desiredYIn;
    double desiredXIn;
    double lowTurnAngleDeg;
    double lowRotAngleDeg;
    double anvilChangeIn;
    double bonkLength = 8;
    double limeLength = 4.5;
    double limeLengthX = -0.5;
    double inchPerAnvil = -145.1 / (0.7121771654*2* Math.PI);
    public DcMotor leftFront, rightFront, leftBack, rightBack;
    MecanumDrive drive;

    private Timer time;

    //boolean searching, found = false, foundLast = false, vertical = false, verticalLast = false;


    @Override
    public void runOpMode() throws InterruptedException {
        dumbBot.init2();
        dumbBot.slidereset2();

        telemetry.setMsTransmissionInterval(50);

        highColor = hardwareMap.get(RevColorSensorV3.class, "highColor");

        lowColor = hardwareMap.get(RevColorSensorV3.class, "lowColor");
        lowhsv = new float[3];

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.updatePythonInputs(1,0,0,0,0,0,0,0);

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

        time = new Timer();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        limelight.start();
        limelight.pipelineSwitch(0);
        changeState("init");
        mode = 3;

        while (opModeIsActive()) {
            drive.moveInTeleop(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    Math.min(Math.pow(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.right_stick_x, 2), 0.5), 1)
            );

            if (mode == 0) gamepad1.setLedColor(255, 0, 0, 10000);
            else if (mode == 1) gamepad1.setLedColor(255, 255, 0, 10000);
            else if (mode == 2) gamepad1.setLedColor(0, 0, 255, 10000);
            else gamepad1.setLedColor(255, 255, 255, 10000);

            switch (state) {
                case "init":
                    if (!wasLast) {
                        wasLast = true;
                        
                        anvilSlidePos = 0;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 1;
                        highTurnPos = 0.225;
                        highBonkPos = 0.9;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("initToLime");
                    }
                    if (gamepad1.dpad_up && !up1Last) {
                        changeState("initToTransferLow");
                    }
                    if (gamepad1.dpad_left && !left1Last) {
                        changeState("initToEmergencyLowSpit");
                    }
                    if (gamepad1.dpad_right && !right1Last) {
                        changeState("initToEmergencyHighSpit");
                    }
                    if (gamepad1.b && !b1Last){
                        changeState("initToSpecGrabOpen");
                    }
                    break;
                case "initToLime":
                    if (!wasLast) {
                        wasLast = true;
                        
                        anvilSlidePos = 0;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.2;
                        lowBonkPos = 0.4;
                        lowRotPos = 0.84;
                        lowClawPos = 1;
                        highTurnPos = 0.225;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 100) {
                        changeState("limeDormant");
                    }
                    break;
                case "limeToInit":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.2;
                        lowBonkPos = 0.4;
                        lowRotPos = 0.84;
                        lowClawPos = 1;
                        highTurnPos = 0.225;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 100) {
                        changeState("init");
                    }
                    break;
                case "limeDormant":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.2;
                        lowBonkPos = 0.55;
                        lowRotPos = 0.84;
                        lowClawPos = 1;
                        highTurnPos = 0.225;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.92;
                        mode = 3;
                    }

                    if (gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right) {
                        changeState("limeSearching");
                        mode = gamepad1.dpad_left ? 0 : gamepad1.dpad_up ? 1 : 2;
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("limeToInit");
                    }
                    break;
                case "limeSearching":
                    if (!wasLast) {
                        limelight.updatePythonInputs(mode,0,0,0,0,0,0,0);
                        wasLast = true;

                        hammerSlidePos = 0;
                        lowTurnPos = 0.2;
                        lowBonkPos = 0.55;
                        lowRotPos = 0.84;
                        lowClawPos = 0;
                        highTurnPos = 0.225;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.92;
                    }

                    result = limelight.getLatestResult();
                    if (result != null) {
                        array = result.getPythonOutput();
                        if (array[0] == 1) {
                            changeState("limeCheck");
                        } else {
                            if (anvilSlidePos > -400) anvilSlidePos -= 2;
                            else anvilSlidePos = 0;
                        }
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("limeDormant");
                    }
                    break;
                case "limeCheck":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = anvilSlidePos;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.2;
                        lowBonkPos = 0.55;
                        lowRotPos = 0.84;
                        lowClawPos = 0;
                        highTurnPos = 0.225;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.92;
                    }

                    result = limelight.getLatestResult();
                    if (result != null) {
                        array = result.getPythonOutput();
                        if (array[0] == 1) {
                            LimelightYOffsets.add(array[4]);
                            LimelightXOffsets.add(array[3]);
                            LimelightThetas.add(array[1]);
                        } else {
                            changeState("limeSearching");
                        }
                    }

                    if (time.getElapsedTime() > 200) {
                        changeState("limeFound");
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("limeDormant");
                    }
                    break;
                case "limeFound":
                    if (!wasLast) {
                        wasLast = true;

                        desiredYIn = dumbBot.averageLastContents(LimelightYOffsets, 5) + limeLength;
                        desiredXIn = dumbBot.averageLastContents(LimelightXOffsets, 5) + limeLengthX;
                        lowTurnAngleDeg = Math.toDegrees(Math.asin(desiredXIn / bonkLength));
                        lowRotAngleDeg = lowTurnAngleDeg + dumbBot.averageLastContents(LimelightThetas, 5);
                        anvilChangeIn = desiredYIn - Math.sqrt(Math.pow(bonkLength, 2) - Math.pow(desiredXIn, 2));

                        anvilSlidePos = (int) (anvilChangeIn * inchPerAnvil) + dumbBot.slide3.getCurrentPosition();
                        hammerSlidePos = 0;
                        lowTurnPos = 0.54 + lowTurnAngleDeg / 180 * 0.68;
                        lowBonkPos = 0.55;
                        lowRotPos = 0.18 + lowRotAngleDeg / 180 * 0.68;
                        lowClawPos = 0;
                        highTurnPos = 0.225;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.36;
                    }

                    if (time.getElapsedTime() > 600) {
                        changeState("limePreBite");
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("limeDormant");
                    }
                    break;
                case "limePreBite":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = anvilSlidePos;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.54 + lowTurnAngleDeg / 180 * 0.68;
                        lowBonkPos = 0.72;
                        lowRotPos = 0.18 + lowRotAngleDeg / 180 * 0.68;
                        lowClawPos = 0;
                        highTurnPos = 0.225;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.36;
                    }

                    if (time.getElapsedTime() > 200) {
                        changeState("limeBite");
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("limeDormant");
                    }
                    break;
                case "limeBite":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = anvilSlidePos;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.54 + lowTurnAngleDeg / 180 * 0.68;
                        lowBonkPos = 0.72;
                        lowRotPos = 0.18 + lowRotAngleDeg / 180 * 0.68;
                        lowClawPos = 1;
                        highTurnPos = 0.225;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.36;
                    }

                    if (time.getElapsedTime() > 200) {
                        changeState("limeRevCheck");
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("limeDormant");
                    }
                    break;
                case "limeRevCheck":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = anvilSlidePos;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.54;
                        lowBonkPos = 0.55;
                        lowRotPos = 0.84;
                        lowClawPos = 1;
                        highTurnPos = 0.225;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.7;
                    }

                    Color.RGBToHSV(
                            (int) (lowColor.red() * 255.0 / 1023),
                            (int) (lowColor.green() * 255.0 / 1023),
                            (int) (lowColor.blue() * 255.0 / 1023),
                            lowhsv
                    );
                    lowHue = lowhsv[0];

                    if (mode == 0 && (lowHue < 30 || lowHue > 330)) {
                        telemetry.addLine("success");
                        if (time.getElapsedTime() > 600) {
                            changeState("limeHolding");
                        }
                    } else if (mode == 1 && (lowHue < 85 && lowHue > 50)) {
                        telemetry.addLine("success");
                        if (time.getElapsedTime() > 600) {
                            changeState("limeHolding");
                        }
                    } else if (mode == 2 && (lowHue < 230 && lowHue > 210)) {
                        telemetry.addLine("success");
                        if (time.getElapsedTime() > 600) {
                            changeState("limeHolding");
                        }
                    } else {
                        telemetry.addLine("fail");
                        if (time.getElapsedTime() > 600) {
                            changeState("limeSearching");
                        }
                    }
                    break;
                case "limeHolding":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = -200;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.54;
                        lowBonkPos = 0.55;
                        lowRotPos = 0.84;
                        lowClawPos = 1;
                        highTurnPos = 0.225;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.36;
                    }

                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("limeToInit");
                    }
                    if (gamepad1.dpad_right && !right1Last) {
                        changeState("limeSearching");
                    }
                    if (gamepad1.dpad_up && !up1Last) {
                        changeState("limeToTransfer1");
                    }
                    break;
                case "limeToTransfer1":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = -500;
                        hammerSlidePos = 800;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 1;
                        highTurnPos = 0.16;
                        highBonkPos = 1;
                        highClawPos = 0;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 500) {
                        changeState("limeToTransfer2");
                    }
                    break;
                case "limeToTransfer2":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 800;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 1;
                        highTurnPos = 0.16;
                        highBonkPos = 1;
                        highClawPos = 0;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 500) {
                        changeState("transferLow");
                    }
                    break;
                case "initToTransferLow":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 800;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 1;
                        highTurnPos = 0.225;
                        highBonkPos = 0.9;
                        highClawPos = 0;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 500) {
                        changeState("transferLow");
                    }
                    break;
                case "transferHighToInit":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 800;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 0;
                        highTurnPos = 0.225;
                        highBonkPos = 0.9;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 500) {
                        changeState("init");
                    }
                    break;
                case "transferLow":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 610;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 1;
                        highTurnPos = 0.16;
                        highBonkPos = 1;
                        highClawPos = 0;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 500) {
                        changeState("transferBoth");
                    }
                    break;
                case "transferBoth":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 610;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 1;
                        highTurnPos = 0.16;
                        highBonkPos = 1;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 500) {
                        changeState("transferHigh");
                    }
                    break;
                case "transferHigh":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 610;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 0;
                        highTurnPos = 0.16;
                        highBonkPos = 1;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("transferHighToInit");
                    }
                    if (gamepad1.dpad_up && !up1Last) {
                        changeState("transferToSamplePlace");
                    }
                    if (gamepad1.dpad_right && !right1Last) {
                        changeState("transferHighToSpecDropOff");
                    }
                    break;
                case "transferToSamplePlace":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 2120;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 0;
                        highTurnPos = 0.16;
                        highBonkPos = 0.5;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 300) {
                        changeState("samplePlace");
                    }
                    break;
                case "samplePlace":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 2120;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 1;
                        highTurnPos = 0.48;
                        highBonkPos = 0.67;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("samplePlaceDropToInit");
                    }
                    if (gamepad1.dpad_up && !up1Last) {
                        changeState("samplePlaceDrop");
                    }
                    break;
                case "samplePlaceDrop":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 2120;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 1;
                        highTurnPos = 0.48;
                        highBonkPos = 0.67;
                        highClawPos = 0;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (gamepad1.dpad_up && !up1Last) {
                        changeState("samplePlaceDropToInit");
                    }
                    break;
                case "samplePlaceDropToInit":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 2120;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 1;
                        highTurnPos = 0.18;
                        highBonkPos = 0.5;
                        highClawPos = 0;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 200) {
                        changeState("init");
                    }
                    break;
                case "transferHighToSpecDropOff":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 400;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 0;
                        highTurnPos = 0.45;
                        highBonkPos = 0.2;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 400) {
                        changeState("specDropOff");
                    }
                    break;
                case "specDropOff":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 200;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 1;
                        highTurnPos = 0.45;
                        highBonkPos = 0.2;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (gamepad1.dpad_right && !right1Last) {
                        changeState("specDropOffDrop");
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("init");
                    }
                    break;
                case "specDropOffDrop":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 200;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 1;
                        highTurnPos = 0.45;
                        highBonkPos = 0.2;
                        highClawPos = 0;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (gamepad1.dpad_down && !down1Last) {
                        changeState("init");
                    }
                    break;
                case "initToEmergencyLowSpit":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.54;
                        lowBonkPos = 0.55;
                        lowRotPos = 0.51;
                        lowClawPos = 1;
                        highTurnPos = 0.2;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 250) {
                        changeState("emergencyLowSpit");
                    }
                    break;
                case "emergencyLowSpit":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.54;
                        lowBonkPos = 0.55;
                        lowRotPos = 0.51;
                        lowClawPos = 0;
                        highTurnPos = 0.2;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 150) {
                        changeState("init");
                    }
                    break;
                case "initToEmergencyHighSpit":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 1;
                        highTurnPos = 0.3;
                        highBonkPos = 0.9;
                        highClawPos = 1;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 250) {
                        changeState("emergencyHighSpit");
                    }
                    break;
                case "emergencyHighSpit":
                    if (!wasLast) {
                        wasLast = true;

                        anvilSlidePos = 0;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 1;
                        highTurnPos = 0.3;
                        highBonkPos = 0.9;
                        highClawPos = 0;
                        snifferPos = 0.36;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 150) {
                        changeState("init");
                    }
                    break;
            }
            down1Last = gamepad1.dpad_down;
            up1Last = gamepad1.dpad_up;
            right1Last = gamepad1.dpad_right;
            left1Last = gamepad1.dpad_left;
            b1Last = gamepad1.b;

            dumbBot.slide3.setTargetPosition(anvilSlidePos);
            dumbBot.slide1.setTargetPosition(hammerSlidePos);
            dumbBot.slide2.setTargetPosition(hammerSlidePos);
            dumbBot.lowTurn.setPosition(lowTurnPos);
            dumbBot.lowBonk.setPosition(lowBonkPos);
            dumbBot.lowRot.setPosition(lowRotPos);
            dumbBot.lowClaw.setPosition(lowClawPos);
            dumbBot.highTurnB.setPosition(highTurnPos);
            dumbBot.highTurnT.setPosition(highTurnPos);
            dumbBot.highBonkR.setPosition(highBonkPos);
            dumbBot.highBonkL.setPosition(highBonkPos);
            dumbBot.highClaw.setPosition(highClawPos);
            dumbBot.sniffer.setPosition(snifferPos);

            telemetry.addData("state", state);
            telemetry.addData("mode", mode);
            telemetry.addData("python output", array != null ? array[0] : "nada");
            telemetry.update();
        }
        limelight.stop();
    }
    public void changeState(String newState){
        state = newState;
        wasLast = false;
        time.resetTimer();
    }
}
