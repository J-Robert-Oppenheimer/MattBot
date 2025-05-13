package org.firstinspires.ftc.teamcode.ExampleOpmodes;
//jizzle
import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
//change
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "UTICARED")
@Config

public class UTICARED extends LinearOpMode {
    public String lowState, highState;
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
    float[] lowhsv, highhsv;
    double lowHue, highHue;
    LLResult result;
    boolean wasLastH, wasLastL;
    public boolean down1Last = false, up1Last = false, right1Last = false, left1Last = false, ps1Last = false, y1Last = false, a1Last = false, b1Last = false, x1Last = false, share1Last = false, option1Last = false, rb1Last = false, lb1Last = false;
    double desiredYIn;
    double desiredXIn;
    double lowTurnAngleDeg;
    double lowRotAngleDeg;
    double anvilChangeIn;
    double bonkLength = 8;
    double limeLength = 4.5;
    double limeLengthX = -0.5;
    double inchPerAnvil = -384.5 / (0.7121771654*2* Math.PI);
    public DcMotor leftFront, rightFront, leftBack, rightBack;
    MecanumDrive drive;
    double lowPower;
    double highPower;

    private Timer lowTime, highTime;

    //boolean searching, found = false, foundLast = false, vertical = false, verticalLast = false;


    @Override
    public void runOpMode() throws InterruptedException {
        dumbBot.init2();
        dumbBot.slidereset2();

        telemetry.setMsTransmissionInterval(50);

        highColor = hardwareMap.get(RevColorSensorV3.class, "highColor");

        lowColor = hardwareMap.get(RevColorSensorV3.class, "lowColor");
        lowhsv = new float[3];
        highhsv = new float[3];

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
        lowPower = 1;
        highPower = 1;

        lowTime = new Timer();
        highTime = new Timer();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        limelight.start();
        limelight.pipelineSwitch(0);
        changeStateLow("init");
        changeStateHigh("init");
        mode = 3;

        while (opModeIsActive()) {
            drive.moveInTeleop(
                    gamepad2.left_stick_x,
                    gamepad2.left_stick_y,
                    gamepad2.right_stick_x,
                    Math.min(Math.pow(Math.pow(gamepad2.left_stick_x, 2) + Math.pow(gamepad2.left_stick_y, 2) + Math.pow(gamepad2.right_stick_x, 2), 0.5), 1) * Math.min(lowPower, highPower)
            );

            if (mode == 0) gamepad1.setLedColor(255, 0, 0, 10000);
            else if (mode == 1) gamepad1.setLedColor(255, 255, 0, 10000);
            else if (mode == 2) gamepad1.setLedColor(0, 0, 255, 10000);
            else gamepad1.setLedColor(255, 255, 255, 10000);

            if (gamepad1.right_trigger > 0){
                hammerSlidePos+= (int)(gamepad2.right_trigger*10);
                hammerSlidePos = Math.min(hammerSlidePos,2050);
            }
            else if (gamepad1.left_trigger > 0){
                hammerSlidePos-= (int)(gamepad2.left_trigger*10);
                hammerSlidePos = Math.max(hammerSlidePos,0);
            }

            switch (lowState) {
                case "init":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 1;
                        snifferPos = 0.36;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (gamepad1.dpad_down && !down1Last) {
                        changeStateLow("initToLime");
                    }
                    if (gamepad1.share && !share1Last) {
                        changeStateLow("initToEmergencyLowSpit");
                    }
//                    if (gamepad1.a && !a1Last) {
//                        changeStateLow("manualSearchingUp");
//                    }
                    break;
                case "initToLime":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.2;
                        lowBonkPos = 0.4;
                        lowRotPos = 0.84;
                        lowClawPos = 1;
                        snifferPos = 0.36;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (lowTime.getElapsedTime() > 100) {
                        changeStateLow("limeDormant");
                    }
                    break;
                case "limeToInit":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.4;
                        lowRotPos = 0.84;
                        lowClawPos = 1;
                        snifferPos = 0.36;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (lowTime.getElapsedTime() > 150) {
                        changeStateLow("init");
                    }
                    break;
                case "limeDormant":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.2;
                        lowBonkPos = 0.5;
                        lowRotPos = 0.84;
                        lowClawPos = 1;
                        snifferPos = 0.92;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (gamepad1.y && !y1Last) {
                        if (highState != "SpecPlaceOpen") {
                            changeStateLow("limeToTransfer");
                            changeStateHigh("limeToTransfer");
                        } else {
                            changeStateLow("SpecPlaceToTransfer");
                            changeStateHigh("SpecPlaceToTransfer");
                        }
                    }
                    if (gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right) {
                        changeStateLow("limeSearching");
                        mode = gamepad1.dpad_left ? 0 : gamepad1.dpad_up ? 1 : 2;
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeStateLow("limeToInit");
                    }
                    break;
                case "limeSearching":
                    if (!wasLastL) {
                        limelight.updatePythonInputs(mode,0,0,0,0,0,0,0);
                        wasLastL = true;

                        lowTurnPos = 0.2;
                        lowBonkPos = 0.5;
                        lowRotPos = 0.84;
                        lowClawPos = 0;
                        snifferPos = 0.92;
                        lowPower = 0.25;
                    }

                    result = limelight.getLatestResult();
                    if (result != null) {
                        array = result.getPythonOutput();
                        if (array[0] == 1) {
                            changeStateLow("limeCheck");
                        } else {
                            if (anvilSlidePos > -1050) anvilSlidePos -= 8;
                            else anvilSlidePos = 0;
                        }
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeStateLow("limeDormant");
                    }
                    break;
                case "limeCheck":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = anvilSlidePos;
                        lowTurnPos = 0.2;
                        lowBonkPos = 0.5;
                        lowRotPos = 0.84;
                        lowClawPos = 0;
                        snifferPos = 0.92;
                        lowPower = 0.25;
                    }

                    result = limelight.getLatestResult();
                    if (result != null) {
                        array = result.getPythonOutput();
                        if (array[0] == 1) {
                            LimelightYOffsets.add(array[4]);
                            LimelightXOffsets.add(array[3]);
                            LimelightThetas.add(array[1]);
                        } else {
                            changeStateLow("limeSearching");
                        }
                    }

                    if (lowTime.getElapsedTime() > 150) {
                        changeStateLow("limeFound");
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeStateLow("limeDormant");
                    }
                    break;
                case "limeFound":
                    if (!wasLastL) {
                        wasLastL = true;

                        desiredYIn = dumbBot.averageLastContents(LimelightYOffsets, 5) + limeLength;
                        desiredXIn = dumbBot.averageLastContents(LimelightXOffsets, 5) + limeLengthX;
                        lowTurnAngleDeg = Math.toDegrees(Math.asin(desiredXIn / bonkLength));
                        lowRotAngleDeg = lowTurnAngleDeg + dumbBot.averageLastContents(LimelightThetas, 5);
                        anvilChangeIn = desiredYIn - Math.sqrt(Math.pow(bonkLength, 2) - Math.pow(desiredXIn, 2));

                        anvilSlidePos = (int) (anvilChangeIn * inchPerAnvil) + dumbBot.slide3.getCurrentPosition();
                        lowTurnPos = 0.54 + lowTurnAngleDeg / 180 * 0.68;
                        lowBonkPos = 0.5;
                        lowRotPos = 0.18 + lowRotAngleDeg / 180 * 0.68;
                        lowClawPos = 0;
                        snifferPos = 0.7;
                        lowPower = 0.25;
                    }

                    if (lowTime.getElapsedTime() > 600) {
                        changeStateLow("limePreBite");
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeStateLow("limeDormant");
                    }
                    break;
                case "limePreBite":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = anvilSlidePos;
                        lowTurnPos = 0.54 + lowTurnAngleDeg / 180 * 0.68;
                        lowBonkPos = 0.72;
                        lowRotPos = 0.18 + lowRotAngleDeg / 180 * 0.68;
                        lowClawPos = 0;
                        snifferPos = 0.7;
                        lowPower = 0.25;
                    }

                    if (lowTime.getElapsedTime() > 200) {
                        changeStateLow("limeBite");
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeStateLow("limeDormant");
                    }
                    break;
                case "limeBite":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = anvilSlidePos;
                        lowTurnPos = 0.54 + lowTurnAngleDeg / 180 * 0.68;
                        lowBonkPos = 0.72;
                        lowRotPos = 0.18 + lowRotAngleDeg / 180 * 0.68;
                        lowClawPos = 1;
                        snifferPos = 0.7;
                        lowPower = 0.25;
                    }

                    if (lowTime.getElapsedTime() > 200) {
                        changeStateLow("limeRevCheck");
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeStateLow("limeDormant");
                    }
                    break;
                case "limeRevCheck":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = anvilSlidePos;
                        lowTurnPos = 0.54;
                        lowBonkPos = 0.5;
                        lowRotPos = 0.84;
                        lowClawPos = 1;
                        snifferPos = 0.7;
                        lowPower = 0.25;
                    }

                    Color.RGBToHSV(
                            (int) (lowColor.red() * 255.0 / 1023),
                            (int) (lowColor.green() * 255.0 / 1023),
                            (int) (lowColor.blue() * 255.0 / 1023),
                            lowhsv
                    );
                    lowHue = lowhsv[0];

                    if (mode == 0 && (lowHue < 30 || lowHue > 330)) {
                        if (lowTime.getElapsedTime() > 400) {
                            gamepad2.rumble(1000);
                            changeStateLow("limeHolding");
                        }
                    } else if (mode == 1 && (lowHue < 85 && lowHue > 50)) {
                        if (lowTime.getElapsedTime() > 400) {
                            gamepad2.rumble(1000);
                            changeStateLow("limeHolding");
                        }
                    } else if (mode == 2 && (lowHue < 230 && lowHue > 210)) {
                        if (lowTime.getElapsedTime() > 400) {
                            gamepad2.rumble(1000);
                            changeStateLow("limeHolding");
                        }
                    } else {
                        if (lowTime.getElapsedTime() > 400) {
                            changeStateLow("limeSearching");
                        }
                    }
                    break;
                case "limeHolding":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.54;
                        lowBonkPos = 0.5;
                        lowRotPos = 0.84;
                        lowClawPos = 1;
                        snifferPos = 0.7;
                        lowPower = 1;
                    }

                    if (gamepad1.dpad_down && !down1Last) {
                        changeStateLow("limeDormant");
                    }
                    if (gamepad1.y && !y1Last) {
                        if (highState != "SpecPlaceOpen") {
                            changeStateLow("limeToTransfer");
                            changeStateHigh("limeToTransfer");
                        } else {
                            changeStateLow("SpecPlaceToTransfer");
                            changeStateHigh("SpecPlaceToTransfer");
                        }
                    }
                    if (gamepad1.share && !share1Last) {
                        changeStateLow("limeDrop");
                    }
                    break;
                case "limeDrop":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.54;
                        lowBonkPos = 0.5;
                        lowRotPos = 0.84;
                        lowClawPos = 0;
                        snifferPos = 0.7;
                        lowPower = 1;
                    }

                    if (lowTime.getElapsedTime() > 250) {
                        changeStateLow("limeDormant");
                    }
                case "limeToTransfer":
                    if (!wasLastL) {
                        wasLastL = true;
                        anvilSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 1;
                        snifferPos = 0.36;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (lowTime.getElapsedTime() > 500) {
                        changeStateLow("transferLow");
                    }
                    break;

                case "transferLow":
                    if (!wasLastL) {
                        wasLastL = true;
                        anvilSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 1;
                        snifferPos = 0.36;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (lowTime.getElapsedTime() > 200) {
                        changeStateLow("transferBoth");
                    }
                    break;

                case "transferBoth":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 1;
                        snifferPos = 0.36;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (lowTime.getElapsedTime() > 300) {
                        changeStateLow("transferHigh");
                    }
                    break;
                case "transferHigh":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 0;
                        snifferPos = 0.36;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (gamepad1.a && !a1Last) {
                        changeStateLow("transferHighToInit");
                    }
                    if (gamepad1.x && !x1Last) {
                        changeStateLow("transferHighToInit");
                    }
                    if (gamepad1.dpad_down && !down1Last) {
                        changeStateLow("init");
                    }
                    if (gamepad1.share && !share1Last) {
                        changeStateLow("initToEmergencyLowSpit");
                    }
                    break;
                case "transferHighToInit":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.33;
                        lowClawPos = 0;
                        snifferPos = 0.36;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (lowTime.getElapsedTime() > 250) {
                        changeStateLow("init");
                    }
                    break;
                case "initToEmergencyLowSpit":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.54;
                        lowBonkPos = 0.55;
                        lowRotPos = 0.51;
                        lowClawPos = 1;
                        snifferPos = 0.36;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (lowTime.getElapsedTime() > 400) {
                        changeStateLow("emergencyLowSpit");
                    }
                    break;
                case "emergencyLowSpit":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.54;
                        lowBonkPos = 0.55;
                        lowRotPos = 0.51;
                        lowClawPos = 0;
                        snifferPos = 0.36;
                        lowPower = 1;
                        mode = 3;
                    }

                    if (lowTime.getElapsedTime() > 150) {
                        changeStateLow("init");
                    }
                    break;
                case "SpecPlaceToTransfer":
                    if (!wasLastL) {
                        wasLastL = true;

                        anvilSlidePos = 0;
                        lowTurnPos = 0.76;
                        lowBonkPos = 0.17;
                        lowRotPos = 0.37;
                        lowClawPos = 1;
                        snifferPos = 0.36;
                        lowPower = 0.35;
                        mode = 3;
                    }

                    if (lowTime.getElapsedTime() > 200) {
                        changeStateLow("limeToTransfer");
                    }
//                case "manualSearchingUp":
//                    if (!wasLastL) {
//                        wasLastL = true;
//
//                        lowTurnPos = 0.53;
//                        lowBonkPos = 0.63;
//                        lowClawPos = 0;
//                        snifferPos = 0.36;
//                        lowPower = 0.5;
//                        mode = 3;
//                    }
//                    if (Math.abs(gamepad1.left_stick_y) > 0) {
//                        anvilSlidePos += (int) (gamepad1.left_stick_y * 25);
//                        anvilSlidePos =  Math.max(Math.min(anvilSlidePos, 0), -1050);
//                    }
//                    if (Math.abs(gamepad1.left_stick_x) > 0) {
//                        lowRotPos += (int) (gamepad1.left_stick_y * 0.05);
//                        lowRotPos =  Math.max(Math.min(anvilSlidePos, 0.86),0.18);
//                    }
//
//                    if (gamepad1.x) {
//                        changeStateLow("manualSearchingDown");
//                    }
//                    if (gamepad1.dpad_down && !down1Last) {
//                        changeStateLow("init");
//                    }
//                    break;
//
//                case "manualSearchingDown":
//                    if (!wasLastL) {
//                        wasLastL = true;
//
//
//                        lowTurnPos = 0.76;
//                        lowBonkPos = 0.72;
//                        lowRotPos = lowRotPos;
//                        lowClawPos = 0;
//                        snifferPos = 0.36;
//                        lowPower = 0.5;
//                        mode = 3;
//                    }
//                    if (Math.abs(gamepad1.left_stick_y) > 0) {
//                        anvilSlidePos += (int) (gamepad1.left_stick_y * 25);
//                        anvilSlidePos =  Math.max(Math.min(anvilSlidePos, 0), -1050);
//                    }
//
//                    if (gamepad1.a) {
//                        changeStateLow("manualSearchingClosed");
//                    }
//                    if (gamepad1.dpad_down && !down1Last) {
//                        changeStateLow("init");
//                    }
//                    break;
//
//                case "manualSearchingClosed":
//                    if (!wasLastL) {
//                        wasLastL = true;
//
//                        anvilSlidePos = anvilSlidePos;
//                        lowTurnPos = 0.76;
//                        lowBonkPos = 0.72;
//                        lowRotPos = lowRotPos;
//                        lowClawPos = 1;
//                        snifferPos = 0.36;
//                        lowPower = 0.5;
//                        mode = 3;
//                    }
//
//                    if (gamepad1.a) {
//                        changeStateLow("limeHolding");
//                    }
//                    if (gamepad1.dpad_down && !down1Last) {
//                        changeStateLow("init");
//                    }
//                    break;

            }

//TODO line IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII

            switch (highState) {
                case "init":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 0;
                        highTurnPos = 0.225;
                        highBonkPos = 0.9;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }
                    if (lowState != "init") {
                        changeStateHigh("lime");
                    }
                    if (gamepad1.options && !option1Last) {
                        changeStateHigh("initToEmergencyHighSpit");
                    }
                    if (gamepad1.right_bumper && !rb1Last) {
                        changeStateHigh("SpecGrabOpen");
                    }
                    if (gamepad1.a && !a1Last) {
                        changeStateHigh("transferHighToSamplePlace");
                    }
                    if (gamepad1.x && !x1Last) {
                        changeStateHigh("transferHighToSpecDropOff");
                    }
                    break;

                case "lime":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 0;
                        highTurnPos = 0.2;
                        highBonkPos = 0.7;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }

                    if (lowState == "init") {
                        changeStateHigh("init");
                    }
                    if (gamepad1.options && !option1Last) {
                        changeStateHigh("initToEmergencyHighSpit");
                    }
                    if (gamepad1.right_bumper && !rb1Last) {
                        changeStateHigh("SpecGrabOpen");
                    }
                    if (gamepad1.a && !a1Last) {
                        changeStateHigh("transferHighToSamplePlace");
                    }
                    if (gamepad1.x && !x1Last) {
                        changeStateHigh("transferHighToSpecDropOff");
                    }
                    break;
                case "limeToTransfer":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 900;
                        highTurnPos = 0.16;
                        highBonkPos = 0.9;
                        highClawPos = 0;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 500) {
                        changeStateHigh("transferLow");
                    }
                    break;

                case "transferLow":
                    if (!wasLastH) {
                        wasLastH = true;

//                        hammerSlidePos = 610;
//                        highTurnPos = 0.16;
                        highBonkPos = 1;
                        highClawPos = 0;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 200) {
                        changeStateHigh("transferBoth");
                    }
                    break;
                case "transferBoth":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 610;
                        highTurnPos = 0.16;
                        highBonkPos = 1;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 300) {
                        changeStateHigh("transferHigh");
                    }
                    break;
                case "transferHigh":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 610;
                        highTurnPos = 0.16;
                        highBonkPos = 1;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                        gamepad1.rumble(1000);
                    }

                    if (gamepad1.options && !option1Last) {
                        changeStateHigh("initToEmergencyHighSpit");
                    }
                    if (gamepad1.a && !a1Last) {
                        changeStateHigh("transferHighToSamplePlace");
                    }
                    if (gamepad1.x && !x1Last) {
                        changeStateHigh("transferHighToSpecDropOff");
                    }
                    if (gamepad1.b && !b1Last) {
                        changeStateHigh("transferHighToInit");
                    }
                    break;
                case "transferHighToInit":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 800;
                        highTurnPos = 0.2;
                        highBonkPos = 0.9;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 500) {
                        changeStateHigh("init");
                    }
                    break;
                case "transferHighToSamplePlace":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 2120;
                        highTurnPos = 0.16;
                        highBonkPos = 0.5;
                        highClawPos = 1;
                        highPower = 0.35;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 300) {
                        changeStateHigh("samplePlace");
                    }
                    break;
                case "samplePlace":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 2120;
                        highTurnPos = 0.75;
                        highBonkPos = 0.67;
                        highClawPos = 1;
                        highPower = 0.35;
                        mode = mode;
                    }

                    if (gamepad1.b && !b1Last) {
                        changeStateHigh("samplePlaceDropToInit");
                    }
                    if (gamepad1.a && !a1Last) {
                        changeStateHigh("samplePlaceDrop");
                    }
                    break;
                case "samplePlaceDrop":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 2120;
                        highTurnPos = 0.75;
                        highBonkPos = 0.67;
                        highClawPos = 0;
                        highPower = 0.35;
                        mode = mode;
                    }

                    if (gamepad1.b && !b1Last) {
                        changeStateHigh("samplePlaceDropToInit");
                    }
                    break;
                case "samplePlaceDropToInit":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 2120;
                        highTurnPos = 0.18;
                        highBonkPos = 0.5;
                        highClawPos = 1;
                        highPower = 0.25;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 350) {
                        if (lowState == "limeDormant") {
                            changeStateHigh("lime");
                        } else {
                            changeStateHigh("init");
                        }
                    }
                    break;
                case "transferHighToSpecDropOff":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 400;
                        highTurnPos = 0.45;
                        highBonkPos = 0.2;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 400) {
                        changeStateHigh("specDropOff");
                    }
                    break;
                case "specDropOff":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 200;
                        highTurnPos = 0.45;
                        highBonkPos = 0.2;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }

                    if (gamepad1.x && !x1Last) {
                        changeStateHigh("specDropOffDrop");
                    }
                    if (gamepad1.b && !b1Last) {
                        if (lowState == "limeDormant") {
                            changeStateHigh("lime");
                        } else {
                            changeStateHigh("init");
                        }
                    }
                    break;
                case "specDropOffDrop":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 200;
                        highTurnPos = 0.45;
                        highBonkPos = 0.2;
                        highClawPos = 0;
                        highPower = 1;
                        mode = mode;
                    }

                    if (gamepad1.b && !b1Last) {
                        if (lowState == "limeDormant") {
                            changeStateHigh("lime");
                        } else {
                            changeStateHigh("init");
                        }
                    }
                    if (gamepad1.right_bumper && !rb1Last) {
                        changeStateHigh("SpecGrabOpen");
                    }
                    break;
                case "initToEmergencyHighSpit":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 0;
                        highTurnPos = 0.3;
                        highBonkPos = 0.9;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 250) {
                        changeStateHigh("emergencyHighSpit");
                    }
                    break;
                case "emergencyHighSpit":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 0;
                        highTurnPos = 0.3;
                        highBonkPos = 0.9;
                        highClawPos = 0;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 150) {
                        if (lowState == "limeDormant") {
                            changeStateHigh("lime");
                        } else {
                            changeStateHigh("init");
                        }
                    }
                    break;

                case "SpecGrabOpen":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 600;
                        highTurnPos = 0.2;
                        highBonkPos = 0.15;
                        highClawPos = 0;
                        highPower = 0.5;
                        mode = mode;
                    }
                    telemetry.addData("ds: ", highColor.getDistance(DistanceUnit.CM));
                    if (highTime.getElapsedTime() > 500) {
                        if (highColor.getDistance(DistanceUnit.CM) < 2) {
                            changeStateHigh("SpecGrabClosed");
                        }
                    }
                    if (gamepad1.b && !b1Last) {
                        if (lowState == "limeDormant") {
                            changeStateHigh("lime");
                        } else {
                            changeStateHigh("init");
                        }
                    }
                    if (highTime.getElapsedTime() > 500) {
                        if (gamepad1.right_bumper && !rb1Last) {
                            changeStateHigh("SpecGrabClosed");
                        }
                    }
                    break;
                case "SpecGrabClosed":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 600;
                        highTurnPos = 0.2;
                        highBonkPos = 0.15;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }

                    Color.RGBToHSV(
                            (int) (highColor.red() * 255.0 / 1023),
                            (int) (highColor.green() * 255.0 / 1023),
                            (int) (highColor.blue() * 255.0 / 1023),
                            highhsv
                    );
                    highHue = highhsv[0];

                    if (highTime.getElapsedTime() > 200) {
                        if ((highHue < 30 || highHue > 330) || (highHue < 230 && highHue > 210)) {
                            changeStateLow("limeDormant");
                            changeStateHigh("SpecGrabToPlace");
                        } else {
                            changeStateHigh("SpecGrabOpen");
                        }
                    }
                    break;
                case "SpecGrabToPlace":
                    if (!wasLastH){
                        wasLastH = true;

                        hammerSlidePos = 780;
                        highTurnPos = 0.75;
                        highBonkPos = 0.15;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 600) {
                        changeStateHigh("SpecPlaceClosed");
                    }
                    break;
                case "SpecPlaceClosed":
                    if (!wasLastH){
                        wasLastH = true;

                        hammerSlidePos = 780;
                        highTurnPos = 0.75;
                        highBonkPos = 0.33;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }

                    Color.RGBToHSV(
                            (int) (highColor.red() * 255.0 / 1023),
                            (int) (highColor.green() * 255.0 / 1023),
                            (int) (highColor.blue() * 255.0 / 1023),
                            highhsv
                    );
                    highHue = highhsv[0];

                    if (gamepad1.left_bumper && !lb1Last) {
                        changeStateHigh("SpecPlaceClosedToOpen");
                        changeStateLow("limeSearching");
                        mode = 0;
                    }
                    if (gamepad1.right_bumper && !rb1Last) {
                        changeStateHigh("SpecPlaceToGrab");
                    }
                    break;
                case "SpecPlaceClosedToOpen":
                    if (!wasLastH){
                        wasLastH = true;

                        hammerSlidePos = 700;
                        highTurnPos = 0.75;
                        highBonkPos = 0.28;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 250) {
                        changeStateHigh("SpecPlaceOpen");
                    }
                    break;
                case "SpecPlaceOpen":
                    if (!wasLastH){
                        wasLastH = true;

                        hammerSlidePos = 700;
                        highTurnPos = 0.75;
                        highBonkPos = 0.28;
                        highClawPos = 0;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 100 && gamepad1.right_bumper && !rb1Last) {
                        changeStateHigh("SpecPlaceToGrab");
                    }
                    if (gamepad1.b && !b1Last) {
                        changeStateHigh("SpecPlaceToInit");
                    }
                    break;
                case "SpecPlaceToGrab":
                    if (!wasLastH){
                        wasLastH = true;

                        hammerSlidePos = 660;
                        highTurnPos = 0.2;
                        highBonkPos = 0.28;
                        highClawPos = 1;
                        highPower = 1;
                        mode = mode;
                    }
                    if (highTime.getElapsedTime() > 300) {
                        changeStateHigh("SpecGrabOpen");
                    }
                    break;
                case "SpecPlaceToInit":
                    if (!wasLastH){
                        wasLastH = true;

                        hammerSlidePos = 0;
                        highTurnPos = 0.20;
                        highBonkPos = 0.6;
                        highClawPos = 0;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 400) {
                        if (lowState == "limeDormant") {
                            changeStateHigh("lime");
                        } else {
                            changeStateHigh("init");
                        }
                    }
                    break;
                case "SpecPlaceToTransfer":
                    if (!wasLastH) {
                        wasLastH = true;

                        hammerSlidePos = 700;
                        highTurnPos = 0.16;
                        highBonkPos = 0.28;
                        highClawPos = 0;
                        highPower = 1;
                        mode = mode;
                    }

                    if (highTime.getElapsedTime() > 200) {
                        changeStateHigh("limeToTransfer");
                    }


            }



            down1Last = gamepad1.dpad_down;
            up1Last = gamepad1.dpad_up;
            right1Last = gamepad1.dpad_right;
            left1Last = gamepad1.dpad_left;
            b1Last = gamepad1.b;
            a1Last = gamepad1.a;
            y1Last = gamepad1.y;
            x1Last = gamepad1.x;
            ps1Last = gamepad1.ps;
            share1Last = gamepad1.share;
            option1Last = gamepad1.options;
            rb1Last = gamepad1.right_bumper;
            lb1Last = gamepad1.left_bumper;

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

            telemetry.addData("stateL", lowState);
            telemetry.addData("stateH", highState);
            telemetry.addData("mode", mode);
            telemetry.addData("python output", array != null ? array[0] : "nada");
            telemetry.addData("lowPower", lowPower);
            telemetry.addData("highPower", highPower);
            telemetry.update();
        }
        limelight.stop();
    }
    public void changeStateLow(String newState){
        lowState = newState;
        wasLastL = false;
        lowTime.resetTimer();
    }
    public void changeStateHigh(String newState){
        highState = newState;
        wasLastH = false;
        highTime.resetTimer();
    }
}
