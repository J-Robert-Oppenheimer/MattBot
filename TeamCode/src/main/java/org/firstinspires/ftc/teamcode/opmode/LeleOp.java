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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auton.dumber.MecanumDrive;
import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LeleOp", group = "oldteles")
@Config

public class LeleOp extends LinearOpMode {
    dumbMap dumbBot = new dumbMap(this);
    public int anvilSlidePos, hammerSlidePos;
    public String scrollPoses = "init";
    private Limelight3A limelight;
    ColorRangeSensor highColor, lowColor;
    double[] array;
    float[] hsv = new float[3];
    int iterCount, mode;
    LLResult result;
    ArrayList<Double> LimelightXOffsets = new ArrayList<>();
    ArrayList<Double> LimelightYOffsets = new ArrayList<>();
    ArrayList<Double> LimelightThetas = new ArrayList<>();
    boolean wasLast;
    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos, snifferPos, hue;
    public boolean a2Last = false, b2Last = false, x2Last = false, lowClawRot = true, lBumpLast = false, lime = true, psLast = false, fromHigh = false;
    private Follower follower;
    double desiredY;
    double desiredX;
    double lowTurnAngle;
    double lowRotAngle;
    double bonkLength = 8;
    double limeLength = 4;
    double inchPerAnvil = -145.1 / (0.7121771654*2* Math.PI);

    private Timer time;
    MecanumDrive drive;
    private final Pose startPose = new Pose(0,0,0);

    //boolean searching, found = false, foundLast = false, vertical = false, verticalLast = false;


    @Override
    public void runOpMode() throws InterruptedException {
        dumbBot.init2();
        dumbBot.slidereset2();
        highColor = hardwareMap.get(RevColorSensorV3.class, "highColor");
        lowColor = hardwareMap.get(RevColorSensorV3.class, "lowColor");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        drive = new MecanumDrive(dumbBot.leftFront, dumbBot.rightFront, dumbBot.leftBack, dumbBot.rightBack);

        time = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        limelight.setPollRateHz(100);
        limelight.start();

        follower.startTeleopDrive();

        while(opModeIsActive()) {
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

//____-__-----_--__--_--__-__-__-___---_-_-__-__-LINE-__-_--_-__--_--__-_-_-___-__-_____-________________-

            if (gamepad2.y) {
                scrollPoses = "init";
            }

            if (gamepad2.ps && !psLast){
                lime = !lime;
            }
            psLast = gamepad2.ps;

            if (mode == 0) {
                gamepad2.setLedColor(255, 0, 0, 10000);
            } else if (mode == 1) {
                gamepad2.setLedColor(255, 255, 0, 10000);
            } else if (mode == 2) {
                gamepad2.setLedColor(0, 0, 255, 10000);
            } else {
                gamepad2.setLedColor(255, 255, 255, 10000);
            }


            switch (scrollPoses) {
                case "init":
                    anvilSlidePos = 0;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.51;
                    lowBonkPos = 0.2;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.2;
                    highBonkPos = 0.9;
                    highClawPos = 1;
                    snifferPos = 0.305;
                    if (gamepad2.a && !a2Last) {
                        changePose("high bonk up");
                    } else if (gamepad2.b && !b2Last) {
                        changePose("spec grab open");
                    }
                    break;
                case "high bonk up":
                    anvilSlidePos = 0;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.51;
                    lowBonkPos = 0.2;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.18;
                    highBonkPos = 0.62;//up
                    highClawPos = 1;
                    if (time.getElapsedTime() > 500) {
                        if (!lime) changePose("intake up");
                        else changePose("lime pre prep");
                    }
                    break;
                case "intake up":
                    if (Math.abs(gamepad2.left_stick_y) > 0) {
                        anvilSlidePos += (int) (gamepad2.left_stick_y * 5);
                        anvilSlidePos =  Math.max(Math.min(anvilSlidePos, 0), -400);
                    }
                    hammerSlidePos = 0;
                    if (Math.abs(gamepad2.left_stick_x) > 0.5){
                        lowTurnPos += gamepad2.left_stick_x * 0.008;
                        lowTurnPos = Math.max(Math.min(lowTurnPos, 0.81), 0.35);
                    }
                    if (gamepad2.left_bumper && !lBumpLast) lowClawRot = !lowClawRot;
                    lowBonkPos = 0.65;
                    lowRotPos = lowClawRot? 0.51:0.89;
                    lowClawPos = 1;
                    highTurnPos = 0.18;
                    highBonkPos = 0.62;//up
                    highClawPos = 1;
                    if (gamepad2.x) {
                        changePose("intake down open");
                    }
                    break;
                case "intake down open":
                    if (Math.abs(gamepad2.left_stick_y) > 0) {
                        anvilSlidePos += (int) (gamepad2.left_stick_y * 10);
                        anvilSlidePos =  Math.max(Math.min(anvilSlidePos, 0), -400);
                    }
                    hammerSlidePos = 0;
                    if (Math.abs(gamepad2.left_stick_x) > 0.5){
                        lowTurnPos -= gamepad2.left_stick_x * 0.008;
                        lowTurnPos = Math.max(Math.min(lowTurnPos, 0.81), 0.35);
                    }
                    lowBonkPos = 0.73;
                    lowRotPos = lowRotPos;
                    lowClawPos = 1;
                    highTurnPos = 0.18;
                    highBonkPos = 0.62;//up
                    highClawPos = 1;
                    if (gamepad2.a && !a2Last) {
                        changePose("intake down closed");
                    }
                    else if (!gamepad2.x) {
                        changePose("intake up");
                    }
                    break;
                case "intake down closed":
                    anvilSlidePos = anvilSlidePos;
                    hammerSlidePos = 0;
                    lowTurnPos = lowTurnPos;
                    lowBonkPos = 0.73;
                    lowRotPos = lowRotPos;
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.62;//up
                    highClawPos = 1;

                    if (time.getElapsedTime() > 400) {
                        changePose("intake down raised");
                    }
                    break;

                case "intake down raised":
                    if (Math.abs(gamepad2.left_stick_y) > 0) {
                        anvilSlidePos += (int) (gamepad2.left_stick_y * 10);
                        anvilSlidePos =  Math.max(Math.min(anvilSlidePos, 0), -400);
                    }
                    hammerSlidePos = 0;
                    lowTurnPos = lowTurnPos;
                    lowBonkPos = 0.65;
                    if (Math.abs(gamepad2.left_stick_x) > 0.5){
                        lowTurnPos -= gamepad2.left_stick_x * 0.008;
                        lowTurnPos = Math.max(Math.min(lowTurnPos, 0.91), 0.16);
                    }
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.62;//up
                    highClawPos = 1;
                    if (gamepad2.b){
                        changePose("intake up");
                    }
                    if (gamepad2.a && !a2Last) {
                        changePose("low bonk up");
                    }
                    break;

                case "low bonk up":
                    if (time.getElapsedTime() > 300) {
                        anvilSlidePos = anvilSlidePos;
                        hammerSlidePos = 0;
                        lowTurnPos = lowTurnPos;
                        lowBonkPos = 0.2;
                        lowRotPos = lowRotPos;
                        lowClawPos = 0;
                        highTurnPos = 0.18;
                        highBonkPos = 0.62;//up
                        highClawPos = 1;
                        if (time.getElapsedTime() > 800) {
                            changePose("transfer 1");
                        }
                    }
                    break;
//_____________________________________________limeline_____________________________________

                case "lime pre prep":
                    anvilSlidePos = 0;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.55;
                    lowRotPos = 0.84;
                    lowClawPos = 1;
                    if (time.getElapsedTime() > 300){
                        changePose("lime prep");
                    }
                    break;

                case "lime prep":
                    anvilSlidePos = 0;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.55;
                    lowRotPos = 0.84;
                    lowClawPos = 1;
                    snifferPos = 0.855;
                    if (gamepad2.dpad_left) {
                        wasLast = false;
                        mode = 0;
                        changePose("lime search");
                    } else if (gamepad2.dpad_up) {
                        wasLast = false;
                        mode = 1;
                        changePose("lime search");
                    } else if (gamepad2.dpad_right) {
                        wasLast = false;
                        mode = 2;
                        changePose("lime search");
                    }

                    if (!lime) changePose("intake up");
                    break;


                case "lime search":
                    if (!wasLast) {
                        limelight.updatePythonInputs(mode, 0,0,0,0,0,0,0);
                        wasLast = true;
                    }
                    snifferPos = 0.855;
                    lowClawPos = 1;
                    lowRotPos = 0.84;
                    lowBonkPos = 0.55;
                    lowTurnPos = 0.81;

                    result = limelight.getLatestResult();
                    if (result != null) {
                        array = result.getPythonOutput();
                        if (array[0] == 1) {
                            changePose("lime check");
                            wasLast = false;
                        } else {
                            if (dumbBot.slide3.getCurrentPosition() > -400) anvilSlidePos -= 2;
                            else anvilSlidePos = 0;
                        }
                    }
                    if (gamepad2.dpad_down) {
                        changePose("lime prep");
                        wasLast = false;
                        mode = 3;
                    }
                    break;

                case "lime check":
                    if (!wasLast) {
                        wasLast = true;
                    }

                    anvilSlidePos = dumbBot.slide3.getCurrentPosition();
                    snifferPos = 0.855;
                    lowClawPos = 1;
                    lowRotPos = 0.84;
                    lowBonkPos = 0.55;
                    lowTurnPos = 0.81;

                    if (time.getElapsedTime() > 50) {
                        changePose("lime found");
                        wasLast = false;
                    }

                    result = limelight.getLatestResult();
                    if (result != null) {
                        array = result.getPythonOutput();
                        if (array[0] == 1) {
                            LimelightYOffsets.add(array[4]);
                            LimelightXOffsets.add(array[3]);
                            LimelightThetas.add(array[1]);
                        } else {
                            changePose("lime search");
                            wasLast = false;
                        }
                    }
                    break;

                case "lime found":
                    if (!wasLast) {
                        iterCount = 0;
                        wasLast = true;

                        desiredY = dumbBot.averageLastContents(LimelightYOffsets, 3) + limeLength;
                        desiredX = dumbBot.averageLastContents(LimelightXOffsets, 3) - 0.5;
                        lowTurnAngle = Math.toDegrees(Math.asin(desiredX / bonkLength));
                        lowRotAngle = lowTurnAngle + dumbBot.averageLastContents(LimelightThetas, 3);
                        anvilSlidePos = (int) ((desiredY - Math.sqrt(bonkLength * bonkLength - Math.pow(desiredX, 2))) * inchPerAnvil + anvilSlidePos);
                    }


                    snifferPos = 0.55;
                    lowClawPos = 1;
                    lowRotPos = (0.18 + lowRotAngle / 180 * 0.67);
                    lowBonkPos = 0.55;
                    lowTurnPos = (0.54 - lowTurnAngle / 180 * 0.54);

                    if (time.getElapsedTime() > 350) {
                        changePose("pre bite");
                        wasLast = false;
                    }
                    break;


                case "pre bite":
                    if (!wasLast) {
                        wasLast = true;
                    }

                    snifferPos = 0.55;
                    lowClawPos = 1;

                    lowRotPos = (0.18 + lowRotAngle / 180 * 0.67);
                    lowBonkPos = 0.73;
                    lowTurnPos = (0.54 - lowTurnAngle / 180 * 0.54);

                    if (time.getElapsedTime() > 150) {
                        changePose("bite");
                        wasLast = false;
                    }
                    break;

                case "bite":
                    if (!wasLast) {
                        wasLast = true;
                    }
                    snifferPos = 0.55;
                    lowClawPos = 1;

                    lowRotPos = (0.18 + lowRotAngle / 180 * 0.67);
                    lowBonkPos = 0.73;
                    lowTurnPos = (0.54 - lowTurnAngle / 180 * 0.54);


                    if (gamepad2.dpad_down) {
                        changePose("lime prep");
                        wasLast = false;
                        mode = 3;
                    }

                    if (time.getElapsedTime() > 200) {
                        changePose("rev check");
                        wasLast = false;
                    }
                    break;

                case "rev check":
                    if (!wasLast) {
                        wasLast = true;
                    }
                    snifferPos = 0.855;
                    lowClawPos = 0;
                    lowRotPos = 0.84;
                    lowBonkPos = 0.65;
                    lowTurnPos = 0.54;
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
                        if (time.getElapsedTime() > 600) {
                            changePose("lime search");
                            wasLast = false;
                        }
                    }
                    if (gamepad2.dpad_down) {
                        changePose("lime prep");
                        wasLast = false;
                        mode = 3;
                    }
                    break;
//_____________________________________________limeline_____________________________________


                case "transfer 1":
                    anvilSlidePos = 0;
                    hammerSlidePos = 640;
                    lowTurnPos = 0.35;
                    lowBonkPos = 0.19;
                    lowRotPos = 0.36;
                    lowClawPos = 0;
                    highTurnPos = 0.17; //0.18
                    highBonkPos = 1.0;
                    highClawPos = 1;
                    snifferPos = 0.305;
                    if (time.getElapsedTime() > 500) {
                        changePose("transfer 2");
                    }
                    break;
                case "transfer 2":
                    anvilSlidePos = 0;
                    hammerSlidePos = 640;
                    lowTurnPos = 0.35;
                    lowBonkPos = 0.19;
                    lowRotPos = 0.36;
                    lowClawPos = 0;
                    highTurnPos = 0.17; //0.18
                    highBonkPos = 1.0;
                    highClawPos = 0;
                    if (time.getElapsedTime() > 500) {
                        changePose("transfer 3");
                    }
                    break;
                case "transfer 3":
                    anvilSlidePos = 0;
                    hammerSlidePos = 640;
                    lowTurnPos = 0.35;
                    lowBonkPos = 0.19;
                    lowRotPos = 0.36;
                    lowClawPos = 0;
                    highTurnPos = 0.17; //0.18
                    highBonkPos = 1.0;
                    highClawPos = 0;
                    if (time.getElapsedTime() > 500) {
                        changePose("transfer 4");
                    }
                    break;
                case "transfer 4":
                    anvilSlidePos = 0;
                    hammerSlidePos = 640;
                    lowTurnPos = 0.35;
                    lowBonkPos = 0.19;
                    lowRotPos = 0.36;
                    lowClawPos = 0;
                    highTurnPos = 0.17; //0.18
                    highBonkPos = 1.0;
                    highClawPos = 0;
                    if (time.getElapsedTime() > 500) {
                        changePose("pre spit");
                    }
                    break;
                case "pre spit":
                    anvilSlidePos = -90;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.66;
                    lowBonkPos = 0.2;
                    lowRotPos = 0.3;
                    lowClawPos = 1;
                    highTurnPos = 0.18;
                    highBonkPos = 0.62; //up
                    highClawPos = 0;
                    if (time.getElapsedTime() > 650) {
                        changePose("spit");
                    }
                    break;

                case "spit":
                    anvilSlidePos = 0;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.51;
                    lowBonkPos = 0.2;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.61;
                    highBonkPos = 0.69; //spit
                    highClawPos = 0;
                    if (gamepad2.a && !a2Last) {
                        changePose("post spit");
                    }
                    break;
                case "post spit":
                    anvilSlidePos = 0;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.51;
                    lowBonkPos = 0.2;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.61;
                    highBonkPos = 0.69; //spit
                    highClawPos = 1;
                    if (time.getElapsedTime() > 500 && gamepad2.a && !a2Last) {
                        changePose("post spit 2");
                    }
                    else if (time.getElapsedTime() > 500 && gamepad2.right_bumper) {
                        changePose("intake up");
                    }
                    break;

                case "post spit 2":
                    anvilSlidePos = 0;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.51;
                    lowBonkPos = 0.2;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.18;
                    highBonkPos = 0.62; //up
                    highClawPos = 1;
                    if (time.getElapsedTime() > 500) {
                        changePose("init");
                    }
                    break;

                case "spec grab open":
                    anvilSlidePos = 0;
                    hammerSlidePos = 690;
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.20; //0.20
                    highBonkPos = 0.15; //specgrab
                    highClawPos = 1;
                    snifferPos = 0.305;
                    if (time.getElapsedTime() > 500) {
                        changePose("spec grab open 2");
                    }
                    else if (gamepad2.b && !b2Last) {
                        changePose("spec grab closed");
                    }
                    break;

                case "spec grab open 2":
                    if (gamepad2.right_trigger > 0){
                    hammerSlidePos+= (int)(gamepad2.right_trigger*25);
                    hammerSlidePos = Math.min(hammerSlidePos,2050);
                }
                else if (gamepad2.left_trigger > 0){
                    hammerSlidePos-= (int)(gamepad2.left_trigger*25);
                    hammerSlidePos = Math.max(hammerSlidePos,0);
                }
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.20;
                    highBonkPos = 0.15; //specgrab
                    highClawPos = 1;
                    snifferPos = 0.855;
                    if ((gamepad2.b && !b2Last) || highColor.getDistance(DistanceUnit.CM) < 2.0) {
                        changePose("spec grab closed");
                    }
                    break;

                case "spec grab closed":
                    Color.RGBToHSV(
                            (int) (lowColor.red() * 255.0 / 1023),
                            (int) (lowColor.green() * 255.0 / 1023),
                            (int) (lowColor.blue() * 255.0 / 1023),
                            hsv
                    );
                    hue = hsv[0];
                    anvilSlidePos = 0;
                    hammerSlidePos = hammerSlidePos;
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.20;
                    highBonkPos = 0.15;
                    highClawPos = 0;
                    if (time.getElapsedTime() > 500) {
                        changePose("spec mid 1");
                    }
                    else if (!((hue < 30 || hue > 330) || (hue < 230 && hue > 210)) && !gamepad2.b) {
                        changePose("spec grab open");
                    }

                    break;
                case "spec mid 1":
                    anvilSlidePos = 0;
                    hammerSlidePos = 870;
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.7;
                    highBonkPos = 0.15;
                    highClawPos = 0;
                    if (time.getElapsedTime() > 350) {
                        changePose("spec place");
                    }
                    else if (gamepad2.a){
                        changePose("spec grab open");
                    }
                    break;


                case "spec place":
                    anvilSlidePos = 0;
                    hammerSlidePos = 870;
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.76; //1
                    highBonkPos = 0.36;//specplace
                    highClawPos = 0;
                    if (time.getElapsedTime() > 200) {
                        changePose("spec place 2");
                    }
                    break;

                case "spec place 2":
                    anvilSlidePos = 0;
                    if (gamepad2.right_trigger > 0){
                    hammerSlidePos+= (int)(gamepad2.right_trigger*25);
                    hammerSlidePos = Math.min(hammerSlidePos,2050);
                }
                else if (gamepad2.left_trigger > 0){
                    hammerSlidePos-= (int)(gamepad2.left_trigger*25);
                    hammerSlidePos = Math.max(hammerSlidePos,0);
                }
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.76;
                    highBonkPos = 0.36;//specplace
                    highClawPos = 0;
                    if (gamepad2.b && !b2Last) {
                        changePose("spec post place");
                    }
                    break;

                case "spec post place":
                    anvilSlidePos = 0;
                    hammerSlidePos = 790;
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.76;
                    highBonkPos = 0.36;
                    highClawPos = 1;
                    if (gamepad2.b && !b2Last) {
                        changePose("spec mid 2");
                    }
                    else if (gamepad2.right_bumper) {
                        changePose("pre spec grab open");
                    }
                    break;
                case "spec mid 2":
                    anvilSlidePos = 0;
                    hammerSlidePos = 120;
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.18;
                    highBonkPos = 0.62; //up
                    highClawPos = 0;
                    if (time.getElapsedTime() > 500) {
                        changePose("init");
                    }
                    break;

                case "pre spec grab open":
                    anvilSlidePos = 0;
                    hammerSlidePos = 790;
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.7;
                    highBonkPos = 0.36;
                    highClawPos = 1;
                    if (time.getElapsedTime() > 200) {
                        changePose("pre spec grab open 2");
                    }
                    break;

                case "pre spec grab open 2":
                    anvilSlidePos = 0;
                    hammerSlidePos = 790;
                    lowTurnPos = 0.81;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.51;
                    lowClawPos = 1;
                    highTurnPos = 0.7;
                    highBonkPos = 0.15;
                    highClawPos = 1;
                    if (time.getElapsedTime() > 200) {
                        changePose("spec grab open");
                    }
                    break;
            }
            a2Last = gamepad2.a;
            b2Last = gamepad2.b;
            lBumpLast = gamepad2.left_bumper;

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

            telemetry.addData("state", scrollPoses);
            telemetry.addLine(lime? "limed":"limeless"); //t:f

            telemetry.addData("halfspeed: ", dumbBot.halfSpeedToggle);
            telemetry.addData("quarterspeed: ", dumbBot.qtrSpeedToggle);
            telemetry.addData("reverse: ", dumbBot.drivingReverse);

            telemetry.addData("hammerSlidePos: ", hammerSlidePos);
            telemetry.addData("anvilSlidePos: ", anvilSlidePos);

            telemetry.update();
        }
    }
    public void changePose(String pose){
        scrollPoses = pose;
        time.resetTimer();
    }
}
