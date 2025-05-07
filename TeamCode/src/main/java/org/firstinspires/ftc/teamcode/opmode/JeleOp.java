package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "JeleOp", group = "oldteles")
@Config

public class JeleOp extends LinearOpMode {
    dumbMap dumbBot = new dumbMap(this);
    public int anvilSlidePos, hammerSlidePos;
    public String scrollPoses = "init";

    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos;
    public boolean lowClawOpen, a2Last = false, b2Last = false, x2Last = false, lowClawRot = true, lBumpLast = false, specs = false, psLast = false;
    private Follower follower;

    private Timer time;
    private long startTime;
    private final Pose startPose = new Pose(0,0,0);

    //boolean searching, found = false, foundLast = false, vertical = false, verticalLast = false;


    @Override
    public void runOpMode() throws InterruptedException {
        dumbBot.init2();
        dumbBot.slidereset2();

        time = new Timer();
        startTime = 0;

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

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
                startTime = time.getElapsedTime();
                scrollPoses = "init";
            }

            if (gamepad2.ps && !psLast){
                specs = !specs;
            }
            psLast = gamepad2.ps;


            switch (scrollPoses) {
                case "init":
                    anvilSlidePos = 0;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.1;
                    highBonkPos = 0.255;
                    highClawPos = 0;
                    if (gamepad2.a && !a2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "high bonk up";
                    } else if (gamepad2.b && !b2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec grab open";
                    }
                    break;
                case "high bonk up":
                    anvilSlidePos = 0;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.185;//up
                    highClawPos = 0;
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "intake up";
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
                        lowTurnPos = Math.max(Math.min(lowTurnPos, 0.91), 0.14);
                    }
                    if (gamepad2.left_bumper && !lBumpLast) lowClawRot = !lowClawRot;
                    lowBonkPos = 0.67;
                    lowRotPos = lowClawRot? 0.60:0.91;
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.185;//up
                    highClawPos = 0;
                    if (gamepad2.x) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "intake down open";
                    }
                    break;
                case "intake down open":
                    if (Math.abs(gamepad2.left_stick_y) > 0) {
                        anvilSlidePos += (int) (gamepad2.left_stick_y * 10);
                        anvilSlidePos =  Math.max(Math.min(anvilSlidePos, 0), -400);
                    }
                    hammerSlidePos = 0;
                    if (Math.abs(gamepad2.left_stick_x) > 0.5){
                        lowTurnPos += gamepad2.left_stick_x * 0.008;
                        lowTurnPos = Math.max(Math.min(lowTurnPos, 0.91), 0.14);
                    }
                    lowBonkPos = 0.75;
                    lowRotPos = lowRotPos;
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.185;//up
                    highClawPos = 0;
                    if (gamepad2.a && !a2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "intake down closed";
                    }
                    else if (!gamepad2.x) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "intake up";
                    }
                    break;
                case "intake down closed":
                    anvilSlidePos = anvilSlidePos;
                    hammerSlidePos = 0;
                    lowTurnPos = lowTurnPos;
                    lowBonkPos = 0.75;
                    lowRotPos = lowRotPos;
                    lowClawPos = 1;
                    highTurnPos = 0.18;
                    highBonkPos = 0.185;//up
                    highClawPos = 0;

                    if (time.getElapsedTime()-startTime > 400) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "intake down raised";
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
                        lowTurnPos += gamepad2.left_stick_x * 0.008;
                        lowTurnPos = Math.max(Math.min(lowTurnPos, 0.91), 0.16);
                    }
                    lowClawPos = 1;
                    highTurnPos = 0.18;
                    highBonkPos = 0.185;//up
                    highClawPos = 0;
                    if (gamepad2.b){
                        startTime = time.getElapsedTime();
                        scrollPoses = "intake up";
                    }
                    if (gamepad2.a && !a2Last && !specs) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "low bonk up";
                    } else if (gamepad2.a && !a2Last && specs) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "transfer to side";
                    }
                    break;
                case "intake up reset":
                    anvilSlidePos = 0;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.67;
                    lowRotPos = 0.60;
                    lowClawRot = true;
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.185;//up
                    highClawPos = 0;
                    if (time.getElapsedTime() - startTime > 200) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "intake up";
                    }
                    break;


                case "low bonk up":
                    if (time.getElapsedTime() - startTime > 300) {
                        anvilSlidePos = anvilSlidePos;
                        hammerSlidePos = 0;
                        lowTurnPos = lowTurnPos;
                        lowBonkPos = 0.22;
                        lowRotPos = lowRotPos;
                        lowClawPos = 1;
                        highTurnPos = 0.18;
                        highBonkPos = 0.185;//up
                        highClawPos = 0;
                        if (time.getElapsedTime() - startTime > 500) {
                            startTime = time.getElapsedTime();
                            scrollPoses = "transfer 1";
                        }
                    }

                    break;

                case "transfer to side":
                    if (time.getElapsedTime() - startTime > 300) {
                        anvilSlidePos = -80;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.91;
                        lowBonkPos = 0.69;
                        lowRotPos = 0.9;
                        lowClawPos = 1;
                        highTurnPos = 0.18;
                        highBonkPos = 0.185;//up
                        highClawPos = 0;
                        if (time.getElapsedTime() - startTime > 500) {
                            startTime = time.getElapsedTime();
                            scrollPoses = "throw off side";
                        }
                    }
                    break;

                case "throw off side":
                    anvilSlidePos = -80;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.91;
                    lowBonkPos = 0.65;
                    lowRotPos = 0.9;
                    lowClawPos = 1;
                    highTurnPos = 0.18;
                    highBonkPos = 0.185;//up
                    highClawPos = 0;
                    if (gamepad2.a && !a2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "post throw";
                    }
                    break;

                case "post throw":
                    anvilSlidePos = -80;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.91;
                    lowBonkPos = 0.69;
                    lowRotPos = 0.9;
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.185;//up
                    highClawPos = 0;
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "high bonk up";
                    }
                    break;

                case "transfer 1":
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.37;
                    lowClawPos = 1;
                    highTurnPos = 0.20; //0.18
                    highBonkPos = 0.26;
                    highClawPos = 0;
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "transfer 2";
                    }
                    break;
                case "transfer 2":
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.37;
                    lowClawPos = 1;
                    highTurnPos = 0.20; //0.18
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "transfer 3";
                    }
                    break;
                case "transfer 3":
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.37;
                    lowClawPos = 0;
                    highTurnPos = 0.20; //0.18
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "transfer 4";
                    }
                    break;
                case "transfer 4":
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 0;
                    highTurnPos = 0.20; //0.18
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "pre spit";
                    }
                    break;
                case "pre spit":
                    anvilSlidePos = -90;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.66;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.2;
                    highClawPos = 1;
                    if (time.getElapsedTime() - startTime > 650) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spit";
                    }
                    break;

                case "spit":
                    anvilSlidePos = 0;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.65;
                    highBonkPos = 0.2; //spit
                    highClawPos = 1;
                    if (gamepad2.a && !a2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "post spit";
                    }
                    break;
                case "post spit":
                    anvilSlidePos = 0;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.65;
                    highBonkPos = 0.2; //spit
                    highClawPos = 0;
                    if (time.getElapsedTime() - startTime > 500 && gamepad2.a && !a2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "post spit 2";
                    }
                    else if (time.getElapsedTime() - startTime > 500 && gamepad2.right_bumper) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "intake up";
                    }
                    break;

                case "post spit 2":
                    anvilSlidePos = 0;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.185; //up
                    highClawPos = 0;
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "init";
                    }
                    break;

                case "spec grab open":
                    hammerSlidePos = 725;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.20; //0.20
                    highBonkPos = 0.105; //specgrab
                    highClawPos = 0;
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec grab open 2";
                    }
                    else if (gamepad2.b && !b2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec grab closed";
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
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.20;
                    highBonkPos = 0.105; //specgrab
                    highClawPos = 0;
                    if ((gamepad2.b && !b2Last)) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec grab closed";
                    }
                    break;

                case "spec grab closed":
                    anvilSlidePos = 0;
                    hammerSlidePos = hammerSlidePos;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.20;
                    highBonkPos = 0.105;
                    highClawPos = 1;
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec mid 1";
                    }
                    break;
                case "spec mid 1":
                    anvilSlidePos = 0;
                    hammerSlidePos = 980;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.20;
                    highBonkPos = 0.185; //up
                    highClawPos = 1;
                    if (time.getElapsedTime() - startTime > 350) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec pre place";
                    }
                    break;

                case "spec pre place":
                    anvilSlidePos = 0;
                    hammerSlidePos = 980;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.76;
                    highBonkPos = 0.185;//up
                    highClawPos = 1;
                    if (time.getElapsedTime() - startTime > 750) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec place";
                    }
                    break;

                case "spec place":
                    anvilSlidePos = 0;
                    hammerSlidePos = 980;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.76; //1
                    highBonkPos = 0.135;//specplace
                    highClawPos = 1;
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec place 2";
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
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.76;
                    highBonkPos = 0.135;//specplace
                    highClawPos = 1;
                    if (gamepad2.b && !b2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec post place";
                    }
                    break;

                case "spec post place":
                    anvilSlidePos = 0;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.76;
                    highBonkPos = 0.135;
                    highClawPos = 0;
                    if (gamepad2.b && !b2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec mid 2";
                    }

                    else if (gamepad2.right_trigger > 0.6) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec grab open";
                    }
                    break;
                case "spec mid 2":
                    anvilSlidePos = 0;
                    hammerSlidePos = 120;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.60;
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.185; //up
                    highClawPos = 1;
                    if (time.getElapsedTime() - startTime > 1000) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "init";
                    }
                    break;
                default:
                    telemetry.addLine("your dumb");
                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "init";
                    }
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
            dumbBot.highBonkR.setPosition(highBonkPos);
            dumbBot.highBonkL.setPosition(highBonkPos);
            dumbBot.highClaw.setPosition(highClawPos);

            telemetry.addData("state", scrollPoses);
            telemetry.addData("specs? ", specs);

            telemetry.addData("halfspeed: ", dumbBot.halfSpeedToggle);
            telemetry.addData("quarterspeed: ", dumbBot.qtrSpeedToggle);
            telemetry.addData("reverse: ", dumbBot.drivingReverse);

            telemetry.addData("hammerSlidePos: ", hammerSlidePos);
            telemetry.addData("anvilSlidePos: ", anvilSlidePos);

            telemetry.update();
        }
    }
}
