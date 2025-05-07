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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BeleOff", group = "oldteles")
@Config
public class BeleOff extends LinearOpMode {
    dumbMap dumbBot = new dumbMap(this);
    public int anvilSlidePos, hammerSlidePos;
    public String scrollPoses = "init";

    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos;
    public boolean lowClawOpen, a2Last = false, b2Last = false, x2Last = false, lowClawRot = true, lBumpLast = false, specs = true, psLast = false;
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
                    lowTurnPos = 0.46;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.5;
                    lowClawPos = 0;

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
                    lowTurnPos = 0.46;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.5;
                    lowClawPos = 0;

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
                        lowTurnPos = Math.max(Math.min(lowTurnPos, 0.83), 0.13);
                    }
                    if (gamepad2.left_bumper && !lBumpLast) lowClawRot = !lowClawRot;
                    lowBonkPos = 0.69;
                    lowRotPos = lowClawRot? 0.5:0.83;
                    lowClawPos = 0;

                    if (gamepad2.x) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "intake down open";
                    }
                    break;
                case "intake down open":
                    anvilSlidePos = anvilSlidePos;
                    hammerSlidePos = 0;
                    lowTurnPos = lowTurnPos;
                    lowBonkPos = 0.75;
                    lowRotPos = lowRotPos;
                    lowClawPos = 0;

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
                    if (time.getElapsedTime() - startTime > 300) {
                        anvilSlidePos = anvilSlidePos;
                        hammerSlidePos = 0;
                        lowTurnPos = lowTurnPos;
                        lowBonkPos = 0.75;
                        lowRotPos = lowRotPos;
                        lowClawPos = 1;

                        if (gamepad2.a && !a2Last && !specs) {
                            startTime = time.getElapsedTime();
                            scrollPoses = "low bonk up";
                        } else if (gamepad2.a && !a2Last && specs) {
                            startTime = time.getElapsedTime();
                            scrollPoses = "transfer to side";
                        }
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
                        lowTurnPos = 0.83;
                        lowBonkPos = 0.69;
                        lowRotPos = 0.83;
                        lowClawPos = 1;

                        if (time.getElapsedTime() - startTime > 500) {
                            startTime = time.getElapsedTime();
                            scrollPoses = "throw off side";
                        }
                    }
                    break;

                case "throw off side":
                    anvilSlidePos = -80;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.83;
                    lowBonkPos = 0.65;
                    lowRotPos = 0.83;
                    lowClawPos = 1;

                    if (gamepad2.a && !a2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "post throw";
                    }
                    break;

                case "post throw":
                    anvilSlidePos = -80;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.83;
                    lowBonkPos = 0.69;
                    lowRotPos = 0.83;
                    lowClawPos = 0;

                    if (time.getElapsedTime() - startTime > 500) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "high bonk up";
                    }
                    break;

                case "spec grab open":
                    anvilSlidePos = 0;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.5;
                    lowBonkPos = 0.69;
                    lowRotPos = 0.5;
                    lowClawPos = 0;
                    if (gamepad2.b && !b2Last) {
                        startTime = time.getElapsedTime();
                        scrollPoses = "spec grab close";
                    }
                    break;

                case "spec grab close":
                    if (time.getElapsedTime() - startTime > 300) {

                        anvilSlidePos = 0;
                        hammerSlidePos = 0;
                        lowTurnPos = 0.5;
                        lowBonkPos = 0.69;
                        lowRotPos = 0.5;
                        lowClawPos = 1;
                        if (time.getElapsedTime() - startTime > 800) {
                            startTime = time.getElapsedTime();
                            scrollPoses = "spec grab up";
                        }
                    }
                    break;
                case "spec grab up":
                    if (time.getElapsedTime() - startTime > 300) {

                        if (Math.abs(gamepad2.left_stick_y) > 0) {
                            anvilSlidePos += (int) (gamepad2.left_stick_y * 5);
                            anvilSlidePos =  Math.max(Math.min(anvilSlidePos, 0), -400);
                        }
                        hammerSlidePos = 0;
                        lowTurnPos = 0.5;
                        lowBonkPos = 0.22;
                        lowRotPos = 0.5;
                        lowClawPos = 1;
                        if (gamepad2.a && !a2Last) {
                            startTime = time.getElapsedTime();
                            scrollPoses = "init";
                        }
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


            telemetry.addData("state", scrollPoses);
            telemetry.addData("specs? ", specs);

            telemetry.addData("halfspeed: ", dumbBot.halfSpeedToggle);
            telemetry.addData("quarterspeed: ", dumbBot.qtrSpeedToggle);
            telemetry.addData("reverse: ", dumbBot.drivingReverse);

            telemetry.update();
        }
    }
}
