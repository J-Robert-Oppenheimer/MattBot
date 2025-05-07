package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "bucketauto")
public class bucketauton extends OpMode {
    dumbMap dumbBot = new dumbMap(this);
    public int anvilSlidePos, hammerSlidePos;

    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(7.75, 105, Math.toRadians(0));
    private final Pose scorePose = new Pose(14, 134, Math.toRadians(0));
    private final Pose pickOne = new Pose(26, 122, Math.toRadians(0));
    private final Pose pickTwo = new Pose(24, 132, Math.toRadians(0));
    private final Pose pickThree = new Pose(40, 134.6, Math.toRadians(0));

    private PathChain scoreOne, pickUpOne, scoreTwo, pickUpTwo, scoreThree, pickUpThree, scoreFour;

    public void buildPaths() {
        scoreOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        pickUpOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickOne)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickOne.getHeading())
                .build();

        scoreTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickOne), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickOne.getHeading(), scorePose.getHeading())
                .build();

        pickUpTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickTwo)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickTwo.getHeading())
                .build();

        scoreThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickTwo), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickTwo.getHeading(), scorePose.getHeading())
                .build();

        pickUpThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickThree)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickThree.getHeading())
                .build();

        scoreFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickThree), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickThree.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scoreOne, true);
                setPathState(1);
            case 1:
                anvilSlidePos = 0;
                hammerSlidePos = 2050;
                lowTurnPos = 0.46;
                lowBonkPos = 0.22;
                lowRotPos = 0.5;
                lowClawPos = 0;
                highTurnPos = 0.24;
                highBonkPos = 0.1;
                highClawPos = 1;
                if (!follower.isBusy()) {
                    setPathState(2);
                }
            break;
            case 2:
                highClawPos = 0;
                if (pathTimer.getElapsedTime()>400) {
                    highBonkPos = 0.08;
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime()>1000) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 870;
                    lowTurnPos = 0.46;
                    lowBonkPos = 0.65;
                    lowRotPos = 0.5;
                    lowClawPos = 0;
                    highTurnPos = 0.80;
                    highBonkPos = 0.16;
                    highClawPos = 0;
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTime()>500) {
                    follower.followPath(pickUpOne, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                        lowClawPos = 0;
                        lowBonkPos = 0.74;
                        if (pathTimer.getElapsedTime() > 1500) {
                            lowClawPos = 1;
                            setPathState(6);
                        }

                    }
                break;
                case 6:
                    if(pathTimer.getElapsedTime()>1000) {
                        anvilSlidePos = -120;
                        hammerSlidePos = 820;
                        lowTurnPos = 0.66;
                        lowBonkPos = 0.22;
                        lowRotPos = 0.32;
                        lowClawPos = 1;
                        highTurnPos = 0.81;
                        highBonkPos = 0.16;
                        highClawPos = 1;
                        setPathState(7);
                    }
                    break;
                case 7:
                    if(pathTimer.getElapsedTime()>750) {
                        anvilSlidePos = -120;
                        hammerSlidePos = 820;
                        lowTurnPos = 0.66;
                        lowBonkPos = 0.22;
                        lowRotPos = 0.32;
                        lowClawPos = 1;
                        highTurnPos = 0.81;
                        highBonkPos = 0.16;
                        highClawPos = 0;
                        setPathState(8);
                    }
                    break;
                case 8:
                    if(pathTimer.getElapsedTime()>500) {
                        anvilSlidePos = -120;
                        hammerSlidePos = 820;
                        lowTurnPos = 0.66;
                        lowBonkPos = 0.22;
                        lowRotPos = 0.32;
                        lowClawPos = 0;
                        highTurnPos = 0.81;
                        highBonkPos = 0.16;
                        highClawPos = 0;
                        setPathState(9);
                        }
                        break;
            case 9:
                follower.followPath(scoreTwo, true);
                setPathState(10);
                break;
            case 10:
                anvilSlidePos = 0;
                hammerSlidePos = 2050;
                lowTurnPos = 0.46;
                lowBonkPos = 0.22;
                lowRotPos = 0.5;
                lowClawPos = 0;
                highTurnPos = 0.24;
                highBonkPos = 0.1;
                highClawPos = 1;
                if (!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            case 11:
                highClawPos = 1;
                if (pathTimer.getElapsedTime()>400) {
                    highBonkPos = 0.1;
                    setPathState(12);
                }
                break;
            case 12:

                if (pathTimer.getElapsedTime()>1000) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 820;
                    lowTurnPos = 0.46;
                    lowBonkPos = 0.65;
                    lowRotPos = 0.5;
                    lowClawPos = 0;
                    highTurnPos = 0.80;
                    highBonkPos = 0.1;
                    highClawPos = 1;
                    setPathState(13);
                }
                break;


            case 13:
                if(pathTimer.getElapsedTime()>500) {
                    follower.followPath(pickUpTwo, true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    lowClawPos = 0;
                    lowBonkPos = 0.74;
                    if (pathTimer.getElapsedTime() > 1500) {
                        lowClawPos = 1;
                        setPathState(15);
                    }

                }
                break;
            case 15:
                if(pathTimer.getElapsedTime()>1000) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 820;
                    lowTurnPos = 0.66;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 1;
                    highTurnPos = 0.81;
                    highBonkPos = 0.16;
                    highClawPos = 0;
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.getElapsedTime()>750) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 820;
                    lowTurnPos = 0.66;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 1;
                    highTurnPos = 0.81;
                    highBonkPos = 0.16;
                    highClawPos = 1;
                    setPathState(17);
                }
                break;
            case 17:
                if(pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 820;
                    lowTurnPos = 0.66;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 0;
                    highTurnPos = 0.81;
                    highBonkPos = 0.16;
                    highClawPos = 1;
                    setPathState(18);
                }
                break;
            case 18:
                follower.followPath(scoreThree, true);
                setPathState(19);
                break;
            case 19:
                anvilSlidePos = 0;
                hammerSlidePos = 2050;
                lowTurnPos = 0.46;
                lowBonkPos = 0.22;
                lowRotPos = 0.5;
                lowClawPos = 0;
                highTurnPos = 0.24;
                highBonkPos = 0.1;
                highClawPos = 0;
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;
            case 20:
                highClawPos = 1;
                if (pathTimer.getElapsedTime()>400) {
                    highBonkPos = 0.08;
                    setPathState(21);
                }
                break;
            case 21:

                if (pathTimer.getElapsedTime()>1000) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 820;
                    lowTurnPos = 0.46;
                    lowBonkPos = 0.65;
                    lowRotPos = 0.5;
                    lowClawPos = 0;
                    highTurnPos = 0.80;
                    highBonkPos = 0.1;
                    highClawPos = 1;
                    setPathState(22);
                }
                break;
            case 22:
                if(pathTimer.getElapsedTime()>500) {
                    follower.followPath(pickUpThree, true);
                    setPathState(23);
                }

            case 23:
                if(!follower.isBusy()) {
                    lowClawPos = 0;
                    lowBonkPos = 0.74;
                    if (pathTimer.getElapsedTime() > 1500) {
                        lowClawPos = 1;
                        setPathState(24);
                    }

                }
                break;
            case 24:
                if(pathTimer.getElapsedTime()>1000) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 820;
                    lowTurnPos = 0.66;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 1;
                    highTurnPos = 0.81;
                    highBonkPos = 0.16;
                    highClawPos = 1;
                    setPathState(25);
                }
                break;
            case 25:
                if(pathTimer.getElapsedTime()>750) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 820;
                    lowTurnPos = 0.66;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 1;
                    highTurnPos = 0.81;
                    highBonkPos = 0.16;
                    highClawPos = 0;
                    setPathState(26);
                }
                break;
            case 26:
                if(pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 820;
                    lowTurnPos = 0.66;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 0;
                    highTurnPos = 0.81;
                    highBonkPos = 0.16;
                    highClawPos = 0;
                    setPathState(27);
                }
                break;
            case 27:
                follower.followPath(scoreFour, true);
                setPathState(28);
                break;
            case 28:
                anvilSlidePos = 0;
                hammerSlidePos = 2050;
                lowTurnPos = 0.46;
                lowBonkPos = 0.22;
                lowRotPos = 0.5;
                lowClawPos = 0;
                highTurnPos = 0.24;
                highBonkPos = 0.08;
                highClawPos = 1;
                if (!follower.isBusy()) {
                    setPathState(29);
                }
                break;
            case 29:
                highClawPos = 1;
                if (pathTimer.getElapsedTime()>400) {
                    highBonkPos = 0.08;
                    setPathState(30);
                }
                break;
            case 30:
                if (pathTimer.getElapsedTime()>1000) {
                    anvilSlidePos = 0;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.46;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.5;
                    lowClawPos = 0;
                    highTurnPos = 0.80;
                    highBonkPos = 0.09;
                    highClawPos = 0;
                    setPathState(31);
                }
                break;




        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        dumbBot.slide3.setTargetPosition(anvilSlidePos);
        dumbBot.slide1.setTargetPosition(hammerSlidePos);
        dumbBot.slide2.setTargetPosition(hammerSlidePos);
        dumbBot.lowTurn.setPosition(lowTurnPos);
        dumbBot.lowBonk.setPosition(lowBonkPos);
        dumbBot.lowRot.setPosition(lowRotPos);
        dumbBot.lowClaw.setPosition(lowClawPos);
        dumbBot.highTurnT.setPosition(highTurnPos);
        dumbBot.highBonkR.setPosition(highBonkPos);
        dumbBot.highBonkL.setPosition(highBonkPos);
        dumbBot.highClaw.setPosition(highClawPos);


        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("busy? ", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void init() {
        dumbBot.init2();
        dumbBot.slidereset2();
        dumbBot.highClaw.setPosition(1);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}