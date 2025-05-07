package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */


@Autonomous(name = "specNoArm")
public class specNoArm extends OpMode {
    dumbMap dumbBot = new dumbMap(this);
    public Limelight3A limelight;
    RevColorSensorV3 highColor, lowColor;
    float[] lowhsv;
    public int anvilSlidePos, hammerSlidePos;
    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
//    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos;

    public Servo lowClaw, lowRot, lowBonk, lowTurn, highClaw, highTurn, highBonkL, highBonkR;
    public DcMotor slide1, slide2, slide3;


    // Robot positions
    private final Pose startPose = new Pose(8, 65, Math.toRadians(0));
    private final Pose preBar = new Pose(20, 67, Math.toRadians(0));
    private final Pose bar1 = new Pose(45.5, 78, Math.toRadians(0));
    private final Pose bar2 = new Pose(45.5, 77, Math.toRadians(0));
    private final Pose bar3 = new Pose(45.5, 76, Math.toRadians(0));
    private final Pose bar4 = new Pose(45.5, 75, Math.toRadians(0));
    private final Pose bar5 = new Pose(45.5, 74, Math.toRadians(0));
    private final Pose bar6 = new Pose(45.5, 73, Math.toRadians(0));
    private final Pose grab1 = new Pose(46-13, 23+13, Math.toRadians(-45));
    private final Pose grab2 = new Pose(46-13, 13+13, Math.toRadians(-45));
    private final Pose grab3 = new Pose(46-13, 3+13, Math.toRadians(-45));
    private final Pose drop1 = new Pose(6+18, 23+13, Math.toRadians(-135));
    private final Pose drop2 = new Pose(6+18, 13+13, Math.toRadians(-135));
    private final Pose drop3 = new Pose(6+18, 3+13, Math.toRadians(-135));
    private final Pose firstPushline = new Pose(24, 32, Math.toRadians(0));
    private final Pose firstPushprep = new Pose(67.5, 32, Math.toRadians(0));
    private final Pose firstPushprep2 = new Pose(67.5, 22, Math.toRadians(0));
    private final Pose firstPush = new Pose(22, 22, Math.toRadians(0));
    private final Pose secondPushprep = new Pose(67.5, 22, Math.toRadians(0));
    private final Pose secondPushprep2 = new Pose(67.5, 16, Math.toRadians(0));
    private final Pose secondPush = new Pose(22, 16, Math.toRadians(0));
    private final Pose secondPushBack = new Pose(67.5, 17, Math.toRadians(0));
    private final Pose thirdPushprep = new Pose(67.5, 12, Math.toRadians(0));
    private final Pose thirdPush = new Pose(22, 12, Math.toRadians(0));
    private final Pose pickUp1 = new Pose(42, 32, Math.toRadians(270));
    private final Pose dropOff1 = new Pose(18, 32, Math.toRadians(270));
    private final Pose pickUp2 = new Pose(42, 23, Math.toRadians(270));
    private final Pose dropOff2 = new Pose(18, 22, Math.toRadians(270));
    private final Pose pickUp3 = new Pose(42, 14, Math.toRadians(270));
    private final Pose dropOff3 = new Pose(18, 12, Math.toRadians(270));

    private final Pose subPickUp = new Pose();
    private final Pose wallGrab = new Pose(15.25, 30, Math.toRadians(0));
    private final Pose wallGrab2 = new Pose(15.25, 30, Math.toRadians(-90));
    private final Pose park = new Pose(15, 15, Math.toRadians(0));

    private PathChain push, spikeMid, sampleDrop, spike1Grab, spike1Drop, spike2Grab, spike2Drop, spike3Grab, spike3Drop, toBar1, toBar2, toBar3, toBar4, toBar5, toBar6, toPreBar1, toPreBar2, toPreBar3, toPreBar4, toPreBar5, toPreBar6, toPickUp1, toPickUp2, toPickUp3, toDropOff1, toDropOff2, toDropOff3, toWallGrab1, toWallGrab2, toWallGrab3, toWallGrab4, toWallGrab5, toPark, total, wallTurn;

    public void buildPaths() {

        toPreBar1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(startPose),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).build();

        toBar1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(startPose),
                        new Point(bar1)
                )).setConstantHeadingInterpolation(0).build();

        sampleDrop = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(bar1),
                        new Point(preBar.getX()-6, preBar.getY()),
                        new Point(wallGrab2)
                )).setLinearHeadingInterpolation(0, Math.toRadians(-115)).build();

        spike1Grab = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab2),
                        new Point(grab1)
                )).setLinearHeadingInterpolation(Math.toRadians(-115), -45).build();

        spike1Drop = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(grab1),
                        new Point(drop1)
                )).setLinearHeadingInterpolation(-45, -135).build();

        spike2Grab = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(drop1),
                        new Point(grab2)
                )).setLinearHeadingInterpolation(-135, -45).build();

        spike2Drop = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(grab2),
                        new Point(drop2)
                )).setLinearHeadingInterpolation(-45, -135).build();

        spike3Grab = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(drop2),
                        new Point(grab3)
                )).setLinearHeadingInterpolation(-135, -45).build();

        spike3Drop = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(grab3),
                        new Point(drop3)
                )).setLinearHeadingInterpolation(-45, -135).build();

        push = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bar1), new Point(firstPushline)))
                .setLinearHeadingInterpolation(0, Math.toRadians(-90))
                .addPath(new BezierLine(new Point(firstPushline), new Point(firstPushprep)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(new BezierLine(new Point(firstPushprep), new Point(firstPushprep2)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(new BezierLine(new Point(firstPushprep2), new Point(firstPush)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(new BezierLine(new Point(firstPush), new Point(secondPushprep)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(new BezierLine(new Point(secondPushprep), new Point(secondPushprep2)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), 0)
                .addPath(new BezierLine(new Point(secondPushprep2), new Point(secondPush)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(new BezierLine(new Point(secondPush), new Point(thirdPushprep)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(new BezierLine(new Point(thirdPushprep), new Point(thirdPush)))
                .setLinearHeadingInterpolation(Math.toRadians(-90), 0)

                .setConstantHeadingInterpolation(0)
//                .addPath(new BezierLine(new Point(secondPush), new Point(secondPushBack)))
//                .setLinearHeadingInterpolation(Math.toRadians(-90), 0)
//                .addPath(new BezierLine(new Point(secondPushBack), new Point(thirdPushprep)))
//                .setConstantHeadingInterpolation(0)
//                .addPath(new BezierLine(new Point(thirdPushprep), new Point(thirdPush)))
//                .setConstantHeadingInterpolation(0)
                .build();


        toPickUp1 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(bar1),
                        new Point(preBar),
                        new Point(pickUp1)
                )).setLinearHeadingInterpolation(0, Math.toRadians(270)).build();

        toDropOff1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(pickUp1),
                        new Point(dropOff1)
                )).setConstantHeadingInterpolation(Math.toRadians(270)).build();

        toPickUp2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(dropOff1),
                        new Point(pickUp2)
                )).setConstantHeadingInterpolation(Math.toRadians(270)).build();

        toDropOff2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(pickUp2),
                        new Point(dropOff2)
                )).setConstantHeadingInterpolation(Math.toRadians(270)).build();

        toPickUp3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(dropOff2),
                        new Point(pickUp3)
                )).setConstantHeadingInterpolation(Math.toRadians(270)).build();

        toDropOff3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(pickUp3),
                        new Point(dropOff3)
                )).setConstantHeadingInterpolation(Math.toRadians(270)).build();

        toWallGrab1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(drop3),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

//        toPreBar2 = follower.pathBuilder().addPath(
//                new BezierLine(
//                        new Point(wallGrab),
//                        new Point(preBar)
//                )).setConstantHeadingInterpolation(0).build();

        toBar2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).addPath(
                new BezierLine(
                        new Point(preBar),
                        new Point(bar2)
                )).setConstantHeadingInterpolation(0).build();

        toWallGrab2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(bar2),
                        new Point(wallGrab2)
                )).setLinearHeadingInterpolation(0, 0).build();

        wallTurn = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(wallGrab2),
                        new Point(45, 30),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

//        toPreBar3 = follower.pathBuilder().addPath(
//                new BezierLine(
//                        new Point(wallGrab),
//                        new Point(preBar)
//                )).setConstantHeadingInterpolation(0).build();

        toBar3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).addPath(
                new BezierLine(
                        new Point(preBar),
                        new Point(bar3)
                )).setConstantHeadingInterpolation(0).build();

        toWallGrab3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(bar3),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

//        toPreBar4 = follower.pathBuilder().addPath(
//                new BezierLine(
//                        new Point(wallGrab),
//                        new Point(preBar)
//                )).setConstantHeadingInterpolation(0).build();

        toBar4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).addPath(
                new BezierLine(
                        new Point(preBar),
                        new Point(bar4)
                )).setConstantHeadingInterpolation(0).build();

        toWallGrab4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(bar4),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

//        toPreBar5 = follower.pathBuilder().addPath(
//                new BezierLine(
//                        new Point(wallGrab),
//                        new Point(preBar)
//                )).setConstantHeadingInterpolation(0).build();

        toBar5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).addPath(
                new BezierLine(
                        new Point(preBar),
                        new Point(bar5)
                )).setConstantHeadingInterpolation(0).build();

        toWallGrab5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(bar5),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

//        toPreBar6 = follower.pathBuilder().addPath(
//                new BezierLine(
//                        new Point(wallGrab),
//                        new Point(preBar)
//                )).setConstantHeadingInterpolation(0).build();

        toBar6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).addPath(
                new BezierLine(
                        new Point(preBar),
                        new Point(bar6)
                )).setConstantHeadingInterpolation(0).build();

        toPark = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(bar5),
                        new Point(preBar),
                        new Point(park)
                )).setConstantHeadingInterpolation(0).build();

        total = follower.pathBuilder().addPath(
                        // Line 1
                        new BezierLine(
                                new Point(8, 64.500, Point.CARTESIAN),
                                new Point(34.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(34.000, 72.000, Point.CARTESIAN),
                                new Point(24.000, 14.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(24.000, 14.500, Point.CARTESIAN),
                                new Point(20.000, 14.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(20.000, 14.500, Point.CARTESIAN),
                                new Point(24.000, 12.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(24.000, 12.500, Point.CARTESIAN),
                                new Point(20.000, 12.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(20.000, 12.500, Point.CARTESIAN),
                                new Point(24.000, 10.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(24.000, 10.500, Point.CARTESIAN),
                                new Point(20.000, 10.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(20.000, 10.500, Point.CARTESIAN),
                                new Point(15.500, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(15.500, 30.000, Point.CARTESIAN),
                                new Point(34.000, 69.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(34.000, 69.500, Point.CARTESIAN),
                                new Point(15.500, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(15.500, 30.000, Point.CARTESIAN),
                                new Point(34.000, 67.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(34.000, 67.000, Point.CARTESIAN),
                                new Point(15.500, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(15.500, 30.000, Point.CARTESIAN),
                                new Point(34.000, 64.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 14
                        new BezierLine(
                                new Point(34.000, 64.500, Point.CARTESIAN),
                                new Point(15.500, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 15
                        new BezierLine(
                                new Point(15.500, 30.000, Point.CARTESIAN),
                                new Point(34.000, 62.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 16
                        new BezierLine(
                                new Point(34.000, 62.000, Point.CARTESIAN),
                                new Point(10.500, 10.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case (1):
                follower.followPath(toBar1);
                setPathState(201);
                break;
//            case (2):
//                if (!follower.isBusy()) {
//                    follower.followPath(toBar1);
//                    setPathState(200);
//                }
//                break;

            case (201):
                if (!follower.isBusy() || pathTimer.getElapsedTime()>3500) {
                    follower.followPath(sampleDrop);
                    setPathState(3);
                }
                break;

            case (3):
                if (!follower.isBusy()) {
                    follower.followPath(spike1Grab);
                    setPathState(4);
                }
                break;
            case (4):
                if (!follower.isBusy()) {
                    follower.followPath(spike1Drop);
                    setPathState(5);
                }
                break;
            case (5):
                if (!follower.isBusy()) {
                    follower.followPath(spike2Grab);
                    setPathState(6);
                }
                break;
            case (6):
                if (!follower.isBusy()) {
                    follower.followPath(spike2Drop);
                    setPathState(7);
                }
                break;
            case (7):
                if (!follower.isBusy()) {
                    follower.followPath(spike3Grab);
                    setPathState(8);
                }
                break;
            case (8):
                if (!follower.isBusy()) {
                    follower.followPath(spike3Drop);
                    setPathState(10);
                }
                break;
            case (10):
                if (!follower.isBusy()) {
                    follower.followPath(toWallGrab1);
                    setPathState(11);
                }
                break;
//            case (102):
//                if (!follower.isBusy()) {
//                    follower.followPath(toPreBar2);
//                    setPathState(11);
//                }
//                break;
            case (11):
                if (!follower.isBusy()) {
                    follower.followPath(toBar2);
                    setPathState(15);
                }
                break;
            case (15):
                if (!follower.isBusy() || pathTimer.getElapsedTime()>3500) {
                    follower.followPath(toWallGrab2);
                    setPathState(18);
                }
//                break;
//            case (17):
//                if (!follower.isBusy()) {
//                    follower.followPath(toPreBar3);
//                    setPathState(18);
//                }
//                break;
            case (18):
                if (!follower.isBusy()) {
                    follower.followPath(toBar3);
                    setPathState(24);
                }
                break;
            case (24):
                if (!follower.isBusy() || pathTimer.getElapsedTime()>3500) {
                    follower.followPath(toWallGrab3);
                    setPathState(27);
                }
                break;
            case (27):
                if (!follower.isBusy()) {
                    follower.followPath(toBar4);
                    setPathState(29);
                }
                break;
            case (29):
                if (!follower.isBusy() || pathTimer.getElapsedTime()>3500) {
                    follower.followPath(toWallGrab4);
                    setPathState(32);
                }
                break;
//            case (31):
//                if (!follower.isBusy()) {
//                    follower.followPath(toPreBar5);
//                    setPathState(32);
//                }
//                break;
            case (32):
                if (!follower.isBusy()) {
                    follower.followPath(toBar5);
                    setPathState(34);
                }
                break;
            case (34):
                if (!follower.isBusy() || pathTimer.getElapsedTime()>3500) {
                    follower.followPath(toWallGrab5);
                    setPathState(37);
                }
//                break;
//            case (36):
//                if (pathTimer.getElapsedTime() > 500) {
//                    follower.followPath(toPreBar6);
//                    setPathState(37);
//                }
//                break;
            case (37):
                if (!follower.isBusy()) {
                    follower.followPath(toBar6);
                    setPathState(38);
                }
                break;
            case (38):
                if (!follower.isBusy() || pathTimer.getElapsedTime()>3500) {
                    follower.followPath(toPark);
                    setPathState(-1);
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

//        slide3.setTargetPosition(anvilSlidePos);
//        slide1.setTargetPosition(hammerSlidePos);
//        slide2.setTargetPosition(hammerSlidePos);
//        lowTurn.setPosition(lowTurnPos);
//        lowBonk.setPosition(lowBonkPos);
//        lowRot.setPosition(lowRotPos);
//        lowClaw.setPosition(lowClawPos);
//        highTurn.setPosition(highTurnPos);
//        highBonkR.setPosition(highBonkPos);
//        highBonkL.setPosition(highBonkPos);
//        highClaw.setPosition(highClawPos);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        dumbBot.init2();
        dumbBot.slidereset2();
        dumbBot.highClaw.setPosition(1);
        dumbBot.slide3.setTargetPosition(0);
        dumbBot.slide1.setTargetPosition(0);
        dumbBot.slide2.setTargetPosition(0);
        dumbBot.lowTurn.setPosition(0.76);
        dumbBot.lowBonk.setPosition(0.17);
        dumbBot.lowRot.setPosition(0.33);
        dumbBot.lowClaw.setPosition(1);
        dumbBot.highTurnB.setPosition(0.225);
        dumbBot.highTurnT.setPosition(0.225);
        dumbBot.highBonkR.setPosition(0.9);
        dumbBot.highBonkL.setPosition(0.9);
        dumbBot.highClaw.setPosition(1);
        dumbBot.sniffer.setPosition((0.36));

        highColor = hardwareMap.get(RevColorSensorV3.class, "highColor");

        lowColor = hardwareMap.get(RevColorSensorV3.class, "lowColor");
        lowhsv = new float[3];

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.updatePythonInputs(1,0,0,0,0,0,0,0);

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
//        follower.followPath(total);
        setPathState(1);
    }

    @Override
    public void stop() {}
}