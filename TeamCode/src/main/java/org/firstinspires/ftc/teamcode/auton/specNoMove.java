package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
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


@Autonomous(name = "specNoMove")
public class specNoMove extends OpMode {
    dumbMap dumbBot = new dumbMap(this);
    public int anvilSlidePos, hammerSlidePos;
    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
//    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos;

    public Servo lowClaw, lowRot, lowBonk, lowTurn, highClaw, highTurn, highBonkL, highBonkR;
    public DcMotor slide1, slide2, slide3;


    // Robot positions
    private final Pose startPose = new Pose(8, 64.5, Math.toRadians(0));
    private final Pose preBar = new Pose(20, 67, Math.toRadians(0));
    private final Pose bar1 = new Pose(40, 78, Math.toRadians(0));
    private final Pose bar2 = new Pose(40, 74.5, Math.toRadians(0));
    private final Pose bar3 = new Pose(40, 71, Math.toRadians(0));
    private final Pose bar4 = new Pose(40, 68.5, Math.toRadians(0));
    private final Pose bar5 = new Pose(40, 65, Math.toRadians(0));
    private final Pose bar6 = new Pose(40, 61.5, Math.toRadians(0));
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

    private PathChain push, toBar1, toBar2, toBar3, toBar4, toBar5, toBar6, toPreBar1, toPreBar2, toPreBar3, toPreBar4, toPreBar5, toPreBar6, toPickUp1, toPickUp2, toPickUp3, toDropOff1, toDropOff2, toDropOff3, toWallGrab1, toWallGrab2, toWallGrab3, toWallGrab4, toWallGrab5, toPark, total, wallTurn;

    public void buildPaths() {

        toPreBar1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(startPose),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).build();

        toBar1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(preBar),
                        new Point(bar1)
                )).setConstantHeadingInterpolation(0).build();

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
                        new Point(thirdPush),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

        toPreBar2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).build();

        toBar2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(preBar),
                        new Point(bar2)
                )).setConstantHeadingInterpolation(0).build();

        toWallGrab2 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(bar2),
                        new Point(wallGrab2)
                )).setConstantHeadingInterpolation(-90).build();

        wallTurn = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(wallGrab2),
                        new Point(45, 30),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

        toPreBar3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).build();

        toBar3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(preBar),
                        new Point(bar3)
                )).setConstantHeadingInterpolation(0).build();

        toWallGrab3 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(bar3),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

        toPreBar4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).build();

        toBar4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(preBar),
                        new Point(bar4)
                )).setConstantHeadingInterpolation(0).build();

        toWallGrab4 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(bar4),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

        toPreBar5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).build();

        toBar5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(preBar),
                        new Point(bar5)
                )).setConstantHeadingInterpolation(0).build();

        toWallGrab5 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(bar5),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

        toPreBar6 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(wallGrab),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).build();

        toBar6 = follower.pathBuilder().addPath(
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
                anvilSlidePos = 0;
                hammerSlidePos = 470;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.205;
                highClawPos = 0.5;
                //follower.followPath(toPreBar1);
                setPathState(2);
                break;
            case (2):
                if (!follower.isBusy()) {
                    //follower.followPath(toBar1);
                    setPathState(3);
                }
                break;

            case (3):
                anvilSlidePos = -1200;
                hammerSlidePos = 0;
                lowTurnPos = 0;
                lowBonkPos = 0.65;
                lowRotPos = 0;
                lowClawPos = 1;
                highTurnPos = 0.76;
                highBonkPos = 0.1475;
                highClawPos = 0.5;
                if (!follower.isBusy()) {
                    //follower.followPath(push);
                    setPathState(100);
                }
                break;
            case (100):
                anvilSlidePos = 0;
                hammerSlidePos = 0;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.205;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(4);
                }
                break;
            case (4):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(10);
                }
                break;
            case (10):
                if (!follower.isBusy()) {
                    //follower.followPath(toWallGrab1);
                    setPathState(101);
                }
                break;
            case (101):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 2000) {
                    setPathState(102);
                }
                break;
            case (102):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.5;
                if (pathTimer.getElapsedTime() > 500) {
                    //follower.followPath(toPreBar2);
                    setPathState(11);
                }
                break;
            case (11):
                anvilSlidePos = 0;
                hammerSlidePos = 470;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.205;
                highClawPos = 0.5;
                if (!follower.isBusy()) {
                    //follower.followPath(toBar2);
                    setPathState(14);
                }
                break;
            case (14):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(142);
                }
                break;
            case (142):
                //LL Grab
                if (!follower.isBusy()) {
                    setPathState(15);
                }
                break;
            case (15):
                if (!follower.isBusy()) {
                    //follower.followPath(toWallGrab2);
                    setPathState(152);
                }
                break;
            case (152):
                if (!follower.isBusy()) {
                    //follower.followPath(wallTurn);
                    setPathState(120);
                }
                break;
            case (120):
                anvilSlidePos = 0;
                hammerSlidePos = 0;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.205;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(16);
                }
                break;
            case (16):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 2500) {
                    setPathState(17);
                }
                break;
            case (17):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.5;
                if (pathTimer.getElapsedTime() > 500) {
                    //follower.followPath(toPreBar3);
                    setPathState(18);
                }
                break;
            case (18):
                anvilSlidePos = 0;
                hammerSlidePos = 470;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.205;
                highClawPos = 0.5;
                if (!follower.isBusy()) {
                    //follower.followPath(toBar3);
                    setPathState(182);
                }
                break;
            case (182):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(24);
                }
                break;
            case (24):
                if (!follower.isBusy()) {
                    //follower.followPath(toWallGrab1);
                    setPathState(25);
                }
                break;
            case (25):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(26);
                }
                break;
            case (26):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.5;
                if (pathTimer.getElapsedTime() > 500) {
                    //follower.followPath(toPreBar4);
                    setPathState(27);
                }
                break;
            case (27):
                anvilSlidePos = 0;
                hammerSlidePos = 470;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.205;
                highClawPos = 0.5;
                if (!follower.isBusy()) {
                    //follower.followPath(toBar4);
                    setPathState(28);
                }
                break;
            case (28):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(29);
                }
                break;
            case (29):
                if (!follower.isBusy()) {
                    //follower.followPath(toWallGrab1);
                    setPathState(30);
                }
                break;
            case (30):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(31);
                }
                break;
            case (31):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.5;
                if (pathTimer.getElapsedTime() > 500) {
                    //follower.followPath(toPreBar5);
                    setPathState(32);
                }
                break;
            case (32):
                anvilSlidePos = 0;
                hammerSlidePos = 470;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.205;
                highClawPos = 0.5;
                if (!follower.isBusy()) {
                    //follower.followPath(toBar5);
                    setPathState(33);
                }
                break;
            case (33):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(34);
                }
                break;
            case (34):
                if (!follower.isBusy()) {
                    //follower.followPath(toWallGrab1);
                    setPathState(35);
                }
                break;
            case (35):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(36);
                }
                break;
            case (36):
                anvilSlidePos = 0;
                hammerSlidePos = 720;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.2;
                highBonkPos = 0.105;
                highClawPos = 0.5;
                if (pathTimer.getElapsedTime() > 500) {
                    //follower.followPath(toPreBar6);
                    setPathState(37);
                }
                break;
            case (37):
                anvilSlidePos = 0;
                hammerSlidePos = 470;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.205;
                highClawPos = 0.5;
                if (!follower.isBusy()) {
                    //follower.followPath(toBar6);
                    setPathState(38);
                }
                break;
            case (38):
                anvilSlidePos = 0;
                hammerSlidePos = 470;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.1475;
                highClawPos = 0.5;
                if (!follower.isBusy()) {
                    //follower.followPath(toPark);
                    setPathState(17);
                }
                break;
            case (39):
                anvilSlidePos = 0;
                hammerSlidePos = 470;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.12;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.1475;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState(18);
                }
                break;
            case (40):
                anvilSlidePos = 0;
                hammerSlidePos = 470;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.12;
                lowClawPos = 0.27;
                highTurnPos = 0.76;
                highBonkPos = 0.25;
                highClawPos = 0.22;
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState(19);
                }
                break;
            case (41):
                anvilSlidePos = 0;
                hammerSlidePos = 0;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0;
                highTurnPos = 0.1;
                highBonkPos = 0.255;
                highClawPos = 0;
                setPathState(-1);
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

        slide3.setTargetPosition(anvilSlidePos);
        slide1.setTargetPosition(hammerSlidePos);
        slide2.setTargetPosition(hammerSlidePos);
        lowTurn.setPosition(lowTurnPos);
        lowBonk.setPosition(lowBonkPos);
        lowRot.setPosition(lowRotPos);
        lowClaw.setPosition(lowClawPos);
        highTurn.setPosition(highTurnPos);
        highBonkR.setPosition(highBonkPos);
        highBonkL.setPosition(highBonkPos);
        highClaw.setPosition(highClawPos);

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {

        slide1 = this.hardwareMap.dcMotor.get("lefthammer"); //
        slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide1.setDirection(DcMotorSimple.Direction.REVERSE);
        slide1.setTargetPosition(0);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide2 = this.hardwareMap.dcMotor.get("righthammer"); //
        slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide2.setTargetPosition(0);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide3 = this.hardwareMap.dcMotor.get("anvilslide"); //0 - -400
        slide3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide3.setDirection(DcMotorSimple.Direction.REVERSE);
        slide3.setTargetPosition(0);
        slide3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide1.setTargetPosition(0);
        slide2.setTargetPosition(0);
        slide3.setTargetPosition(0);
        slide1.setPower(1);
        slide2.setPower(1);
        slide3.setPower(1);

        lowClaw = this.hardwareMap.get(Servo.class, "lowClaw" ); // 0.27 (close) 0.5 (open)
        lowRot = this.hardwareMap.get(Servo.class, "lowRot" ); // 0.52 (par) 0.85 (perp) 0.21 (transfer)
        lowBonk = this.hardwareMap.get(Servo.class, "lowBonk" ); // 0.77 (down) 0.36 (up) 0.22  (back)
        lowTurn = this.hardwareMap.get(Servo.class, "lowTurn" ); // 0.48 straight 0.84 left

        highClaw = this.hardwareMap.get(Servo.class, "highClaw" ); // 0.5 (close) 0.22 (open)
        highBonkL = this.hardwareMap.get(Servo.class, "highBonkL" );// spec grab  0.105 spec place 0.135(little more?) blockplace 0.1 transfer 0.26 up 0.08
        highBonkR = this.hardwareMap.get(Servo.class, "highBonkR" );
        highBonkR.setDirection(Servo.Direction.REVERSE);
        highTurn = this.hardwareMap.get(Servo.class, "highTurn" ); // spec grab 0.11 spec place 1 blockplace 0.2 transfer 0.73


        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        slide3.setTargetPosition(0);
        slide1.setTargetPosition(0);
        slide2.setTargetPosition(0);
        lowTurn.setPosition(0.48);
        lowBonk.setPosition(0.22);
        lowRot.setPosition(0.52);
        lowClaw.setPosition(0.27);
        highTurn.setPosition(0.74);
        highBonkL.setPosition(0.11);
        highBonkR.setPosition(0.11);
        highClaw.setPosition(0.5);

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