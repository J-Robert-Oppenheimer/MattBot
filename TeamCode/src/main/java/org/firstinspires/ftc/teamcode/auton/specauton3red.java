package org.firstinspires.ftc.teamcode.auton;

import android.graphics.Color;

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
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */


@Autonomous(name = "ASpecRed")
public class specauton3red extends OpMode {
    dumbMap dumbBot = new dumbMap(this);
    public Limelight3A limelight;
    RevColorSensorV3 highColor, lowColor;
    LLResult result;
    float[] lowhsv;
    public int anvilSlidePos, hammerSlidePos;
    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos, snifferPos;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, lowTime;;
    private int mode=2;
    private String pathState;
    //    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos;
    double desiredYIn;
    double desiredXIn;

    double limeLength = 4.5;
    double limeLengthX = -0.5;
    double lowTurnAngleDeg;
    double lowRotAngleDeg;
    double bonkLength = 8;
    double anvilChangeIn;
    double inchPerAnvil = -145.1 / (0.7121771654*2* Math.PI);
    double lowHue;
    public Servo lowClaw, lowRot, lowBonk, lowTurn, highClaw, highTurn, highBonkL, highBonkR;
    public DcMotor slide1, slide2, slide3;
    double[] array;
    ArrayList<Double> LimelightXOffsets = new ArrayList<>();
    ArrayList<Double> LimelightYOffsets = new ArrayList<>();
    ArrayList<Double> LimelightThetas = new ArrayList<>();
    public String state, lowState = "initToLime";
    boolean wasLastL, isLime = true;


    // Robot positions
    private final Pose startPoseBohdan = new Pose(7.25, 63, Math.toRadians(0));
    private final Pose specimen1 = new Pose(41.5, 70.5, Math.toRadians(0));
    private final Pose push1 = new Pose(63, 28, Math.toRadians(0));
    private final Pose push1Control = new Pose(22, 32, Math.toRadians(0));
    //    private final Pose pCon1 = new Pose(68.37, 39.77, Math.toRadians(0));
    private final Pose pCon1 = new Pose(53.3, 37.2, Math.toRadians(0));
    private final Pose push2 = new Pose(19.6, 28, Math.toRadians(0));
    private final Pose push2Control = new Pose(76.2, 21.5, Math.toRadians(0));
    //    private final Pose pCon2 = new Pose(76.2, 26.5, Math.toRadians(0));
    private final Pose pCon2 = new Pose(54, 28.4, Math.toRadians(0));
    private final Pose push3 = new Pose(63, 18, Math.toRadians(0));
    private final Pose pCon3 = new Pose(58.2, 20.4, Math.toRadians(90));
    private final Pose push4 = new Pose(19.2, 18, Math.toRadians(0));
    private final Pose push4Control = new Pose(64.45, 29, Math.toRadians(90));
    private final Pose push5 = new Pose(63, 9, Math.toRadians(90));
    private final Pose push6 = new Pose(11, 9, Math.toRadians(0));
    private final Pose push6Control = new Pose(77.8, 9, Math.toRadians(90));
    private final Pose wallPickPrep = new Pose(15, 26, Math.toRadians(0));
    private final Pose wallPick = new Pose(12, 26, Math.toRadians(0));
    private final Pose specimen2 = new Pose(41.5, 68.5, Math.toRadians(0));
    private final Pose specimen2Control = new Pose(30, 72.3, Math.toRadians(0));
    private final Pose specimen3 = new Pose(41.5, 66.5, Math.toRadians(0));
    private final Pose specimen3Control = new Pose(27, 70, Math.toRadians(0));
    private final Pose specimen4 = new Pose(41.5, 64.5, Math.toRadians(270));
    private final Pose specimen4Control = new Pose(27.6, 67.2, Math.toRadians(270));
    private final Pose specimen5 = new Pose(41.5, 72.5, Math.toRadians(270));
    private final Pose specimen5Control = new Pose(23.5, 72.3, Math.toRadians(0));

    private final Pose startPoseAlex = new Pose(7, 63.5, Math.toRadians(0));
    private final Pose preBar = new Pose(20, 67, Math.toRadians(0));
    private final Pose bar4 = new Pose(43, 74, Math.toRadians(0));
    private final Pose bar3 = new Pose(43, 75, Math.toRadians(0));
    private final Pose bar2 = new Pose(43, 76, Math.toRadians(0));
    private final Pose bar1 = new Pose(43, 73, Math.toRadians(0));
    private final Pose bar5 = new Pose(43, 74, Math.toRadians(0));
    private final Pose bar6 = new Pose(45.5, 77, Math.toRadians(0));
    private final Pose pushStart = new Pose(16, 36, Math.toRadians(0));
    private final Pose grab1 = new Pose(33, 36, Math.toRadians(-45));
    private final Pose grab2 = new Pose(28, 29.5, Math.toRadians(-45));
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
    private final Pose wallGrab = new Pose(12.75, 23, Math.toRadians(0));


    private final Pose wallGrab2 = new Pose(12.25, 23, Math.toRadians(0));

    private final Pose wallGrabCon1 = new Pose(30, 60,Math.toRadians(0));
    private final Pose wallGrabCon2 = new Pose(30, 33, Math.toRadians(0));
    private final Pose park = new Pose(15, 15, Math.toRadians(0));


    private PathChain push, spikeMid, sampleDrop, spike1Grab, spike1Drop, spike2Grab, spike2Drop, spike3Grab, spike3Drop, toBar1, toBar2, toBar3, toBar4, toBar5, toBar6, toPreBar1, toPreBar2, toPreBar3, toPreBar4, toPreBar5, toPreBar6, toPickUp1, toPickUp2, toPickUp3, toDropOff1, toDropOff2, toDropOff3, toWallGrab1, toWallGrab2, toWallGrab3, toWallGrab4, toWallGrab5, toPark, total, wallTurn, score1, speciminGrab1, score2, speciminGrab2, score3, speciminGrab3, score4, speciminGrab4, score5, toBar1two, pushTwo, barToPush, pushToGrab;

    public void buildPaths() {

        toPreBar1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(startPoseAlex),
                        new Point(preBar)
                )).setConstantHeadingInterpolation(0).build();

        toBar1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(startPoseAlex),
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

        barToPush = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bar1), new Point(pushStart)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        push = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pushStart), new Point(pCon1), new Point(push1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(push1), new Point(push2)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();


        pushTwo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(push2), new Point(pCon2), new Point(push3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(push3), new Point(push4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(new BezierCurve(new Point(push4),  new Point(pCon3), new Point(push5)))
//                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
//                .addPath(new BezierLine(new Point(push5), new Point(push6)))
//                .setLinearHeadingInterpolation(Math.toRadians(-90),0)
                .build();

        pushToGrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push4), new Point(wallGrab2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
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

        toBar1two = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(push6),
                        new Point(preBar),
                        new Point(bar2)
                )).setConstantHeadingInterpolation(0).build();



        toBar2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(wallGrab),
                        new Point(preBar),
                        new Point(bar2)
                )).setConstantHeadingInterpolation(0).build();

        toWallGrab2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(bar2),
                        new Point(wallGrabCon1),
                        new Point(wallGrabCon2),
                        new Point(wallGrab)
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
                new BezierCurve(
                        new Point(wallGrab),
                        new Point(preBar),
                        new Point(bar3)
                )).setConstantHeadingInterpolation(0).build();

        toWallGrab3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(bar3),
                        new Point(wallGrabCon1),
                        new Point(wallGrabCon2),
                        new Point(wallGrab)
                )).setConstantHeadingInterpolation(0).build();

//        toPreBar4 = follower.pathBuilder().addPath(
//                new BezierLine(
//                        new Point(wallGrab),
//                        new Point(preBar)
//                )).setConstantHeadingInterpolation(0).build();

        toBar4 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(wallGrab),
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
                        new Point(bar4),
                        new Point(preBar),
                        new Point(park)
                )).setConstantHeadingInterpolation(0).build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case ("BarOne"):
                hammerSlidePos = 900;
                highTurnPos = 0.75;
                lowBonkPos = 0.5;
                highBonkPos = 0.33;
                highClawPos = 1;
                if(pathTimer.getElapsedTime()>135) {
                    follower.followPath(toBar1);
                    setPathState("PlaceOne");
                }
                break;
            case ("PlaceOne"):
                if (pathTimer.getElapsedTime()>1300) {
                    follower.breakFollowing();
                    hammerSlidePos = 600;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    setPathState("HClawOpenOne");
                }
                break;
            case ("HClawOpenOne"):
                if (pathTimer.getElapsedTime()>500) {
                    hammerSlidePos = 650;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 0;
                    setPathState("barToPush");//REEEEEEE
                }
                break;

            case "barToPush":
                hammerSlidePos = 640;
                highTurnPos = 0.2;
                highBonkPos = 0.15;
                highClawPos = 0;
                anvilSlidePos = -100;
                snifferPos = 0.36;
                follower.followPath(barToPush);
                setPathState("PushOne");
                break;
//            case ("LimeOne"):
//                if (!follower.isBusy()) {
//                    Lime();
//                    if (!isLime) {
//                        setPathState("SampleHold");
//                        isLime = true;
//                    }
//                }
//                break;
//            case ("SampleHold"):
//                hammerSlidePos = 640;
//                highTurnPos = 0.2;
//                highBonkPos = 0.15;
//                highClawPos = 0;
//                anvilSlidePos = -100;
//                snifferPos = 0.36;
//                setPathState("SampleDrop");
//                break;
//
//
//            case ("SampleDrop"):
//                if (pathTimer.getElapsedTime()>250) {
//                    follower.followPath(sampleDrop);
//                    setPathState("SampleDropDrop");
//                }
//                break;
//            case ("SampleDropDrop"):
//                if (follower.getPose().roughlyEquals(wallGrab2,1) || pathTimer.getElapsedTime() > 1900) {
//                    lowClawPos = 0;
//                    hammerSlidePos = 0;
//                    highTurnPos = 0.2;
//                    highBonkPos = 0.9;
//                    highClawPos = 0;
//                    anvilSlidePos = 0;
//                    anvilSlidePos = 0;
//                    setPathState("barToPush");
//                }
//                break;

            case ("PushOne"):
                hammerSlidePos = 100;
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1.5) {
                    lowBonkPos = 0.22;
                    follower.followPath(push);
                    setPathState("PushTwo");
                }
                break;
            case ("PushTwo"):
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(pushTwo);
                    setPathState("GrabOneOpen");
                }
                break;
            case ("GrabOneOpen"):
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2) {
                    hammerSlidePos = 640;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 0;
                    follower.followPath(pushToGrab);
                    setPathState("GrabOneClosed");
                }
                break;

            case "GrabOneClosed":
                if (pathTimer.getElapsedTime()>1500) {
                    hammerSlidePos = 640;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 1;
                    setPathState("BarTwo");
                }
                break;

            case ("BarTwo"):
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(toBar2);
                    hammerSlidePos = 900;
                    highTurnPos = 0.75;
                    highBonkPos = 0.33;
                    highClawPos = 1;
                    setPathState("PlaceTwo");
                }
                break;
            case ("PlaceTwo"):
                if (!follower.isBusy() || pathTimer.getElapsedTime()>2500) {
                    follower.breakFollowing();
                    hammerSlidePos = 650;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    setPathState("PlaceTwoOpen");
                }
                break;

            case ("PlaceTwoOpen"):
                if (pathTimer.getElapsedTime()>700) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 0;
                    follower.followPath(toWallGrab2);
                    setPathState("GrabTwoOpen");
                }
                break;

            case ("GrabTwoOpen"):
                if (pathTimer.getElapsedTime() > 250) {
                    hammerSlidePos = 640;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 0;
                    if (!follower.isBusy() && pathTimer.getElapsedTime() > 2000) {
                        setPathState("GrabTwoClosed");
                    }
                }
                break;

            case "GrabTwoClosed":
                if (!follower.isBusy()) {
                    hammerSlidePos = 640;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 1;
                    if (pathTimer.getElapsedTime() > 500) {
                        setPathState("BarThree");
                    }
                }
                break;

            case ("BarThree"):
                if (!follower.isBusy()) {
                    follower.followPath(toBar3);
                    hammerSlidePos = 900;
                    highTurnPos = 0.75;
                    highBonkPos = 0.33;
                    highClawPos = 1;
                    setPathState("PlaceThree");
                }
                break;
            case ("PlaceThree"):
                if (!follower.isBusy() || pathTimer.getElapsedTime()>2500) {
                    follower.breakFollowing();
                    hammerSlidePos = 650;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    setPathState("PlaceThreeOpen");
                }
                break;
            case ("PlaceThreeOpen"):
                if (pathTimer.getElapsedTime()>200) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.2;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    follower.followPath(toWallGrab3);
                    setPathState("GrabThreeOpen");
                }
                break;

            case ("GrabThreeOpen"):
                hammerSlidePos = 640;
                highTurnPos = 0.2;
                highBonkPos = 0.15;
                highClawPos = 0;
                if (!follower.isBusy()) {
                    setPathState("GrabThreeClosed");
                }
                break;

            case "GrabThreeClosed":
                if (!follower.isBusy()) {
                    hammerSlidePos = 640;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 1;
                    if (pathTimer.getElapsedTime() > 500) {
                        setPathState("BarFour");
                    }
                }
                break;

            case ("BarFour"):
                if (!follower.isBusy()) {
                    follower.followPath(toBar3);
                    hammerSlidePos = 900;
                    highTurnPos = 0.75;
                    highBonkPos = 0.33;
                    highClawPos = 1;
                    setPathState("PlaceFour");
                }
                break;
            case ("PlaceFour"):
                if (!follower.isBusy() || pathTimer.getElapsedTime()>2500) {
                    follower.breakFollowing();
                    hammerSlidePos = 650;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    setPathState("PlaceFourOpen");
                }
                break;

            case ("PlaceFourOpen"):
                if (pathTimer.getElapsedTime()>200) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    setPathState("PARK");
                }
                break;



            case ("PARK"):
                if (!follower.isBusy() || pathTimer.getElapsedTime()>3500) {
                    follower.followPath(toPark);
                    hammerSlidePos = 0;
                    highTurnPos = 0.2;
                    highBonkPos = 0.9;
                    highClawPos = 0;
                    setPathState("haolin smelly");
                }
                break;
        }
    }


    public void setPathState(String pState) {
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
        dumbBot.highTurnB.setPosition(highTurnPos);
        dumbBot.highTurnT.setPosition(highTurnPos);
        dumbBot.highBonkR.setPosition(highBonkPos);
        dumbBot.highBonkL.setPosition(highBonkPos);
        dumbBot.highClaw.setPosition(highClawPos);
        dumbBot.sniffer.setPosition((snifferPos));

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
        dumbBot.lowTurn.setPosition(0.53);
        dumbBot.lowBonk.setPosition(0.19);
        dumbBot.lowRot.setPosition(0.33);
        dumbBot.lowClaw.setPosition(1);
        dumbBot.highTurnB.setPosition(0.75);
        dumbBot.highTurnT.setPosition(0.75);
        dumbBot.highBonkR.setPosition(0.35);
        dumbBot.highBonkL.setPosition(0.35);
        dumbBot.highClaw.setPosition(1);
        dumbBot.sniffer.setPosition((0.36));

        highColor = hardwareMap.get(RevColorSensorV3.class, "highColor");

        lowColor = hardwareMap.get(RevColorSensorV3.class, "lowColor");
        lowhsv = new float[3];

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.updatePythonInputs(1,0,0,0,0,0,0,0);
        limelight.start();

        pathTimer = new Timer();
        lowTime = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPoseAlex);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
//        follower.followPath(total);
        setPathState("BarOne");
    }

    @Override
    public void stop() {}
    public void Lime(){

        switch (lowState) {
            case "initToLime":
                if (!wasLastL) {
                    wasLastL = true;

                    anvilSlidePos = 0;
                    lowTurnPos = 0.2;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.84;
                    lowClawPos = 1;
                    snifferPos = 0.36;
                    mode = 2;
                }

                if (lowTime.getElapsedTime() > 100) {
                    changeStateLow("limeSearching");
                }
                break;

            case "limeSearching":
                if (!wasLastL) {
                    limelight.updatePythonInputs(mode, 0, 0, 0, 0, 0, 0, 0);
                    wasLastL = true;

                    lowTurnPos = 0.2;
                    lowBonkPos = 0.55;
                    lowRotPos = 0.84;
                    lowClawPos = 0;
                    snifferPos = 0.92;

                }

                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    if (array[0] == 1) {
                        changeStateLow("limeCheck");
                    } else {
                        if (anvilSlidePos > -1200) anvilSlidePos -= 5;
                        else anvilSlidePos = 0;
                    }
                }
                break;
            case "limeCheck":
                if (!wasLastL) {
                    wasLastL = true;

                    anvilSlidePos = anvilSlidePos;
                    lowTurnPos = 0.2;
                    lowBonkPos = 0.55;
                    lowRotPos = 0.84;
                    lowClawPos = 0;
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
                        changeStateLow("limeSearching");
                    }
                }
                if (lowTime.getElapsedTime() > 150) {
                    changeStateLow("limeFound");
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
                    lowBonkPos = 0.55;
                    lowRotPos = 0.18 + lowRotAngleDeg / 180 * 0.68;
                    lowClawPos = 0;
                    snifferPos = 0.36;

                }

                if (lowTime.getElapsedTime() > 600) {
                    changeStateLow("limePreBite");
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
                    snifferPos = 0.6;
                }

                if (lowTime.getElapsedTime() > 200) {
                    changeStateLow("limeBite");
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
                }

                if (lowTime.getElapsedTime() > 200) {
                    changeStateLow("limeRevCheck");
                }
                break;
            case "limeRevCheck":
                if (!wasLastL) {
                    wasLastL = true;

                    anvilSlidePos = anvilSlidePos;
                    lowTurnPos = 0.54;
                    lowBonkPos = 0.55;
                    lowRotPos = 0.84;
                    lowClawPos = 1;
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
                    if (lowTime.getElapsedTime() > 400) {
                        changeStateLow("end");
                        isLime = false;
                    }
                }  else {
                    if (lowTime.getElapsedTime() > 400) {
                        changeStateLow("limeSearching");
                    }
                }
                break;
        }
        telemetry.addData("limestate: ", lowState);
    }
    public void changeStateLow(String newState){
        lowState = newState;
        wasLastL = false;
        lowTime.resetTimer();
    }
}