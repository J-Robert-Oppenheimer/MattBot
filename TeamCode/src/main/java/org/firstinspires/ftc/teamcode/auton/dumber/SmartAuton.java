package org.firstinspires.ftc.teamcode.auton.dumber;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;

@Autonomous(name = "Smart_Buck")
@Config
public class SmartAuton extends OpMode {
    public Limelight3A limelight;
    LLResult result;
    double[] array;
    public int mode = 1;
    String stage;
    int iterCount;
    boolean wasLast;
    public static boolean RUNARM = true;
    double desiredY;
    double biProductY;
    double desiredX;
    double lowTurnAngle;
    double lowRotAngle;
    int anvilTarget = 0;
    double lowTurnTarget;
    double lowRotTarget;
    RevColorSensorV3 highColor, lowColor;
    float[] lowhsv;
    double lowHue;
    double desiredYIn;
    double desiredXIn;
    double lowTurnAngleDeg;
    double lowRotAngleDeg;
    double anvilChangeIn;
    double bonkLength = 8;
    double limeLength = 4.5;
    double limeLengthX = -0.5;
    double inchPerAnvil = -145.1 / (0.7121771654*2* Math.PI);
    public String state, lowState = "limeSearching";
    boolean wasLastL, isLime = true, parker = false;

    ArrayList<Double> LimelightXOffsets = new ArrayList<>();
    ArrayList<Double> LimelightYOffsets = new ArrayList<>();
    ArrayList<Double> LimelightThetas = new ArrayList<>();
    dumbMap dumbBot = new dumbMap(this);
    public int anvilSlidePos, hammerSlidePos;

    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos, snifferPos;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer, lowTime;
    private String pathState = "among us is a 2018 online multiplayer social deduction game developed by";

     public static double startPoseX = 7, startPoseY = 112, startPoseHeading = 0;
    public static double scorePoseX=17, scorePoseY=130.5, scorePoseHeading=-45;
    public static double pickThreeX=31.5, pickThreeY=135, pickThreeHeading=0;
    public static double pickTwoX=26.5, pickTwoY=131, pickTwoHeading=0;
    public static double pickOneX=26.5, pickOneY=121.5, pickOneHeading=0;
    public static double pickFromSubOneX=71.7, pickFromSubOneY=98, pickFromSubOneHeading=-90;
    public static double pickFromSubTwoX=75.7, pickFromSubTwoY=98, pickFromSubTwoHeading=-90;
    public static double subToBuckMidX=48, subToBuckMidY=120, subToBuckMidHeading=-90;
    public static double preParkX=71.7, preParkY=105, preParkHeading=90;
    public static double ParkX=71.7, ParkY=100, ParkHeading=90;
    private Pose startPose = new Pose();
    private Pose scorePose = new Pose();
    private Pose pickThree = new Pose();
    private Pose pickTwo =  new Pose();
    private Pose pickOne =  new Pose();
    private Pose pickFromSubOne = new Pose();
    private Pose pickFromSubTwo = new Pose();
    private Pose subToBuckMid = new Pose();
    private Pose prePark = new Pose();
    private Pose Park = new Pose();


    private PathChain scoreOne, pickUpOne, scoreTwo, pickUpTwo, scoreThree, pickUpThree, scoreFour, pickUpSubOne, pickSubOneBack, scoreFive, pickUpSubTwo, scoreSix, park;
//    private Path scoreOne, pickUpOne, scoreTwo, pickUpTwo, scoreThree, pickUpThree, scoreFour, pickUpSubOne, scoreFive, pickUpSubTwo, scoreSix;

    public void buildPaths() {
        scoreOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();


        pickUpOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickOne)))
                .setConstantHeadingInterpolation(pickOne.getHeading())
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

        pickUpThree= follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickThree)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickThree.getHeading())
                .build();

        scoreFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickThree), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickThree.getHeading(), scorePose.getHeading())
                .build();

        pickUpSubOne= follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickFromSubOne)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickFromSubOne.getHeading())
                .build();

        pickSubOneBack= follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickFromSubOne), new Point(subToBuckMid)))
                .setLinearHeadingInterpolation(pickFromSubOne.getHeading(), subToBuckMid.getHeading())
                .addPath(new BezierLine(new Point(subToBuckMid), new Point(scorePose)))
                .setLinearHeadingInterpolation(subToBuckMid.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickFromSubOne), new Point(prePark)))
                .setLinearHeadingInterpolation(pickFromSubOne.getHeading(), prePark.getHeading())
                .addPath(new BezierLine(new Point(prePark), new Point(Park)))
                .setLinearHeadingInterpolation(prePark.getHeading(), Park.getHeading())
                .build();

        scoreFive = follower.pathBuilder()
                .addPath(new BezierLine(new Point(subToBuckMid), new Point(scorePose)))
                .setLinearHeadingInterpolation(subToBuckMid.getHeading(), scorePose.getHeading())
                .build();

        pickUpSubTwo= follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickFromSubTwo)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickFromSubTwo.getHeading())
                .build();

        scoreSix = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickFromSubTwo), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickFromSubTwo.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case "ScoreOne": //raise slides
                hammerSlidePos = 2120;
                anvilSlidePos = 0;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.75;
                highBonkPos = 0.6;
                snifferPos = 0.36;
                highClawPos = 1;
                if(pathTimer.getElapsedTime()>350) setPathState("ScoreOne2");
                break;
            case "ScoreOne2": //raise slides
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.75;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.36;
                follower.followPath(scoreOne, true);
                setPathState("PreDropOne");
                break;

            case "PreDropOne":
                if(follower.getPose().roughlyEquals(scorePose, 0.15)){
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;
                    snifferPos = 0.36;
                    hammerSlidePos = 2120;
                    highClawPos = 1;
                    setPathState("test");
                }
                break;

            case "test":
                if (pathTimer.getElapsedTime() > 500) setPathState("DropOne");

            case "DropOne": //open claw
                    anvilSlidePos = 0;
                    hammerSlidePos = 2120;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;//fix me
                    highClawPos = 0;
                    snifferPos = 0.36;
                    if(pathTimer.getElapsedTime()>350){ setPathState("idiot");}

                break;

            case "idiot":
                highBonkPos = 0.6;
                if (pathTimer.getElapsedTime() > 150) setPathState("PickupOne");//PICKUPONE
                break;
            case "PickupOne": //lower slides, extend low arm
                if (!follower.isBusy()) {
                    anvilSlidePos = -50;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.6;
                    highClawPos = 0;
                    snifferPos = 0.36;
                    follower.followPath(pickUpOne, true);
                    setPathState("GrabPrepOne");
                }
                break;

            case "GrabPrepOne": //lower bonk
                if(!follower.isBusy()){
                    anvilSlidePos = -400;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.7;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.6;
                    highClawPos = 0;
                    snifferPos = 0.36;
                    if (pathTimer.getElapsedTime() > 1300) {
                        setPathState("GrabOne");
                    }
                }
                break;

            case "GrabOne": //close
                anvilSlidePos = -400;
                hammerSlidePos = 710;
                lowTurnPos = 0.53;
                lowBonkPos = 0.71;
                lowRotPos = 0.54;
                lowClawPos = 1;
                highTurnPos = 0.16;
                highBonkPos = 1;
                highClawPos = 0;
                snifferPos = 0.36;
                if (pathTimer.getElapsedTime() > 650) {
                    setPathState("TransferOnePrep");
                }
                break;

            case "TransferOnePrep":
                anvilSlidePos = -100;
                hammerSlidePos = 710;
                lowTurnPos = 0.76;
                lowBonkPos = 0.17;
                lowRotPos = 0.37;
                lowClawPos = 1;
                highTurnPos = 0.16;
                highBonkPos = 1;
                highClawPos = 0;
                snifferPos = 0.36;
                if (pathTimer.getElapsedTime() > 600) {
                    setPathState("TransferOneBoth");
                }
                break;

            case "TransferOneBoth":
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
                if (pathTimer.getElapsedTime() > 200) {
                    setPathState("TransferOneHigh");
                }
                break;
            case "TransferOneHigh":
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
                if (pathTimer.getElapsedTime() > 300) {
                    setPathState("ScoreTwo");
                }
                break;

            case "ScoreTwo": //transfer to sample place
                if(!follower.isBusy()) {
                    hammerSlidePos = 2120;
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;
                    snifferPos = 0.36;
                    highClawPos = 1;
                    if (pathTimer.getElapsedTime() > 400) {
                    follower.followPath(scoreTwo, true);
                    setPathState("PreDropTwo");
                    }
                }
                break;
            case "PreDropTwo":
                if ((follower.getPose().roughlyEquals(scorePose,0.25)) || pathTimer.getElapsedTime() > 1200) {
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;
                    snifferPos = 0.36;
                    hammerSlidePos = 2120;
                    highClawPos = 1;
                    if (pathTimer.getElapsedTime()>1450) {
                        setPathState("DropTwo");
                    }
                }
                break;

            case "DropTwo": //open claw
                    anvilSlidePos = 0;
                    hammerSlidePos = 2120;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;
                    highClawPos = 0;
                    snifferPos = 0.36;
                    if(pathTimer.getElapsedTime()>700){ setPathState("DropTwo2");}

                break;

            case "DropTwo2":
                    anvilSlidePos = 0;
                    hammerSlidePos = 2120;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.6;
                    highClawPos = 0;
                    snifferPos = 0.36;
                    if(pathTimer.getElapsedTime()>100){ setPathState("PickupTwo");}

                break;



////TODO***************************************************** part 2 ********************************************
            case "PickupTwo": //lower slides, extend low arm
                if(pathTimer.getElapsedTime() > 200) {
                    anvilSlidePos = -380;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.6;
                    highClawPos = 0;
                    snifferPos = 0.36;
                    follower.followPath(pickUpTwo, true);
                    setPathState("GrabPrepTwo");
                }
                break;

            case "GrabPrepTwo": //lower bonk
                if(!follower.isBusy()){
                    anvilSlidePos = -400;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.71;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.6;
                    highClawPos = 0;
                    snifferPos = 0.36;
                    if (pathTimer.getElapsedTime() > 1500) {
                        setPathState("GrabTwo");
                    }
                }
                break;

            case "GrabTwo": //close
                anvilSlidePos = -400;
                hammerSlidePos = 710;
                lowTurnPos = 0.53;
                lowBonkPos = 0.71;
                lowRotPos = 0.54;
                lowClawPos = 1;
                highTurnPos = 0.16;
                highBonkPos = 1;
                highClawPos = 0;
                snifferPos = 0.36;
                if (pathTimer.getElapsedTime() > 650) {
                    setPathState("TransferTwoPrep");
                }
                break;

            case "TransferTwoPrep":
                anvilSlidePos = 0;
                hammerSlidePos = 710;
                lowTurnPos = 0.76;
                lowBonkPos = 0.17;
                lowRotPos = 0.37;
                lowClawPos = 1;
                highTurnPos = 0.16;
                highBonkPos = 1;
                highClawPos = 0;
                snifferPos = 0.36;
                if (pathTimer.getElapsedTime() > 600) {
                    setPathState("TransferTwoBoth");
                }
                break;

            case "TransferTwoBoth":
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
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState("TransferTwoHigh");
                }
                break;
            case "TransferTwoHigh":
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
                if (pathTimer.getElapsedTime() > 300) {
                    setPathState("ScoreThree");
                }
                break;

            case "ScoreThree": //transfer to sample place
                if(!follower.isBusy()) {
                    hammerSlidePos = 2030;
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;
                    snifferPos = 0.36;
                    highClawPos = 1;
                    if (pathTimer.getElapsedTime() > 500) {
                        follower.followPath(scoreThree, true);
                        setPathState("PreDropThree");
                    }
                }
                break;
            case "PreDropThree":
                if (((follower.getPose().roughlyEquals(scorePose,0.3)) || pathTimer.getElapsedTime() > 1200)) {
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;
                    snifferPos = 0.36;
                    hammerSlidePos = 2120;
                    highClawPos = 1;
                    if (pathTimer.getElapsedTime()>1450) {
                        setPathState("DropThree");
                    }
                }
                break;

            case "DropThree": //open claw
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.53;
                lowBonkPos = 0.6;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.75;
                highBonkPos = 0.67;
                highClawPos = 0;
                snifferPos = 0.36;
                if(pathTimer.getElapsedTime()>300){ setPathState("DropThree2");}

                break;

            case "DropThree2":
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.53;
                lowBonkPos = 0.6;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.75;
                highBonkPos = 0.6;
                highClawPos = 0;
                snifferPos = 0.36;
                if(pathTimer.getElapsedTime()>200){ setPathState("PickupThree");}

                break;
////TODO************************************ part 3 ************************************
            case "PickupThree": //lower slides, extend low arm
                if(pathTimer.getElapsedTime() > 100) {
                    anvilSlidePos = -180;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.35;
                    lowBonkPos = 0.6;
                    lowRotPos = 0.30;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.6;
                    highClawPos = 0;
                    snifferPos = 0.36;
                    follower.followPath(pickUpThree, true);
                    setPathState("GrabPrepThree");
                }
                break;

            case "GrabPrepThree": //lower bonk
                if(!follower.isBusy()){
                    anvilSlidePos = -310;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.3;
                    lowBonkPos = 0.71;
                    lowRotPos = 0.28;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.6;
                    highClawPos = 0;
                    snifferPos = 0.36;
                    if (pathTimer.getElapsedTime() > 1400) {
                        setPathState("GrabThree");
                    }
                }
                break;

            case "GrabThree": //close
                anvilSlidePos = -180;
                hammerSlidePos = 900;
                lowTurnPos = 0.35;
                lowBonkPos = 0.71;
                lowRotPos = 0.30;
                lowClawPos = 1;
                highTurnPos = 0.16;
                highBonkPos = 1;
                highClawPos = 0;
                snifferPos = 0.36;
                if (pathTimer.getElapsedTime() > 650) {
                    setPathState("TransferThreePrep");
                }
                break;

            case "TransferThreePrep":
                anvilSlidePos = -100;
                hammerSlidePos = 710;
                lowTurnPos = 0.76;
                lowBonkPos = 0.17;
                lowRotPos = 0.37;
                lowClawPos = 1;
                highTurnPos = 0.16;
                highBonkPos = 1;
                highClawPos = 0;
                snifferPos = 0.36;
                if (pathTimer.getElapsedTime() > 1000) {
                    setPathState("TransferThreeBoth");
                }
                break;

            case "TransferThreeBoth":
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
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState("TransferThreeHigh");
                }
                break;
            case "TransferThreeHigh":
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
                if (pathTimer.getElapsedTime() > 300) {
                    setPathState("ScoreFour");
                }
                break;

            case "ScoreFour": //transfer to sample place
                if(!follower.isBusy()) {
                    hammerSlidePos = 2120;
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;
                    snifferPos = 0.36;
                    highClawPos = 1;
                    if (pathTimer.getElapsedTime() > 400) {
                        follower.followPath(scoreFour, true);
                        setPathState("PreDropFour");
                    }
                }
                break;
            case "PreDropFour":
                if (((follower.getPose().roughlyEquals(scorePose,0.25)) || pathTimer.getElapsedTime() > 1200)) {
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;
                    snifferPos = 0.36;
                    hammerSlidePos = 2120;
                    highClawPos = 1;
                    if (pathTimer.getElapsedTime()>1450) {
                        setPathState("DropFour");
                    }
                }
                break;

            case "DropFour": //open claw
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.75;
                highBonkPos = 0.67;
                highClawPos = 0;
                snifferPos = 0.36;
                if(pathTimer.getElapsedTime()>200){ setPathState("DropFour2");}

                break;

            case "DropFour2":
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.75;
                highBonkPos = 0.6;
                highClawPos = 0;
                snifferPos = 0.36;
                if (pathTimer.getElapsedTime() > 100) {
                    setPathState("PickupSubOne");
                }

                break;
////TODO***************************************** part 4 **************************************first sub


            case "PickupSubOne": //sample place drop to init
                anvilSlidePos = 0;
                hammerSlidePos = 800;
                lowTurnPos = 0.2;
                lowBonkPos = 0.55;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.19;
                highBonkPos = 0.6;
                highClawPos = 0;
                snifferPos = 0.92;
                follower.followPath(pickUpSubOne, true);
                setPathState("LimesOne");
                changeStateLow("limeSearching");
                break;

            case "LimesOne":
                if (follower.getPose().roughlyEquals(pickFromSubOne, 2.5)) {
                    Lime();
                    if (!isLime) {
                        setPathState((parker ? "Park" : "ScoreFiveCycle"));

                        isLime = true;
                    }
                }
                break;

            case "ScoreFiveCycle":
                follower.followPath(pickSubOneBack);
                setPathState("TransferFive");
                break;

            case "TransferFive":
                anvilSlidePos = -100;
                hammerSlidePos = 690;
                lowTurnPos = 0.76;
                lowBonkPos = 0.17;
                lowRotPos = 0.37;
                lowClawPos = 1;
                highTurnPos = 0.16;
                highBonkPos = 1;
                highClawPos = 0;
                snifferPos = 0.36;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState("TransferFiveBoth");
                }
                break;

            case "TransferFiveBoth":
                anvilSlidePos = 0;
                hammerSlidePos = 590;
                lowTurnPos = 0.76;
                lowBonkPos = 0.17;
                lowRotPos = 0.37;
                lowClawPos = 1;
                highTurnPos = 0.16;
                highBonkPos = 1;
                highClawPos = 1;
                snifferPos = 0.36;
                if (pathTimer.getElapsedTime() > 500) {
                    setPathState("TransferFiveHigh");
                }
                break;
            case "TransferFiveHigh":
                anvilSlidePos = 0;
                hammerSlidePos = 590;
                lowTurnPos = 0.76;
                lowBonkPos = 0.17;
                lowRotPos = 0.37;
                lowClawPos = 0;
                highTurnPos = 0.16;
                highBonkPos = 1;
                highClawPos = 1;
                snifferPos = 0.36;
                if (pathTimer.getElapsedTime() > 400) {
                    setPathState("ScoreFive");
                }
                break;
            case "ScoreFive": //transfer to sample place
//                if(!follower.isBusy()) {
                    hammerSlidePos = 2120;
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;
                    snifferPos = 0.36;
                    highClawPos = 1;
                    if (pathTimer.getElapsedTime() > 300) {
                        follower.followPath(scoreFive, true);
                        setPathState("PreDropFive");
                    }
//                }
                break;
            case "PreDropFive":
                if (follower.getPose().roughlyEquals(scorePose,0.4)) {
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.75;
                    highBonkPos = 0.67;
                    snifferPos = 0.36;
                    hammerSlidePos = 2120;
                    highClawPos = 1;
                    if (pathTimer.getElapsedTime()>1400) {
                        setPathState("DropFive");
                    }
                }
                break;

            case "DropFive": //open claw
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.75;
                highBonkPos = 0.67;
                highClawPos = 0;
                snifferPos = 0.36;
                if(pathTimer.getElapsedTime()>200){ setPathState("DropFive2");}

                break;

            case "DropFive2":
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.75;
                highBonkPos = 0.61;
                highClawPos = 0;
                snifferPos = 0.36;
                if(pathTimer.getElapsedTime()>100){ setPathState("DropFive3");}
                break;

            case "DropFive3":
                anvilSlidePos = 0;
                hammerSlidePos = 0;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.2;
                highBonkPos = 0.61;
                highClawPos = 0;
                snifferPos = 0.36;
                if(pathTimer.getElapsedTime()>100){ setPathState("fin:)");}
                break;

            case "Park":
                anvilSlidePos = 0;
                hammerSlidePos = 0;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.75;
                highBonkPos = 0.655;
                highClawPos = 1;
                snifferPos = 0.36;
                if(pathTimer.getElapsedTime()>100) {
                    follower.followPath(park);
                    setPathState("Park2");
                }
                break;

            case "Park2":
                anvilSlidePos = 0;
                hammerSlidePos = 0;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.75;
                highBonkPos = 0.655;
                highClawPos = 1;
                snifferPos = 0.36;
                if(pathTimer.getElapsedTime()>800){ setPathState("fin:)");}
                break;


//            case "PickupSubTwo": //sample place drop to init
//                if (!follower.isBusy()&& pathTimer.getElapsedTime() > 4000) {
//                    follower.followPath(pickUpSubTwo, true);
//                    setPathState(55);
//                }
//
//                break;
//
//            case "ScoreSix":
//                if(!follower.isBusy()) {
//                    follower.followPath(scoreSix, true);
//                    setPathState(56);
//                }
//                break;
//            case "PickupSubThree": //sample place drop to init
//                if (!follower.isBusy()) {
//                    follower.followPath(pickUpSubTwo, true);
//                    setPathState(57);
//                }
//
//                break;
//
//            case "ScoreSeven":
//                if(!follower.isBusy()) {
//                    follower.followPath(scoreSix, true);
//                    setPathState(58);
//                }
//                break;
// TODO************************************************* back to init ******************************




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

        if(RUNARM) {
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
        }
        mode = 1;


        telemetry.addData("path state", pathState);
        telemetry.addData("busy? ", follower.isBusy());
        telemetry.addData("opmodetimer ", opmodeTimer.getElapsedTime());
        telemetry.addData("pathtimer ", pathTimer.getElapsedTime());
        telemetry.addData("parker ", parker);
        telemetry.update();
    }

    @Override
    public void init() { startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseHeading));
        scorePose = new Pose(scorePoseX, scorePoseY, Math.toRadians(scorePoseHeading));
        pickThree = new Pose(pickThreeX, pickThreeY, Math.toRadians(pickThreeHeading));
        pickTwo = new Pose(pickTwoX, pickTwoY, Math.toRadians(pickTwoHeading));
        pickOne = new Pose(pickOneX, pickOneY, Math.toRadians(pickOneHeading));
        pickFromSubOne = new Pose(pickFromSubOneX, pickFromSubOneY, Math.toRadians(pickFromSubOneHeading));
        pickFromSubTwo = new Pose(pickFromSubTwoX, pickFromSubTwoY, Math.toRadians(pickFromSubTwoHeading));
        subToBuckMid = new Pose(subToBuckMidX, subToBuckMidY, Math.toRadians(subToBuckMidHeading));
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

        startPose = new Pose(startPoseX, startPoseY, Math.toRadians(startPoseHeading));
                scorePose = new Pose(scorePoseX, scorePoseY, Math.toRadians(scorePoseHeading));
                pickThree = new Pose(pickThreeX, pickThreeY, Math.toRadians(pickThreeHeading));
                pickTwo = new Pose(pickTwoX, pickTwoY, Math.toRadians(pickTwoHeading));
                pickOne = new Pose(pickOneX, pickOneY, Math.toRadians(pickOneHeading));
                pickFromSubOne = new Pose(pickFromSubOneX, pickFromSubOneY, Math.toRadians(pickFromSubOneHeading));
                pickFromSubTwo = new Pose(pickFromSubTwoX, pickFromSubTwoY, Math.toRadians(pickFromSubTwoHeading));
                subToBuckMid = new Pose(subToBuckMidX, subToBuckMidY, Math.toRadians(subToBuckMidHeading));
        prePark = new Pose(preParkX, preParkY, Math.toRadians(preParkHeading));
        Park = new Pose(ParkX, ParkY, Math.toRadians(ParkHeading));


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
        lowTime = new Timer();

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
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(50);

        limelight.start();
        wasLast = false;
        setPathState("ScoreOne");
    }

    @Override
    public void stop() {}
    public void Lime(){

        switch (lowState) {
            case "limeSearching":
                if (!wasLastL) {
                    mode = 1;
                    limelight.updatePythonInputs(mode, 0, 0, 0, 0, 0, 0, 0);
                    wasLastL = true;

                    lowTurnPos = 0.2;
                    lowBonkPos = 0.5;
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
                        if (anvilSlidePos > -1050) anvilSlidePos -= 8;
                        else anvilSlidePos = 0;
                    }
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
                    lowBonkPos = 0.5;
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
                    lowBonkPos = 0.71;
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
                    lowBonkPos = 0.71;
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
                    lowBonkPos = 0.5;
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
                 if (mode == 1 && (lowHue < 85 && lowHue > 50)) {
                    if (lowTime.getElapsedTime() > 400) {
                        changeStateLow("end");
                        isLime = false;
                        parker = false; //remove this
                    }
                }  else {
                    if (lowTime.getElapsedTime() > 400) {
                        changeStateLow("end");
                        isLime = false;
                        parker = true;
                    }
                }
                break;
        }
        if (opmodeTimer.getElapsedTime() > 27000){
            changeStateLow("end");
            isLime = false;
            parker = true;
        }
        telemetry.addData("limestate: ", lowState);
    }
    public void changeStateLow(String newState){
        lowState = newState;
        wasLastL = false;
        lowTime.resetTimer();
    }
}
