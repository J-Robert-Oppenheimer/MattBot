package org.firstinspires.ftc.teamcode.auton.dumber;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.ArrayList;

@Autonomous(name = "H_Buck")
public class dumbestAuton extends OpMode {
    public Limelight3A limelight;
    LLResult result;
    double[] array;
    public int mode;
    String stage;
    int iterCount;
    boolean wasLast;
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
    public String state;

    ArrayList<Double> LimelightXOffsets = new ArrayList<>();
    ArrayList<Double> LimelightYOffsets = new ArrayList<>();
    ArrayList<Double> LimelightThetas = new ArrayList<>();
    dumbMap dumbBot = new dumbMap(this);
    public int anvilSlidePos, hammerSlidePos;

    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos, snifferPos;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(7.4, 112.3, Math.toRadians(0));
    private final Pose scorePose = new Pose(16.5, 132, Math.toRadians(0));
    private final Pose pickThree = new Pose(30.5, 135.5, Math.toRadians(0));
    private final Pose pickTwo = new Pose(27.8, 131.6, Math.toRadians(0));
    private final Pose pickOne = new Pose(22, 121, Math.toRadians(0));
    private final Pose pickFromSubOne = new Pose(71.7, 96, Math.toRadians(-90));
    private final Pose pickFromSubTwo = new Pose(73.7, 96, Math.toRadians(-90));


    private PathChain scoreOne, pickUpOne, scoreTwo, pickUpTwo, scoreThree, pickUpThree, scoreFour, pickUpSubOne, scoreFive, pickUpSubTwo, scoreSix;

    public void buildPaths() {
        scoreOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        pickUpOne = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickOne)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickOne.getHeading())
                .build();

        scoreTwo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickOne), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickOne.getHeading(), scorePose.getHeading())
                .build();

        pickUpTwo = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickTwo)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickTwo.getHeading())
                .build();

        scoreThree = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickTwo), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickTwo.getHeading(), scorePose.getHeading())
                .build();

        pickUpThree= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickThree)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickThree.getHeading())
                .build();

        scoreFour = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickThree), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickThree.getHeading(), scorePose.getHeading())
                .build();

        pickUpSubOne= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickFromSubOne)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickFromSubOne.getHeading())
                .build();

        scoreFive = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickFromSubOne), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickFromSubOne.getHeading(), scorePose.getHeading())
                .build();

        pickUpSubTwo= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickFromSubTwo)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickFromSubTwo.getHeading())
                .build();

        scoreSix = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickFromSubTwo), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickFromSubTwo.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                anvilSlidePos = 0;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.6;
                highBonkPos = 0.67;
                snifferPos = 0.36;
                hammerSlidePos = 2120;
                highClawPos = 1;
                mode = 1;
                setPathState(1);
                break;

            case 1: //goes to score first sample
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 500) {
                    follower.followPath(scoreOne, true);
                    setPathState(2);
                }
                break;
            case 2: //drops first sample
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 2000) {
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    highTurnPos = 0.6;
                    highBonkPos = 0.67;
                    snifferPos = 0.36;
                    hammerSlidePos = 2120;
                    highClawPos = 0;
                    mode = 1;
                    setPathState(100);
                }
                break;
            case 100: //doesn't work (drop back down) need to test
                follower.followPath(pickUpOne, true);
                if(pathTimer.getElapsedTime() > 250) {
                    anvilSlidePos = 0;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    snifferPos = 0.36;
                    mode = 1;
                    hammerSlidePos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.5;
                    highClawPos = 0;
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime() > 200) {
                    anvilSlidePos = -1000;
                    lowTurnPos = 0.53;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.54;
                    lowClawPos = 0;
                    snifferPos = 0.36;
                    mode = 1;
                    hammerSlidePos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.5;
                    highClawPos = 0;
                    if (pathTimer.getElapsedTime() > 400) {
                        setPathState(-1);
                    }
                }
                break;

            case 5: //lower slides
                anvilSlidePos = -1200;
                hammerSlidePos = 0;
                lowTurnPos = 0.53;
                lowBonkPos = 0.4;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.19;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.36;
                mode = 1;
                if(!follower.isBusy()) {
                    wasLast = false;
                    setPathState(7);
                }
                break;
            case 7: //removed lime
                anvilSlidePos = -1200;
                hammerSlidePos = 0;
                lowTurnPos = 0.53;
                lowBonkPos = 0.72;
                lowRotPos = 0.54;
                lowClawPos = 0;
                highTurnPos = 0.19;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.36;
                mode = 1;
                if(pathTimer.getElapsedTime() > 500) {
                    setPathState(8);
                }
                break;

            case 8:
                anvilSlidePos = -540;
                hammerSlidePos = 0;
                lowTurnPos = 0.53;
                lowBonkPos = 0.72;
                lowRotPos = 0.54;
                lowClawPos = 1;
                highTurnPos = 0.19;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.36;
                mode = 1;
                if(pathTimer.getElapsedTime() > 300) {
                    setPathState(1314);
                }
                break;

            case 1314://transfer
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
                mode = 1;
                if (pathTimer.getElapsedTime() > 300) {
                    setPathState(13414);
                }

                break;
            case 13414: //init to transfer low?
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(314);
                }
                break;
            case 314: //transfer low?
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(3141);
                }
                break;
            case 3141: //transfer both
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(31415);
                }
                break;
            case 31415: //transfer high
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(9);
                }
                break;
            case 9: //transfer to sample place
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.76;
                lowBonkPos = 0.17;
                lowRotPos = 0.37;
                lowClawPos = 0;
                highTurnPos = 0.16;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.36;
                mode = 1;
                follower.followPath(scoreTwo, true);
                setPathState(10);

                break;
//TODO***************************************************** part 2 ********************************************
            case 10: //sampledrop1Position
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.6;
                highBonkPos = 0.7;
                highClawPos = 1;
                snifferPos = 0.92;
                mode = 1;
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(101);
                }
                break;
            case 101: //sampledroptoInit1
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.6;
                highBonkPos = 0.7;
                highClawPos = 1;
                snifferPos = 0.92;
                mode = 1;
                if (!follower.isBusy()) {
                    setPathState(102);
                }
                break;

            case 102: //sampleplacedrop
                if (pathTimer.getElapsedTime() > 400) {
                    anvilSlidePos = 0;
                    hammerSlidePos = 2120;
                    lowTurnPos = 0.2;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.84;
                    lowClawPos = 0;
                    highTurnPos = 0.6;
                    highBonkPos = 0.7;
                    highClawPos = 0;
                    snifferPos = 0.92;
                    mode = 1;
                    setPathState(103);
                }
                break;
            case 103:
                if (pathTimer.getElapsedTime() > 200) {
                    anvilSlidePos = -540;
                    hammerSlidePos = 2120;
                    lowTurnPos = 0.2;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.84;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.6;
                    highClawPos = 0;
                    if (pathTimer.getElapsedTime() > 400) {
                        setPathState(104);
                    }
                }
                break;

            case 104: //sample place drop to init
                anvilSlidePos = -540;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 1;
                highTurnPos = 0.19;
                highBonkPos = 0.6;
                highClawPos = 0;
                snifferPos = 0.92;
                mode = 1;
                if(pathTimer.getElapsedTime()>250) {
                    follower.followPath(pickUpTwo, true);
                    setPathState(105);
                }
                break;
            case 105: //lower slides
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.6;
                highBonkPos = 0.7;
                highClawPos = 1;
                snifferPos = 0.92;
                mode = 1;
                if(!follower.isBusy()) {
                    wasLast = false;
                    setPathState(11);
                }
                break;
            case 11: //lime searching
                if (!wasLast) {
                    limelight.updatePythonInputs(mode, 0, 0, 0, 0, 0, 0, 0);
                    wasLast = true;
                }
                hammerSlidePos = 0;
                lowTurnPos = 0.2;
                lowBonkPos = 0.55;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.2;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.92;
                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    if (array[0] == 1) {
                        setPathState(12);
                    } else {
                        if (anvilSlidePos > -1200) anvilSlidePos -= 4;
                        else anvilSlidePos = 0;
                    }
                }
                break;
            case 12: //limeCheck
                anvilSlidePos = anvilSlidePos;
                hammerSlidePos = 0;
                lowTurnPos = 0.2;
                lowBonkPos = 0.55;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.2;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.92;
                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    if (array[0] == 1) {
                        LimelightYOffsets.add(array[4]);
                        LimelightXOffsets.add(array[3]);
                        LimelightThetas.add(array[1]);
                    } else {
                        setPathState(11);
                    }

                    if (pathTimer.getElapsedTime() > 500) {
                        setPathState(120);
                    }

                }
                break;
            case 120: //lime found
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
                highTurnPos = 0.2;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.36;

                if (pathTimer.getElapsedTime() > 400) {
                    setPathState(121);
                }

                break;
            case 121: //lime prebite
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

                if (pathTimer.getElapsedTime() > 200) {
                    setPathState(122);

                }
                break;
            case 122: //lime bite
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
                if (pathTimer.getElapsedTime() > 200)
                    setPathState(123);

                break;
            case 123: //lime REV check
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


                Color.RGBToHSV(
                        (int) (lowColor.red() * 255.0 / 1023),
                        (int) (lowColor.green() * 255.0 / 1023),
                        (int) (lowColor.blue() * 255.0 / 1023),
                        lowhsv
                );
                lowHue = lowhsv[0];

                if (mode == 1 && (lowHue < 85 && lowHue > 50)) {
                    telemetry.addLine("success");
                    if (pathTimer.getElapsedTime() > 400) {
                        setPathState(124);
                    }
                } else {
                    telemetry.addLine("fail");
                    if (pathTimer.getElapsedTime() > 600) {
                        setPathState(11);
                    }
                }
                break;
            case 124://lime to transfer 2
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
                mode = 1;
                if (pathTimer.getElapsedTime() > 300) {
                    setPathState(125);
                }

                break;
            case 125: //init to transfer low?
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(126);
                }
                break;
            case 126: //transfer low?
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(127);
                }
                break;
            case 127: //transfer both
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(128);
                }
                break;
            case 128: //transfer high
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(13);
                }
                break;
            case 13: //transfer to sample place
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.6;
                highBonkPos = 0.7;
                highClawPos = 1;
                snifferPos = 0.92;
                mode = 1;
                follower.followPath(scoreThree, true);
                setPathState(20);

                break;
//TODO************************************ part 3 ************************************
            case 20: //sampledrop1Position
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.6;
                highBonkPos = 0.7;
                highClawPos = 1;
                snifferPos = 0.92;
                mode = 1;
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(201);
                }
                break;
            case 201: //sampledroptoInit1
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.6;
                highBonkPos = 0.7;
                highClawPos = 1;
                snifferPos = 0.92;
                mode = 1;
                if (!follower.isBusy()) {
                    setPathState(202);
                }
                break;

            case 202: //sampleplacedrop
                if (pathTimer.getElapsedTime() > 400) {
                    anvilSlidePos = 0;
                    hammerSlidePos = 2120;
                    lowTurnPos = 0.2;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.84;
                    lowClawPos = 0;
                    highTurnPos = 0.6;
                    highBonkPos = 0.7;
                    highClawPos = 0;
                    snifferPos = 0.92;
                    mode = 1;
                    setPathState(203);
                }
                break;
            case 203:
                if (pathTimer.getElapsedTime() > 200) {
                    anvilSlidePos = -540;
                    hammerSlidePos = 2120;
                    lowTurnPos = 0.2;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.84;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.6;
                    highClawPos = 0;
                    if (pathTimer.getElapsedTime() > 400) {
                        setPathState(204);
                    }
                }
                break;

            case 204: //sample place drop to init
                anvilSlidePos = -540;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 1;
                highTurnPos = 0.19;
                highBonkPos = 0.6;
                highClawPos = 0;
                snifferPos = 0.92;
                mode = 1;
                if(pathTimer.getElapsedTime()>250) {
                    follower.followPath(pickUpThree, true);
                    setPathState(205);
                }
                break;
            case 205: //sample place
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
                mode = 1;
                if (!follower.isBusy()) {
                    setPathState(21);
                }
            case 21: //lower slides
                anvilSlidePos = -540;
                hammerSlidePos = 0;
                lowTurnPos = 0.2;
                lowBonkPos = 0.55;
                lowRotPos = 0.84;
                lowClawPos = 1;
                highTurnPos = 0.19;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.92;
                mode = 1;
                if(!follower.isBusy()) {
                    wasLast = false;
                    setPathState(22);
                }
                break;
            case 22: //lime searching
                if (!wasLast) {
                    limelight.updatePythonInputs(mode, 0, 0, 0, 0, 0, 0, 0);
                    wasLast = true;
                }
                hammerSlidePos = 0;
                lowTurnPos = 0.2;
                lowBonkPos = 0.55;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.2;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.92;
                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    if (array[0] == 1) {
                        setPathState(23);
                    } else {
                        if (anvilSlidePos > -1200) anvilSlidePos -= 4;
                        else anvilSlidePos = 0;
                    }
                }
                break;
            case 23: //limeCheck
                anvilSlidePos = anvilSlidePos;
                hammerSlidePos = 0;
                lowTurnPos = 0.2;
                lowBonkPos = 0.55;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.2;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.92;
                result = limelight.getLatestResult();
                if (result != null) {
                    array = result.getPythonOutput();
                    if (array[0] == 1) {
                        LimelightYOffsets.add(array[4]);
                        LimelightXOffsets.add(array[3]);
                        LimelightThetas.add(array[1]);
                    } else {
                        setPathState(22);
                    }

                    if (pathTimer.getElapsedTime() > 500) {
                        setPathState(230);
                    }

                }
                break;
            case 231: //lime found
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
                highTurnPos = 0.2;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.36;

                if (pathTimer.getElapsedTime() > 400) {
                    setPathState(232);
                }

                break;
            case 232: //lime prebite
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

                if (pathTimer.getElapsedTime() > 200) {
                    setPathState(233);

                }
                break;
            case 233: //lime bite
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
                if (pathTimer.getElapsedTime() > 200)
                    setPathState(234);

                break;
            case 234: //lime REV check
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


                Color.RGBToHSV(
                        (int) (lowColor.red() * 255.0 / 1023),
                        (int) (lowColor.green() * 255.0 / 1023),
                        (int) (lowColor.blue() * 255.0 / 1023),
                        lowhsv
                );
                lowHue = lowhsv[0];

                if (mode == 1 && (lowHue < 85 && lowHue > 50)) {
                    telemetry.addLine("success");
                    if (pathTimer.getElapsedTime() > 400) {
                        setPathState(235);
                    }
                } else {
                    telemetry.addLine("fail");
                    if (pathTimer.getElapsedTime() > 600) {
                        setPathState(22);
                    }
                }
                break;
            case 235://lime to transfer 2
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
                mode = 1;
                if (pathTimer.getElapsedTime() > 300) {
                    setPathState(236);
                }

                break;
            case 236: //init to transfer low?
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(237);
                }
                break;
            case 237: //transfer low?
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(238);
                }
                break;
            case 238: //transfer both
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(239);
                }
                break;
            case 239: //transfer high
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
                mode = 1;
                if(pathTimer.getElapsedTime()>300) {
                    setPathState(24);
                }
                break;
            case 24: //transfer to sample place
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.76;
                lowBonkPos = 0.17;
                lowRotPos = 0.37;
                lowClawPos = 0;
                highTurnPos = 0.16;
                highBonkPos = 0.6;
                highClawPos = 1;
                snifferPos = 0.36;
                mode = 1;
                follower.followPath(scoreFour, true);
                setPathState(28);

                break;
//TODO***************************************** part 4 **************************************first sub
            case 28: //sampledrop1Position
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.6;
                highBonkPos = 0.7;
                highClawPos = 1;
                snifferPos = 0.92;
                mode = 1;
                if (pathTimer.getElapsedTime() > 250) {
                    setPathState(280);
                }
                break;
            case 280: //sampledroptoInit1
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.6;
                highBonkPos = 0.7;
                highClawPos = 1;
                snifferPos = 0.92;
                mode = 1;
                if (!follower.isBusy()) {
                    setPathState(281);
                }
                break;

            case 281: //sampleplacedrop
                if (pathTimer.getElapsedTime() > 400) {
                    anvilSlidePos = 0;
                    hammerSlidePos = 2120;
                    lowTurnPos = 0.2;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.84;
                    lowClawPos = 0;
                    highTurnPos = 0.6;
                    highBonkPos = 0.7;
                    highClawPos = 0;
                    snifferPos = 0.92;
                    mode = 1;
                    setPathState(282);
                }
                break;
            case 282:
                if (pathTimer.getElapsedTime() > 200) {
                    anvilSlidePos = -540;
                    hammerSlidePos = 2120;
                    lowTurnPos = 0.2;
                    lowBonkPos = 0.4;
                    lowRotPos = 0.84;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.6;
                    highClawPos = 0;
                    if (pathTimer.getElapsedTime() > 400) {
                        setPathState(283);
                    }
                }
                break;

            case 283: //sample place drop to init
                anvilSlidePos = -540;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 1;
                highTurnPos = 0.19;
                highBonkPos = 0.6;
                highClawPos = 0;
                snifferPos = 0.92;
                mode = 1;
                if(pathTimer.getElapsedTime()>250) {
                    follower.followPath(pickUpOne, true);
                    setPathState(284);
                }
                break;
            case 284: //sample place
                anvilSlidePos = 0;
                hammerSlidePos = 2120;
                lowTurnPos = 0.2;
                lowBonkPos = 0.4;
                lowRotPos = 0.84;
                lowClawPos = 0;
                highTurnPos = 0.6;
                highBonkPos = 0.7;
                highClawPos = 1;
                snifferPos = 0.92;
                mode = 1;
                if (!follower.isBusy()) {
                    setPathState(29);
                }


                break;
            case 29: //sample place drop
                if (pathTimer.getElapsedTime()>400) {
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
                    mode = 1;
                    setPathState(33);
                }
                break;

            case 33: //sample place drop to init
                if (!follower.isBusy()) {
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
                    mode = 1;
                    follower.followPath(pickUpSubOne, true);
                    setPathState(34);
                }
                break;
            case 34: //init
                if(!follower.isBusy()) {
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
                    mode = 1;
                    setPathState(340);
                }
                break;

            case 340: //initToLime
                if (pathTimer.getElapsedTime() > 500) {
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
                    mode = 1;
                    setPathState(234);
                }
            case 341: //limeDormant
                if(pathTimer.getElapsedTime()>600) {
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
                    mode = 1;
                    setPathState(342);
                }
                break;
            case 342: //lime searching = case 7, 420, 235
                if(pathTimer.getElapsedTime()>650) {
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
                    result = limelight.getLatestResult();
                    if (result != null) {
                        array = result.getPythonOutput();
                        if (array[0] == 1) {
                            setPathState(343);
                        } else {
                            if (anvilSlidePos > -1200) anvilSlidePos -= 2;
                            else anvilSlidePos = 0;
                        }
                    }
                }
                break;
            case 343: //limeCheck
                if(pathTimer.getElapsedTime()>650) {
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
                    result = limelight.getLatestResult();
                    if (result != null) {
                        array = result.getPythonOutput();
                        if (array[0] == 1) {
                            LimelightYOffsets.add(array[4]);
                            LimelightXOffsets.add(array[3]);
                            LimelightThetas.add(array[1]);
                        } else {
                            setPathState(342);
                        }
                    }

                    if (pathTimer.getElapsedTime() > 200) {
                        setPathState(344);
                    }

                }
                break;
            case 344: //lime found
                if(pathTimer.getElapsedTime()>500){
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


                    if (pathTimer.getElapsedTime() > 600) {
                        setPathState(345);
                    }

                }

                break;
            case 345: //lime prebite
                if(pathTimer.getElapsedTime()>750) {
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


                    if (pathTimer.getElapsedTime() > 200) {
                        setPathState(346);
                    }

                }
                break;
            case 346: //lime bite
                if(pathTimer.getElapsedTime()>750) {
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
                    setPathState(347);
                }
                break;
            case 347: //lime REV check
                if(pathTimer.getElapsedTime()>750) {
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


                    Color.RGBToHSV(
                            (int) (lowColor.red() * 255.0 / 1023),
                            (int) (lowColor.green() * 255.0 / 1023),
                            (int) (lowColor.blue() * 255.0 / 1023),
                            lowhsv
                    );
                    lowHue = lowhsv[0];

                    if (mode == 1 && (lowHue < 85 && lowHue > 50)) {
                        telemetry.addLine("success");
                        if (pathTimer.getElapsedTime() > 600) {
                            setPathState(348);
                        }
                    } else if (mode == 1 && (lowHue < 85 && lowHue > 50)) {
                        telemetry.addLine("success");
                        if (pathTimer.getElapsedTime() > 600) {
                            setPathState(348);
                        }
                    } else if (mode == 2 && (lowHue < 230 && lowHue > 210)) {
                        telemetry.addLine("success");
                        if (pathTimer.getElapsedTime() > 600) {
                            setPathState(348);
                        }
                    } else {
                        telemetry.addLine("fail");
                        if (pathTimer.getElapsedTime() > 600) {
                            setPathState(342);
                        }
                    }

                }
                break;
            case 348: //limeHolding = case 154, 42632, 241
                if(pathTimer.getElapsedTime()>750) {
                    anvilSlidePos = -540;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.54;
                    lowBonkPos = 0.55;
                    lowRotPos = 0.84;
                    lowClawPos = 1;
                    highTurnPos = 0.225;
                    highBonkPos = 0.7;
                    highClawPos = 1;
                    snifferPos = 0.36;
                    setPathState(349);
                }
                break;
            case 349: //lime to transfer 1
                if(pathTimer.getElapsedTime()>750) {
                    anvilSlidePos = -500;
                    hammerSlidePos = 800;
                    lowTurnPos = 0.76;
                    lowBonkPos = 0.55;
                    lowRotPos = 0.37;
                    lowClawPos = 1;
                    highTurnPos = 0.16;
                    highBonkPos = 1;
                    highClawPos = 0;
                    snifferPos = 0.36;
                    mode = 1;
                    setPathState(350);
                }
                break;
            case 350://lime to transfer 2
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    setPathState(351);
                }
                break;
            case 351: //init to transfer low?
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    setPathState(352);
                }
                break;
            case 352: //transfer low?
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    setPathState(353);
                }
                break;
            case 353: //transfer both
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    setPathState(354);
                }
                break;
            case 354: //transfer high
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    setPathState(42);
                }
                break;
            case 42: //transfer to sample place
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    follower.followPath(scoreFive, true);
                    setPathState(43);
                }
                break;
//TODO** part 5 ***********************************************************
            case 43: //sample place
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
                mode = 1;
                if (!follower.isBusy()) {
                    setPathState(44);
                }


                break;
            case 44: //sample place drop
                if (pathTimer.getElapsedTime()>400) {
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
                    mode = 1;
                    setPathState(46);
                }
                break;

            case 46: //sample place drop to init
                if(pathTimer.getElapsedTime()>500) {
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
                    mode = 1;
                    follower.followPath(pickUpSubTwo, true);
                    setPathState(47);
                }
                break;
            case 47: //init
                if(!follower.isBusy()) {
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
                    mode = 1;
                    setPathState(501);
                }
                break;

            case 501: //initToLime
                if (pathTimer.getElapsedTime() > 500) {
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
                    mode = 1;
                    setPathState(502);
                }
            case 502: //limeDormant
                if(pathTimer.getElapsedTime()>600) {
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
                    mode = 1;
                    setPathState(551);
                }
                break;
            case 551: //lime searching = case 7, 420, 235, 342
                if(pathTimer.getElapsedTime()>650) {
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
                    result = limelight.getLatestResult();
                    if (result != null) {
                        array = result.getPythonOutput();
                        if (array[0] == 1) {
                            setPathState(552);
                        } else {
                            if (anvilSlidePos > -1200) anvilSlidePos -= 2;
                            else anvilSlidePos = 0;
                        }
                    }
                }
                break;
            case 552: //limeCheck
                if(pathTimer.getElapsedTime()>650) {
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
                    result = limelight.getLatestResult();
                    if (result != null) {
                        array = result.getPythonOutput();
                        if (array[0] == 1) {
                            LimelightYOffsets.add(array[4]);
                            LimelightXOffsets.add(array[3]);
                            LimelightThetas.add(array[1]);
                        } else {
                            setPathState(551);
                        }
                    }

                    if (pathTimer.getElapsedTime() > 200) {
                        setPathState(553);
                    }

                }
                break;
            case 553: //lime found
                if(pathTimer.getElapsedTime()>500){
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


                    if (pathTimer.getElapsedTime() > 600) {
                        setPathState(554);
                    }

                }

                break;
            case 554: //lime prebite
                if(pathTimer.getElapsedTime()>750) {
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


                    if (pathTimer.getElapsedTime() > 200) {
                        setPathState(555);
                    }

                }
                break;
            case 555: //lime bite
                if(pathTimer.getElapsedTime()>750) {
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
                    setPathState(556);
                }
                break;
            case 556: //lime REV check
                if(pathTimer.getElapsedTime()>750) {
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


                    Color.RGBToHSV(
                            (int) (lowColor.red() * 255.0 / 1023),
                            (int) (lowColor.green() * 255.0 / 1023),
                            (int) (lowColor.blue() * 255.0 / 1023),
                            lowhsv
                    );
                    lowHue = lowhsv[0];

                    if (mode == 1 && (lowHue < 85 && lowHue > 50)) {
                        telemetry.addLine("success");
                        if (pathTimer.getElapsedTime() > 600) {
                            setPathState(557);
                        }
                    } else if (mode == 1 && (lowHue < 85 && lowHue > 50)) {
                        telemetry.addLine("success");
                        if (pathTimer.getElapsedTime() > 600) {
                            setPathState(557);
                        }
                    } else if (mode == 2 && (lowHue < 230 && lowHue > 210)) {
                        telemetry.addLine("success");
                        if (pathTimer.getElapsedTime() > 600) {
                            setPathState(557);
                        }
                    } else {
                        telemetry.addLine("fail");
                        if (pathTimer.getElapsedTime() > 600) {
                            setPathState(551);
                        }
                    }

                }
                break;
            case 557: //limeHolding = case 154, 42632, 241, 348
                if(pathTimer.getElapsedTime()>750) {
                    anvilSlidePos = -540;
                    hammerSlidePos = 0;
                    lowTurnPos = 0.54;
                    lowBonkPos = 0.55;
                    lowRotPos = 0.84;
                    lowClawPos = 1;
                    highTurnPos = 0.225;
                    highBonkPos = 0.7;
                    highClawPos = 1;
                    snifferPos = 0.36;
                    setPathState(558);
                }
                break;
            case 558: //lime to transfer 1
                if(pathTimer.getElapsedTime()>750) {
                    anvilSlidePos = -500;
                    hammerSlidePos = 800;
                    lowTurnPos = 0.76;
                    lowBonkPos = 0.55;
                    lowRotPos = 0.37;
                    lowClawPos = 1;
                    highTurnPos = 0.16;
                    highBonkPos = 1;
                    highClawPos = 0;
                    snifferPos = 0.36;
                    mode = 1;
                    setPathState(559);
                }
                break;
            case 559://lime to transfer 2
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    setPathState(561);
                }
                break;
            case 561: //init to transfer low?
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    setPathState(562);
                }
                break;
            case 562: //transfer low?
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    setPathState(563);
                }
                break;
            case 563: //transfer both
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    setPathState(564);
                }
                break;
            case 564: //transfer high
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    setPathState(55);
                }
                break;
            case 55:
                if(pathTimer.getElapsedTime()>750) {
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
                    mode = 1;
                    follower.followPath(scoreFive, true);
                    setPathState(56);
                }
                break;
// TODO************************************************* back to init ******************************
            case 56:
                anvilSlidePos = 0;
                hammerSlidePos = 2050;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0;
                highTurnPos = 0.65;
                highBonkPos = 0.18; //spit
                highClawPos = 1;
                if (!follower.isBusy()) {
                    highBonkPos = 0.2;
                    setPathState(57);
                }


                break;
            case 57: //sample place
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
                mode = 1;
                if (!follower.isBusy()) {
                    setPathState(58);
                }


                break;
            case 58: //sample place drop
                if (pathTimer.getElapsedTime()>400) {
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
                    mode = 1;
                    setPathState(59);
                }
                break;

            case 59: //sample place drop to init
                if(pathTimer.getElapsedTime()>500) {
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
                    mode = 1;
                    //follower.followPath(pickUpSubTwo, true); //maybe  use if want to go back to sub?
                    setPathState(60);
                }
                break;
            case 60: //init
                if(!follower.isBusy()) {
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
                    mode = 1;
                    setPathState(61);
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
        dumbBot.highTurnB.setPosition(highTurnPos);
        dumbBot.highTurnT.setPosition(highTurnPos);
        dumbBot.highBonkR.setPosition(highBonkPos);
        dumbBot.highBonkL.setPosition(highBonkPos);
        dumbBot.highClaw.setPosition(highClawPos);
        dumbBot.sniffer.setPosition((snifferPos));
        mode = 1;

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
        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(50);

        limelight.start();
        wasLast = false;
        stage = "do nothing right now pleaseeeee workkkkk for the love of god";
        setPathState(0);
    }

    @Override
    public void stop() {}
}