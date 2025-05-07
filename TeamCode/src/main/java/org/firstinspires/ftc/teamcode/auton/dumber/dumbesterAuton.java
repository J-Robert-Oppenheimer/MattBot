package org.firstinspires.ftc.teamcode.auton.dumber;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
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

@Autonomous(name = "H_Buck_paths")
public class dumbesterAuton extends OpMode {
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
    private final Pose pickFromSubTwo = new Pose(75.7, 96, Math.toRadians(-90));


    private PathChain scoreOne, pickUpOne, scoreTwo, pickUpTwo, scoreThree, pickUpThree, scoreFour, pickUpSubOne, scoreFive, pickUpSubTwo, scoreSix;
//    private Path scoreOne, pickUpOne, scoreTwo, pickUpTwo, scoreThree, pickUpThree, scoreFour, pickUpSubOne, scoreFive, pickUpSubTwo, scoreSix;

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

        scoreFive = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickFromSubOne), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickFromSubOne.getHeading(), scorePose.getHeading())
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
            case 0: //goes to score first sample
                if(!follower.isBusy()) {
                    follower.followPath(scoreOne, true);
                    setPathState(4);
                }


            case 4: //sample place drop to init

                if(!follower.isBusy() && pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(pickUpOne, true);
                    setPathState(9);
                }
                break;

            case 9: //transfer to sample place
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(scoreTwo, true);
                    setPathState(104);
                }

                break;
//TODO***************************************************** part 2 ********************************************

            case 104: //sample place drop to init
                mode = 1;
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(pickUpTwo, true);
                    setPathState(13);
                }
                break;

            case 13: //transfer to sample place
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 4000) {
                follower.followPath(scoreThree, true);
                setPathState(204);}

                break;
//TODO************************************ part 3 ************************************

            case 204: //sample place drop to init

                if(!follower.isBusy() && pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(pickUpThree, true);
                    setPathState(24);
                }
                break;

            case 24: //transfer to sample place
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 4000){
                follower.followPath(scoreFour, true);
                setPathState(33);}

                break;
//TODO***************************************** part 4 **************************************first sub


            case 33: //sample place drop to init
                if (!follower.isBusy()&& pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(pickUpSubOne, true);
                    setPathState(42);
                }
                break;

            case 42: //transfer to sample place
                    if (follower.isBusy()&& pathTimer.getElapsedTime() > 4000) {
                        follower.followPath(scoreFive, true);
                        setPathState(46);
                    }

                break;


            case 46: //sample place drop to init
                if (!follower.isBusy()&& pathTimer.getElapsedTime() > 4000) {
                    follower.followPath(pickUpSubTwo, true);
                    setPathState(55);
                }

                break;

            case 55:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSix, true);
                    setPathState(56);
                }
                break;
            case 56: //sample place drop to init
                if (!follower.isBusy()) {
                    follower.followPath(pickUpSubTwo, true);
                    setPathState(57);
                }

                break;

            case 57:
                if(!follower.isBusy()) {
                    follower.followPath(scoreSix, true);
                    setPathState(58);
                }
                break;
// TODO************************************************* back to init ******************************




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

//        dumbBot.slide3.setTargetPosition(anvilSlidePos);
//        dumbBot.slide1.setTargetPosition(hammerSlidePos);
//        dumbBot.slide2.setTargetPosition(hammerSlidePos);
//        dumbBot.lowTurn.setPosition(lowTurnPos);
//        dumbBot.lowBonk.setPosition(lowBonkPos);
//        dumbBot.lowRot.setPosition(lowRotPos);
//        dumbBot.lowClaw.setPosition(lowClawPos);
//        dumbBot.highTurnB.setPosition(highTurnPos);
//        dumbBot.highTurnT.setPosition(highTurnPos);
//        dumbBot.highBonkR.setPosition(highBonkPos);
//        dumbBot.highBonkL.setPosition(highBonkPos);
//        dumbBot.highClaw.setPosition(highClawPos);
//        dumbBot.sniffer.setPosition((snifferPos));
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