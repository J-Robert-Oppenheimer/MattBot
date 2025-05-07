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


@Autonomous(name = "specautopush")
public class specAutoPush extends OpMode {
    dumbMap dumbBot = new dumbMap(this);
    public int anvilSlidePos, hammerSlidePos;
    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos, snifferPos;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
//    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos;

    public Servo lowClaw, lowRot, lowBonk, lowTurn, highClaw, highTurn, highBonkL, highBonkR;
    public DcMotor slide1, slide2, slide3, sniffer, h;


    // Robot positions
    private final Pose startPose = new Pose(7.25, 63, Math.toRadians(0));
    private final Pose specimen1 = new Pose(41.5, 70.5, Math.toRadians(0));
    private final Pose push1 = new Pose(52.5, 34, Math.toRadians(0));
    private final Pose push1Control = new Pose(23, 32, Math.toRadians(0));
    private final Pose push2 = new Pose(19.6, 22.5, Math.toRadians(0));
    private final Pose push2Control = new Pose(76.2, 21.5, Math.toRadians(0));
    private final Pose push3 = new Pose(57.5, 22.7, Math.toRadians(0));
    private final Pose push4 = new Pose(19.2, 14.5, Math.toRadians(0));
    private final Pose push4Control = new Pose(68.8, 14.5, Math.toRadians(0));
    private final Pose push5 = new Pose(57, 13, Math.toRadians(0));
    private final Pose push6 = new Pose(18.8, 8.8, Math.toRadians(0));
    private final Pose push6Control = new Pose(77.8, 5.6, Math.toRadians(0));



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



    private PathChain score1, push, speciminGrab1, score2, speciminGrab2, score3, speciminGrab3, score4, speciminGrab4, score5, toPark;

    public void buildPaths() {

        score1 = follower.pathBuilder().addPath(
                new BezierLine(
                        new Point(startPose),
                        new Point(specimen1)
                )).setConstantHeadingInterpolation(0).build();
        push = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specimen1), new Point(push1Control), new Point(push1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(push1), new Point(push2Control), new Point(push2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath (new BezierLine(new Point(push2), new Point(push3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(push3),new Point(push4Control), new Point(push4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(push4), new Point(push5)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(new Point(push5), new Point(push6Control), new Point(push6)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        speciminGrab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(push6), new Point(wallPickPrep)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(wallPickPrep), new Point(wallPick)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score2 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(wallPick),
                        new Point(specimen2Control),
                        new Point(specimen2)
                )).setConstantHeadingInterpolation(0).build();

        speciminGrab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen2), new Point(wallPickPrep)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(wallPickPrep), new Point(wallPick)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score3 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(wallPick),
                        new Point(specimen3Control),
                        new Point(specimen3)
                )).setConstantHeadingInterpolation(0).build();

        speciminGrab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen3), new Point(wallPickPrep)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(wallPickPrep), new Point(wallPick)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score4 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(wallPick),
                        new Point(specimen4Control),
                        new Point(specimen4)
                )).setConstantHeadingInterpolation(0).build();

        speciminGrab4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(specimen4), new Point(wallPickPrep)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(wallPickPrep), new Point(wallPick)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score5 = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Point(wallPick),
                        new Point(specimen5Control),
                        new Point(specimen5)
                )).setConstantHeadingInterpolation(0).build();


//        toPark = follower.pathBuilder().addPath(
//                new BezierCurve(
//                        new Point(bar5),
//                        new Point(preBar),
//                        new Point(park)
//                )).setConstantHeadingInterpolation(0).build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case (1):
                // Set positions for scoreing bottem slides shouldnt move the entire time, also start to move to score one
                hammerSlidePos = 800;
                highTurnPos = 0.75;
                highBonkPos = 0.33;
                highClawPos = 1;
                anvilSlidePos = 0;
                lowTurnPos = 0.76;
                lowBonkPos = 0.17;
                lowRotPos = 0.33;
                lowClawPos = 1;
                snifferPos = 0.36;
                follower.followPath(score1);
                setPathState(2);
                break;
            case (2):
                if (!follower.isBusy()) {
                    //contiuing to follow poses for scoreing in bellyop
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    setPathState(3);
                }
                break;
            case (3):
                if(pathTimer.getElapsedTime() > 250){
                    //continguing to follow poses for scoreing in bellyop mainly lets go of the specimin
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 0;
                    setPathState(4);
                }
                break;
            case (4):
                if (pathTimer.getElapsedTime() > 100) {
                    //move the claw out of the way. we may need to move the bot back first but this shouldnt hit anything
                    hammerSlidePos = 660;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 0;
                    setPathState(5);
                }
                break;
            case (5):
                if(pathTimer.getElapsedTime() > 100){
                    //start to push the 3 blocks, gets top arm ready to pick up off of the wall
                    hammerSlidePos = 640;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 0;
                    anvilSlidePos = 0;
                    lowTurnPos = 0.76;
                    lowBonkPos = 0.17;
                    lowRotPos = 0.33;
                    lowClawPos = 1;
                    snifferPos = 0.36;
                    follower.followPath(push);
                    setPathState(28);
                }







            case (29):
                //when pushing is done get into position to pick up off the wall
                if (!follower.isBusy()){
                    follower.followPath(speciminGrab1);
                    setPathState(30);
                }
                break;
            case (30):
                //close the claw
                if (!follower.isBusy()) {
                    hammerSlidePos = 660;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 1;
                    setPathState(31);
                }
                break;
            case (31):
                follower.followPath(score2);
                //move to the score pose and bring the arm around
                if(pathTimer.getElapsedTime() > 150) {
                    hammerSlidePos = 800;
                    highTurnPos = 0.75;
                    highBonkPos = 0.33;
                    highClawPos = 1;
                    setPathState(32);
                }
                break;
            case (32):
                //continue to move through the Jelly op poses to score
                if (!follower.isBusy()) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    setPathState(33);
                }
                break;
            case (33):
                //continue to move through the Jelly op poses. this part lets go
                if (pathTimer.getElapsedTime() > 50) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 0;
                    setPathState(34);
                }
                break;
            case (34):
                //beings to move to the score pose
                if (pathTimer.getElapsedTime() > 100) {
                    follower.followPath(speciminGrab2);
                    setPathState(35);
                }
                break;
            case (35):
                if (pathTimer.getElapsedTime() > 50) {
                    //brings the arm around to pick off the wall. this may need to happen before the bot beings to move depending on how fast the bot is and how fast the servos are
                    hammerSlidePos = 660;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 0;
                    setPathState(36);
                }
            case (36):
                //picks up off the wall and begins the cycle again
                if (!follower.isBusy()) {
                    hammerSlidePos = 660;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 1;
                    setPathState(37);
                }
            case (37):
                hammerSlidePos = 800;
                highTurnPos = 0.75;
                highBonkPos = 0.33;
                highClawPos = 1;
                follower.followPath(score3);
                setPathState(38);
                break;
            case (38):
                if (!follower.isBusy()) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    setPathState(39);
                }
                break;
            case (39):
                if (pathTimer.getElapsedTime() > 50) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 0;
                    setPathState(40);
                }
                break;
            case (41):
                if (pathTimer.getElapsedTime() > 100) {
                    follower.followPath(speciminGrab3);
                    setPathState(42);
                }
                break;
            case (42):
                if (pathTimer.getElapsedTime() > 50) {
                    hammerSlidePos = 660;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 0;
                    setPathState(36);
                }
            case (43):
                if (!follower.isBusy()) {
                    hammerSlidePos = 660;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 1;
                    setPathState(44);
                }
            case (44):
                hammerSlidePos = 800;
                highTurnPos = 0.75;
                highBonkPos = 0.33;
                highClawPos = 1;
                follower.followPath(score3);
                setPathState(45);
                break;
            case (45):
                if (!follower.isBusy()) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    setPathState(46);
                }
                break;
            case (46):
                if (pathTimer.getElapsedTime() > 50) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 0;
                    setPathState(47);
                }
                break;
            case (47):
                if (pathTimer.getElapsedTime() > 100) {
                    follower.followPath(speciminGrab3);
                    setPathState(48);
                }
                break;
            case (48):
                if (pathTimer.getElapsedTime() > 50) {
                    hammerSlidePos = 660;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 0;
                    setPathState(49);
                }
            case (49):
                if (!follower.isBusy()) {
                    hammerSlidePos = 660;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 1;
                    setPathState(50);
                }
            case (50):
                hammerSlidePos = 800;
                highTurnPos = 0.75;
                highBonkPos = 0.33;
                highClawPos = 1;
                follower.followPath(score4);
                setPathState(51);
                break;
            case (51):
                if (!follower.isBusy()) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 1;
                    setPathState(52);
                }
                break;
            case (52):
                if (pathTimer.getElapsedTime() > 50) {
                    hammerSlidePos = 700;
                    highTurnPos = 0.75;
                    highBonkPos = 0.28;
                    highClawPos = 0;
                    setPathState(53);
                }
                break;
            case (53):
                if (pathTimer.getElapsedTime() > 100) {
                    follower.followPath(speciminGrab3);
                    setPathState(54);
                }
                break;
            case (54):
                if (pathTimer.getElapsedTime() > 50) {
                    hammerSlidePos = 660;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 0;
                    setPathState(55);
                }
            case (55):
                if (!follower.isBusy()) {
                    hammerSlidePos = 660;
                    highTurnPos = 0.2;
                    highBonkPos = 0.15;
                    highClawPos = 1;
                    setPathState(56);
                }
            case (56):
                hammerSlidePos = 800;
                highTurnPos = 0.75;
                highBonkPos = 0.33;
                highClawPos = 1;
                follower.followPath(score5);
                setPathState(57);
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
        dumbBot.sniffer.setPosition(snifferPos);



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

        dumbBot.slide3.setTargetPosition(0);
        dumbBot.slide1.setTargetPosition(0);
        dumbBot.slide2.setTargetPosition(0);
        dumbBot.lowTurn.setPosition(0.76);
        dumbBot.lowBonk.setPosition(0.17);
        dumbBot.lowRot.setPosition(0.33);
        dumbBot.lowClaw.setPosition(1);
        dumbBot.highTurnT.setPosition(0.225);
        dumbBot.highTurnB.setPosition(0.225);
        dumbBot.highBonkL.setPosition(0.9);
        dumbBot.highBonkR.setPosition(0.9);
        dumbBot.highClaw.setPosition(1);
        dumbBot.sniffer.setPosition(0.36);


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