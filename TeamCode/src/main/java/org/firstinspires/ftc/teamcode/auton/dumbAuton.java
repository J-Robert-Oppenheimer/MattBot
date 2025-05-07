package org.firstinspires.ftc.teamcode.auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@Autonomous(name = "dumbauto")
public class dumbAuton extends OpMode{

    /**
     * This is an example auto that showcases movement and control of two servos autonomously.
     * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
     * There are examples of different ways to build paths.
     * A path progression method has been created and can advance based on time, position, or other factors.
     *
     * @author Baron Henderson - 20077 The Indubitables
     * @version 2.0, 11/28/2024
     */


    dumbMap dumbBot = new dumbMap(this);


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(7.75, 103.75, Math.toRadians(0));

    private final Pose startPose2 = new Pose(7.75, 60, Math.toRadians(0));
    private final Pose lineUpOne = new Pose(48, 110, Math.toRadians(0));
    private final Pose pushOneprep = new Pose(64, 120, Math.toRadians(0));
    private final Pose pushOne = new Pose(15, 124, Math.toRadians(0));
    private final Pose lineUp2 = new Pose(64, 120, Math.toRadians(0));
    private final Pose push2prep = new Pose(64, 128, Math.toRadians(0));
    private final Pose push2 = new Pose(15, 128, Math.toRadians(0));
    private final Pose lineUp3 = new Pose(64, 128, Math.toRadians(0));
    private final Pose push3prep = new Pose(64, 136, Math.toRadians(0));
    private final Pose push3 = new Pose(15, 136, Math.toRadians(0));
    private final Pose park = new Pose(9, 10, Math.toRadians(0));


    private PathChain thing;
    private PathChain park_thing;





    public void buildPaths() {
        thing = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(lineUpOne)))
                .setLinearHeadingInterpolation(startPose.getHeading(), lineUpOne.getHeading())
                .addPath(new BezierLine(new Point(lineUpOne), new Point(pushOneprep)))
                .setLinearHeadingInterpolation(lineUpOne.getHeading(), pushOneprep.getHeading())
                .addPath(new BezierLine(new Point(pushOneprep), new Point(pushOne)))
                .setLinearHeadingInterpolation(pushOneprep.getHeading(), pushOneprep.getHeading())
                .addPath(new BezierLine(new Point(pushOne), new Point(lineUp2)))
                .setLinearHeadingInterpolation(pushOne.getHeading(), lineUp2.getHeading())
                .addPath(new BezierLine(new Point(lineUp2), new Point(push2prep)))
                .setLinearHeadingInterpolation(lineUp2.getHeading(), push2prep.getHeading())
                .addPath(new BezierLine(new Point(push2prep), new Point(push2)))
                .setLinearHeadingInterpolation(push2prep.getHeading(), push2.getHeading())
                .addPath(new BezierLine(new Point(push2), new Point(lineUp3)))
                .setLinearHeadingInterpolation(push2.getHeading(), lineUp3.getHeading())
                .addPath(new BezierLine(new Point(lineUp3), new Point(push3prep)))
                .setLinearHeadingInterpolation(lineUp3.getHeading(), push3prep.getHeading())
                .addPath(new BezierLine(new Point(push3prep), new Point(push3)))
                .setLinearHeadingInterpolation(push3prep.getHeading(), push3.getHeading())
                .build();
        park_thing = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose2), new Point(park)))
                 .setLinearHeadingInterpolation(startPose2.getHeading(), park.getHeading())
                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(thing, true);
                setPathState(1);
                break;
            case 1:

                if (!follower.isBusy()) {
                    setPathState(2);
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
    public void stop() {}}
