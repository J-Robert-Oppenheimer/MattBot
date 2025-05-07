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

@Autonomous(name = "bucketautoAGAIN")
public class bucketauton3 extends OpMode {
    dumbMap dumbBot = new dumbMap(this);
    public int anvilSlidePos, hammerSlidePos;

    public double lowTurnPos, lowBonkPos, lowRotPos, lowClawPos, highTurnPos, highBonkPos, highClawPos;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(7.75, 103.75, Math.toRadians(0));
    private final Pose scorePose = new Pose(15.5, 133, Math.toRadians(0)); //14 132
    private final Pose pickOne = new Pose(26.5, 120, Math.toRadians(0));
    private final Pose pickTwo = new Pose(26.5, 130.5, Math.toRadians(0));
    private final Pose pickThree = new Pose(29,  133, Math.toRadians(0));
    private final Pose pickSub = new Pose(63, 98, Math.toRadians(270));

    private PathChain scorePreload, pickUpOne, scoreOne, pickUpTwo, scoreTwo, pickUpThree, scoreThree, pickUpFour, scoreFour;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        pickUpOne = follower.pathBuilder()

                .addPath(new BezierLine(new Point(scorePose), new Point(pickOne)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickOne.getHeading())
                .build();

        scoreOne = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickOne), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickOne.getHeading(), scorePose.getHeading())
                .build();

        pickUpTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickTwo)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickTwo.getHeading())
                .build();

        scoreTwo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickTwo), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickTwo.getHeading(), scorePose.getHeading())
                .build();

        pickUpThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickThree)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickThree.getHeading())
                .build();

        scoreThree = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickThree), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickThree.getHeading(), scorePose.getHeading())
                .build();

        pickUpFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickSub)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickSub.getHeading())
                .build();

        scoreFour = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickSub)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickSub.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
            case 1:
                anvilSlidePos = 0;
                hammerSlidePos = 2100;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0;
                highTurnPos = 0.12;
                highBonkPos = 0.18; //spit
                highClawPos = 1;
                if (!follower.isBusy()) {
                    setPathState(100);
                }
            break;
            case 100:
                if (pathTimer.getElapsedTime()>250) {
                    highTurnPos = 0.65;
                    highBonkPos = 0.195;
                    setPathState(456);
                }
                break;
            case 2:

                if (pathTimer.getElapsedTime()>650) {
                    highBonkPos = 0.192;
                    setPathState(456);
                }
                break;

            case 456:
                if(pathTimer.getElapsedTime()>650) {
                    highClawPos = 0;
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTime()>400) {
                    anvilSlidePos = -140;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.69;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    setPathState(4);
                }
                break;

            case 4:
                anvilSlidePos = -140;
                hammerSlidePos = 2050;
                lowTurnPos = 0.48;
                lowBonkPos = 0.69;
                lowRotPos = 0.52;
                lowClawPos = 0;
                highTurnPos = 0.19;
                highBonkPos = 0.18;
                highClawPos = 0;
                if(pathTimer.getElapsedTime()>250) {
                    follower.followPath(pickUpOne, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    hammerSlidePos = 870;
                    lowClawPos = 0;
                    lowBonkPos = 0.69;
                    setPathState(51);
                    }
                break;

            case 51:
                if (pathTimer.getElapsedTime() > 500) {
                    lowBonkPos = 0.75;
                    lowClawPos = 1;
                    setPathState(6);
                }
                case 6:
                    if(pathTimer.getElapsedTime()>600) {
                        anvilSlidePos = -80;
                        hammerSlidePos = 900;
                        lowTurnPos = 0.73;
                        lowBonkPos = 0.22;
                        lowRotPos = 0.35;
                        lowClawPos = 1;
                        highTurnPos = 0.18; //0.08
                        highBonkPos = 0.26;
                        highClawPos = 0;
                        setPathState(7);
                    }
                    break;
                case 7:
                    if(pathTimer.getElapsedTime()>650) {
                        anvilSlidePos = -80;
                        hammerSlidePos = 900;
                        lowTurnPos = 0.73;
                        lowBonkPos = 0.22;
                        lowRotPos = 0.35;
                        lowClawPos = 1;
                        highTurnPos = 0.18; //0.08
                        highBonkPos = 0.26;
                        highClawPos = 1;
                        setPathState(8);
                    }
                    break;
                case 8:
                    if(pathTimer.getElapsedTime()>650) {
                        anvilSlidePos = -80;
                        hammerSlidePos = 900;
                        lowTurnPos = 0.73;
                        lowBonkPos = 0.22;
                        lowRotPos = 0.35;
                        lowClawPos = 0;
                        highTurnPos = 0.18; //0.08
                        highBonkPos = 0.26;
                        highClawPos = 1;
                        setPathState(1000);
                        }
                        break;
            case 1000:
                if(pathTimer.getElapsedTime()>500){
                anvilSlidePos = 0;
                hammerSlidePos = 2050;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0;
                highTurnPos = 0.18;
                highBonkPos = 0.18; //spit
                highClawPos = 1;
                setPathState(9);
                }

                break;

            case 9:
                if(pathTimer.getElapsedTime()>750) {
                    highTurnPos = 0.65;
                    highBonkPos = 0.18;
                    follower.followPath(scoreOne, true);
                    setPathState(10);
                }
                    break;

            case 10:
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
                        setPathState(11);
                }


                break;
            case 11:
                if (pathTimer.getElapsedTime()>400) {
                    highClawPos = 0;
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime()>650) {
                    anvilSlidePos =-140;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.69;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.65;
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    setPathState(13);
                }
                break;


            case 13:
                if(pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -140;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.69;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.19; //CHANGED
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    follower.followPath(pickUpTwo, true);
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    hammerSlidePos = 900;
                    lowClawPos = 0;
                    lowBonkPos = 0.69;
                    setPathState(140);
                }
                break;

            case 140:
                if (pathTimer.getElapsedTime() > 500) {
                    lowBonkPos = 0.75;
                    lowClawPos = 1;
                    setPathState(15);
                }
                break;

            case 15:
                if(pathTimer.getElapsedTime()>600) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 1;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 0;
                    setPathState(16);
                }
                break;
            case 16:
                if(pathTimer.getElapsedTime()>750) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 1;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    setPathState(17);
                }
                break;
            case 17:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 0;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    setPathState(18);
                }
                break;

            case 18:
                if(pathTimer.getElapsedTime()>500){
                    anvilSlidePos = 0;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.18;
                    highBonkPos = 0.18; //spit
                    highClawPos = 1;
                    setPathState(19);
                }

            case 19:
                highBonkPos = 0.19; //spit
                if (pathTimer.getElapsedTime()>800) {
                    highTurnPos = 0.65;
                    follower.followPath(scoreTwo, true);
                    setPathState(20);
                }
                break;
            case 20:
                if(!follower.isBusy()) {
                    setPathState(201);
                }
                break;

            case 201:
                if (pathTimer.getElapsedTime() > 400) {
                    highBonkPos = 0.2;

                    setPathState(211);
                }
                break;

            case 211:
                if(pathTimer.getElapsedTime()>500){
                    highClawPos = 0;
                    setPathState(21);
                }
                break;


            case 21:
                if (pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -100;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.68;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    setPathState(22);
                }
                break;

            case 22:
                if(!follower.isBusy()) {
                    anvilSlidePos = -100;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.68;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    setPathState(232);
                }
                break;

            case 232:
                if (!follower.isBusy()) {
                    anvilSlidePos = -295;
                    lowBonkPos = 0.67;
                    lowRotPos = 0.85;
                    lowTurnPos=0.13;
                    follower.followPath(pickUpThree, true);
                    setPathState(23);
                }



            case 23:
                //if(pathTimer.getElapsedTime()>300) {
                    if (!follower.isBusy()) {
                        lowClawPos = 0;
                        hammerSlidePos = 870;
                        anvilSlidePos = -315;
                        lowBonkPos = 0.67;
                        lowRotPos = 0.85;
                        lowTurnPos = 0.13;
                        setPathState(200);
                    }
                //}
                break;
            case 200:
                if(pathTimer.getElapsedTime()>400) {
                    lowBonkPos = 0.75;
                        lowClawPos = 1;
                        setPathState(24);
                }
                break;
            case 24:
                if(pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.68;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 1;
                    setPathState(512);
                }
                break;
            case 512:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 1;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 0;
                    setPathState(25);
                }
                break;
            case 25:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 1;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    setPathState(26);
                }
                break;
            case 26:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 0;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    setPathState(27);
                }
                break;
            case 27:
                anvilSlidePos = 0;
                hammerSlidePos = 2050;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0;
                highTurnPos = 0.18;
                highBonkPos = 0.18;
                highClawPos = 1;
                if (pathTimer.getElapsedTime()>850) {
                    follower.followPath(scoreThree, true);
                    setPathState(28);
                }
                break;
            case 28:
                if(!follower.isBusy()) {
                    setPathState(281);
                }
                break;

            case 281:
                if (pathTimer.getElapsedTime() > 400) {
                    highBonkPos = 0.2;

                    setPathState(291);
                }
                break;

            case 291:
                if(pathTimer.getElapsedTime()>500){
                    highClawPos = 0;
                    setPathState(29);
                }
                break;


            case 29:
                if (pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -100;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.68;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    setPathState(30);
                }
                break;

            case 30:
                if(!follower.isBusy()) {
                    anvilSlidePos = -100;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.68;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    setPathState(312);
                }
                break;
            case 312:
                if (!follower.isBusy()) {
                    anvilSlidePos = -295;
                    lowBonkPos = 0.67;
                    lowRotPos = 0.85;
                    lowTurnPos=0.13;
                    follower.followPath(pickUpFour, true);
                    setPathState(31);
                }



            case 31:
                //if(pathTimer.getElapsedTime()>300) {
                if (!follower.isBusy()) {
                    lowClawPos = 0;
                    hammerSlidePos = 870;
                    anvilSlidePos = -315;
                    lowBonkPos = 0.67;
                    lowRotPos = 0.85;
                    lowTurnPos = 0.13;
                    setPathState(32);
                }
                //}
                break;
            case 32:
                if(pathTimer.getElapsedTime()>400) {
                    lowBonkPos = 0.75;
                    lowClawPos = 1;
                    setPathState(33);
                }
                break;
            case 33:
                if(pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.68;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 1;
                    setPathState(34);
                }
                break;
            case 34:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 1;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 0;
                    setPathState(35);
                }
                break;
            case 35:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 1;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    setPathState(36);
                }
                break;
            case 36:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 0;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    setPathState(37);
                }
                break;
            case 37:
                anvilSlidePos = 0;
                hammerSlidePos = 2050;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0;
                highTurnPos = 0.18;
                highBonkPos = 0.18;
                highClawPos = 1;
                if (pathTimer.getElapsedTime()>850) {
                    follower.followPath(scoreFour, true);
                    setPathState(38);
                }
                break;
            case 38:
                if(!follower.isBusy()) {
                    setPathState(39);
                }
                break;

            case 39:
                if (pathTimer.getElapsedTime() > 400) {
                    highBonkPos = 0.2;

                    setPathState(40);
                }
                break;

            case 40:
                if(pathTimer.getElapsedTime()>500){
                    highClawPos = 0;
                    setPathState(41);
                }
                break;


            case 41:
                if (pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -100;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.68;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    setPathState(42);
                }
                break;

            case 42:
                if(!follower.isBusy()) {
                    anvilSlidePos = -100;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.68;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    setPathState(43);
                }
                break;
            case 43:
                if (!follower.isBusy()) {
                    anvilSlidePos = -295;
                    lowBonkPos = 0.67;
                    lowRotPos = 0.85;
                    lowTurnPos=0.13;
                    follower.followPath(pickUpFour, true);
                    setPathState(44);
                }



            case 44:
                //if(pathTimer.getElapsedTime()>300) {
                if (!follower.isBusy()) {
                    lowClawPos = 0;
                    hammerSlidePos = 870;
                    anvilSlidePos = -315;
                    lowBonkPos = 0.67;
                    lowRotPos = 0.85;
                    lowTurnPos = 0.13;
                    setPathState(45);
                }
                //}
                break;
            case 45:
                if(pathTimer.getElapsedTime()>400) {
                    lowBonkPos = 0.75;
                    lowClawPos = 1;
                    setPathState(46);
                }
                break;
            case 46:
                if(pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.68;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 1;
                    setPathState(47);
                }
                break;
            case 47:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 1;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 0;
                    setPathState(48);
                }
                break;
            case 48:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 1;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    setPathState(49);
                }
                break;
            case 49:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 0;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    setPathState(50);
                }
                break;
            case 50:
                anvilSlidePos = 0;
                hammerSlidePos = 2050;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0;
                highTurnPos = 0.18;
                highBonkPos = 0.18;
                highClawPos = 1;
                if (pathTimer.getElapsedTime()>850) {
                    follower.followPath(scoreFour, true);
                    setPathState(513);
                }
                break;
            case 513:
                if(!follower.isBusy()) {
                    setPathState(52);
                }
                break;

            case 52:
                if (pathTimer.getElapsedTime() > 400) {
                    highBonkPos = 0.2;

                    setPathState(52);
                }
                break;

            case 53:
                if(pathTimer.getElapsedTime()>500){
                    highClawPos = 0;
                    setPathState(54);
                }
                break;


            case 54:
                if (pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -100;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.68;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    setPathState(55);
                }
                break;

            case 55:
                if(!follower.isBusy()) {
                    anvilSlidePos = -100;
                    hammerSlidePos = 2050;
                    lowTurnPos = 0.48;
                    lowBonkPos = 0.68;
                    lowRotPos = 0.52;
                    lowClawPos = 0;
                    highTurnPos = 0.19;
                    highBonkPos = 0.18;
                    highClawPos = 0;
                    setPathState(56);
                }
                break;
            case 56:
                if (!follower.isBusy()) {
                    anvilSlidePos = -295;
                    lowBonkPos = 0.67;
                    lowRotPos = 0.85;
                    lowTurnPos=0.13;
                    follower.followPath(pickUpFour, true);
                    setPathState(57);
                }



            case 57:
                //if(pathTimer.getElapsedTime()>300) {
                if (!follower.isBusy()) {
                    lowClawPos = 0;
                    hammerSlidePos = 870;
                    anvilSlidePos = -315;
                    lowBonkPos = 0.67;
                    lowRotPos = 0.85;
                    lowTurnPos = 0.13;
                    setPathState(58);
                }
                //}
                break;
            case 58:
                if(pathTimer.getElapsedTime()>400) {
                    lowBonkPos = 0.75;
                    lowClawPos = 1;
                    setPathState(59);
                }
                break;
            case 59:
                if(pathTimer.getElapsedTime()>500) {
                    anvilSlidePos = -120;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.68;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.32;
                    lowClawPos = 1;
                    setPathState(60);
                }
                break;
            case 60:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 1;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 0;
                    setPathState(61);
                }
                break;
            case 61:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 1;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    setPathState(62);
                }
                break;
            case 62:
                if(pathTimer.getElapsedTime()>650) {
                    anvilSlidePos = -80;
                    hammerSlidePos = 900;
                    lowTurnPos = 0.73;
                    lowBonkPos = 0.22;
                    lowRotPos = 0.35;
                    lowClawPos = 0;
                    highTurnPos = 0.18; //0.08
                    highBonkPos = 0.26;
                    highClawPos = 1;
                    setPathState(63);
                }
                break;
            case 63:
                anvilSlidePos = 0;
                hammerSlidePos = 2050;
                lowTurnPos = 0.48;
                lowBonkPos = 0.22;
                lowRotPos = 0.52;
                lowClawPos = 0;
                highTurnPos = 0.18;
                highBonkPos = 0.18;
                highClawPos = 1;
                if (pathTimer.getElapsedTime()>850) {
                    follower.followPath(scoreFour, true);
                    setPathState(64);
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
        dumbBot.highTurnB.setPosition(highTurnPos);
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
        //dumbBot.slidereset2();
        dumbBot.slide3.setPower(0.8);
        dumbBot.slide1.setPower(1);
        dumbBot.slide2.setPower(1);
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