package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.dumbMap;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ATeleOp", group = "oldteles")
@Config
public class TeleOp extends LinearOpMode {
    dumbMap dumbBot = new dumbMap(this);
    public int upslidePos = 0, lowslidePos = 0, scrollPoses = 3;
    public boolean rbumpLast = false, lbumpLast = false, first = true;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    //boolean searching, found = false, foundLast = false, vertical = false, verticalLast = false;


    @Override
    public void runOpMode() throws InterruptedException {
        dumbBot.init2();
        dumbBot.slidereset2();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

//        HuskyLens.Block[] list;
//        int iters = 0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        follower.startTeleopDrive();

        while(opModeIsActive()) {
//            iters++;
            //movement
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * dumbBot.drivePower,
                    -gamepad1.left_stick_x * dumbBot.drivePower,
                    -gamepad1.right_stick_x * Math.abs(dumbBot.drivePower), true);
            follower.update();

            if (!dumbBot.xLast && gamepad1.x) {
                dumbBot.halfSpeedToggle = !dumbBot.halfSpeedToggle;
                dumbBot.qtrSpeedToggle = false;
            }
            if (!dumbBot.bLast && gamepad1.b) {
                dumbBot.qtrSpeedToggle = !dumbBot.qtrSpeedToggle;
                dumbBot.halfSpeedToggle = false;
            }
            if (!dumbBot.yLast && gamepad1.y) {
                dumbBot.drivingReverse = !dumbBot.drivingReverse;
            }
            if (dumbBot.drivingReverse) {
                if (dumbBot.halfSpeedToggle) {
                    dumbBot.drivePower = -0.5;
                } else if (dumbBot.qtrSpeedToggle) {
                    dumbBot.drivePower = -0.25;
                } else {
                    dumbBot.drivePower = -1;
                }
            } else {
                if (dumbBot.halfSpeedToggle) {
                    dumbBot.drivePower = 0.5;
                } else if (dumbBot.qtrSpeedToggle) {
                    dumbBot.drivePower = 0.25;
                } else {
                    dumbBot.drivePower = 1;
                }
            }
            dumbBot.xLast = gamepad1.x;
            dumbBot.yLast = gamepad1.y;
            dumbBot.bLast = gamepad1.b;
            telemetry.addData("halfspeed: ", dumbBot.halfSpeedToggle);
            telemetry.addData("quarterspeed: ", dumbBot.qtrSpeedToggle);
            telemetry.addData("reverse: ", dumbBot.drivingReverse);

            //servos
 /*           list = dumbBot.huskyLens.blocks(dumbBot.currentColor);


            found = list.length > 0;
            if (found) {
                searching = false;
                vertical = list[0].height > list[0].width;
            }
            if (found && (!foundLast || vertical != verticalLast) && !dumbBot.clawOpen && dumbBot.slideangle.getCurrentPosition() > 50) {
                if (!vertical && dumbBot.cSpin.getPosition() > 0.99) {
                    dumbBot.clawRot = true;
                } else if (!vertical && dumbBot.cSpin.getPosition() < 0.01) {
                    dumbBot.clawRot = false;

                }
            }
            foundLast = found;
            verticalLast = vertical;

  */


            if(!dumbBot.aLast2 && gamepad2.a){
                dumbBot.clawOpen = !dumbBot.clawOpen;
                dumbBot.highClaw.setPosition(dumbBot.clawOpen? 0.37:0.61);
            }

            if (!gamepad2.a && dumbBot.aLast2){
                dumbBot.lowClaw.setPosition(dumbBot.clawOpen? 1:0);
            }

            dumbBot.aLast2 = gamepad2.a;

            if (gamepad2.left_bumper && !dumbBot.lbumpLast){
                //clawRot = !clawRot;
                if (Math.abs(dumbBot.lowRot.getPosition() - 0.53) < 0.02) {
                    dumbBot.lowRot.setPosition(0.21);
                }
//                else if (Math.abs(dumbBot.lowRot.getPosition() - 0.21) < 0.02) {
//                    dumbBot.lowRot.setPosition(0.53);
//                }
                else{
                    dumbBot.lowRot.setPosition(0.53);
                }
            }
            //lowRot.setPosition(clawRot? 0.53:0.21);
            //dumbBot.rstickpressLast = gamepad2.right_stick_button;
            dumbBot.lbumpLast = gamepad2.left_bumper;



            if(Math.abs(gamepad2.right_stick_y)>0.5 && Math.abs(dumbBot.rstickyLast)<0.5){
                dumbBot.bonk = !dumbBot.bonk;
                dumbBot.lowBonk.setPosition(dumbBot.bonk? 0.76:0.22);
            }

            dumbBot.rstickyLast = gamepad2.right_stick_y;

            if(Math.abs(gamepad2.right_stick_x)>0.5){
                dumbBot.lTpos+=gamepad2.right_stick_x*0.008;
                dumbBot.lTpos = Math.max(Math.min(dumbBot.lTpos,0.83),0.13);
                dumbBot.lowRot.setPosition(dumbBot.lTpos);
                dumbBot.lowTurn.setPosition(dumbBot.lTpos);
            }


            dumbBot.rstickxLast = gamepad2.right_stick_x;


            if (gamepad2.x){ //grabb
                dumbBot.lTpos = 0.46;
                dumbBot.lowTurn.setPosition(dumbBot.lTpos);
                dumbBot.lowRot.setPosition(0.46);
                dumbBot.bonk = false;
                dumbBot.lowBonk.setPosition(0.69);
                dumbBot.clawOpen = true;
                dumbBot.lowClaw.setPosition(0.23);
            }

            if (gamepad2.b){ //off right
                dumbBot.lTpos = 0.83;
                dumbBot.lowBonk.setPosition(0.58);
                dumbBot.lowRot.setPosition(0.53);
            }

/*
        if(gamepad2.dpad_up) {
            dumbBot.currentColor = 1;
            telemetry.addLine("current color: YELLOW");
        }
        if(gamepad2.dpad_left) {
            dumbBot.currentColor = 2;
            telemetry.addLine("current color: RED");
        }
        if(gamepad2.dpad_right) {
            dumbBot.currentColor = 3;
            telemetry.addLine("current color: BLUE");
        }

 */

            //arms

            /*f(gamepad2.right_trigger>0){
                upslidePos+= (int)(gamepad2.right_trigger*100);
                upslidePos = Math.min(upslidePos,2050);
            }
            else if(gamepad2.left_trigger>0){
                upslidePos-= (int)(gamepad2.left_trigger*100);
                upslidePos = Math.max(upslidePos,0);
            }

            dumbBot.slide1.setTargetPosition(upslidePos);
            dumbBot.slide2.setTargetPosition(upslidePos);
            dumbBot.slide1.setPower(1);
            dumbBot.slide2.setPower(1);
            */

            if(gamepad2.left_stick_y>0){
                lowslidePos+= (int)(gamepad2.left_stick_y*5);
                lowslidePos = Math.min(0,lowslidePos);
            }
            else if(gamepad2.left_stick_y<0){
                lowslidePos+= (int)(gamepad2.left_stick_y*5);
                lowslidePos = Math.max(lowslidePos,-400);
            }
            dumbBot.slide3.setTargetPosition(lowslidePos);
            dumbBot.slide3.setPower(0.5);
//            telemetry.addData("ticks passed: ", iters);


/*
            if (gamepad2.left_bumper && !lbumpLast){
                scrollPoses--;
                scrollPoses = Math.max(scrollPoses,1);
            }
            else if (gamepad2.right_bumper && !rbumpLast){
                scrollPoses++;
                scrollPoses = Math.min(scrollPoses,5);
            }

            if (((gamepad2.right_bumper && !rbumpLast) || (gamepad2.left_bumper && !lbumpLast)) || first) {
                if (scrollPoses == 3) {
                    upslidePos = 280;
                    dumbBot.highTurn.setPosition(0.79); //specgrab
                    dumbBot.highBonkL.setPosition(0.15);
                    dumbBot.highBonkR.setPosition(0.15);

                } else if (scrollPoses == 2) { //transfer
                    upslidePos = 650;
                    lowslidePos = -100;
                    dumbBot.lowBonk.setPosition(0.22);
                    dumbBot.lTpos = 0.66;
                    dumbBot.lowTurn.setPosition(0.65);
                    dumbBot.lowRot.setPosition(0.35);
                    dumbBot.highBonkL.setPosition(0.005);
                    dumbBot.highBonkR.setPosition(0.005);
                    dumbBot.highTurn.setPosition(0.81);

                } else if (scrollPoses == 1) {
                    upslidePos = 2050;
                    dumbBot.highTurn.setPosition(0.24); //blockplace
                    dumbBot.highBonkL.setPosition(0.07);
                    dumbBot.highBonkR.setPosition(0.07);
                } else if (scrollPoses == 4) {
                    upslidePos = 50;
                    dumbBot.highBonkL.setPosition(0.105); //specplace
                    dumbBot.highBonkR.setPosition(0.105);
                    dumbBot.lowBonk.setPosition(0.7);
                }
                else if (scrollPoses == 5) {
                    upslidePos = 50;
                    dumbBot.highTurn.setPosition(0.13); //specplace
                    dumbBot.highBonkL.setPosition(0.105);
                    dumbBot.highBonkR.setPosition(0.105);
                }
            }

            rbumpLast = gamepad2.right_bumper;
            lbumpLast = gamepad2.left_bumper;

            */

            //telemetry.addData("scrollpose: ", scrollPoses);
//            telemetry.addData("slidePos1", dumbBot.slide1.getCurrentPosition());
//            telemetry.addData("slidePos2", dumbBot.slide2.getCurrentPosition());
            telemetry.addData("slidePos3", dumbBot.slide3.getCurrentPosition());

            telemetry.addData("Lclaw",dumbBot.lowClaw.getPosition());
            telemetry.addData("Lrot",dumbBot.lowRot.getPosition());
            telemetry.addData("Lbonk",dumbBot.lowBonk.getPosition());
            telemetry.addData("Lturn",dumbBot.lowTurn.getPosition());

            /*telemetry.addData("Hclaw",dumbBot.highClaw.getPosition());
            telemetry.addData("Hturn",dumbBot.highTurn.getPosition());
            telemetry.addData("Hbonk",dumbBot.highBonkL.getPosition());



            telemetry.addLine("1: blockplace\n2:transfer\n3:specgrab\n4:blockplace");

             */


            telemetry.update();
            first = false;
        }
    }
}
