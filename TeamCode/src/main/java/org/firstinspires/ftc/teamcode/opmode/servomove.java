package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servothing", group = "test")

public class servomove extends LinearOpMode {
    public Servo lowClaw, lowRot, lowBonk, lowTurn, highClaw, highTurn, highBonkL, highBonkR;

    public boolean clawOpen = false, clawOpenH = false, clawRot = false, bonk = false, left = false;
    public boolean aLast = false, rstickpressLast = false, rbumpLast = false, lbumpLast = false, bLast = false;
    public double rstickxLast = 0, rstickyLast = 0, lTpos = 0.46;
    public int scrollPoses = 1;

//    Telemetry telemetry;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lowClaw = hardwareMap.get(Servo.class, "lowClaw" ); // 0 (close) 0.26 (open)
        lowRot = hardwareMap.get(Servo.class, "lowRot" ); //0.21 (par) 0.53 (perp)
        lowBonk = hardwareMap.get(Servo.class, "lowBonk" ); //0.76 (down) 0.7 (searching or smth) 0.25 (back)
        lowTurn = hardwareMap.get(Servo.class, "lowTurn" );// L0.13, M0.46, R0.83


        highClaw = hardwareMap.get(Servo.class, "highClaw" ); // 0.79 (close) 0.54 (open)
        highBonkL = hardwareMap.get(Servo.class, "highBonkL" ); // 0.31 (specplace) 0.35 (specgrab) 0.285 (blockplace) 0.22 (transfer)
        highBonkR = hardwareMap.get(Servo.class, "highBonkR" ); // see above
        highTurn = hardwareMap.get(Servo.class, "highTurn" ); // 0.13 to 0 (specplace) 0.79 to 0.1 (specgrab) 0.24 to 0.02(blockplace) 0.81 to 0.1 (transfer)



/*        lowClaw.scaleRange(0.23,0.48);
        lowRot.scaleRange(0.21,0.53);
        lowBonk.scaleRange(0.08,0.65);
        lowTurn.scaleRange(0,0.46);
        highClaw.scaleRange(0.54,0.79);
        */
//        highTurn.scaleRange(0.13,0.46);
//        highBonk.scaleRange(0.13,0.46);



        waitForStart();

        while (opModeIsActive()) {
            if(!aLast && gamepad2.a){
                clawOpen = !clawOpen;
            }
            lowClaw.setPosition(clawOpen? 0.23:0.48);

            aLast = gamepad2.a;

            if(!bLast && gamepad2.b){
                clawOpenH = !clawOpenH;
            }
            highClaw.setPosition(clawOpen? 0.49:0.24);

            bLast = gamepad2.b;

            if (gamepad2.right_stick_button && !rstickpressLast){
                if (Math.abs(lowRot.getPosition() - 0.53) < 0.02) {
                    lowRot.setPosition(0.21);
                    clawRot = true;
                }
                else if (Math.abs(lowRot.getPosition() - 0.21) < 0.02) {
                    lowRot.setPosition(0.53);
                    clawRot = false;
                }
                else{
                    lowRot.setPosition(0.53);
                    clawRot = false;
                }
            }
            //lowRot.setPosition(clawRot? 0.53:0.21);
            rstickpressLast = gamepad2.right_stick_button;



            if(Math.abs(gamepad2.right_stick_y)>0.9 && Math.abs(rstickyLast)<0.9){
                bonk = !bonk;
            }
            lowBonk.setPosition(bonk? 0.73:0.22);
            rstickyLast = gamepad2.right_stick_y;

            if(Math.abs(gamepad2.right_stick_x)>0.5){
                lTpos+=gamepad2.right_stick_x*0.005;
                lTpos = Math.max(Math.min(lTpos,0.83),0.13);
                if (!clawRot){
                    if (lTpos < 0.32) {
                        lowRot.setPosition(1-Math.abs(lTpos - 0.32));
                    } else {
                        lowRot.setPosition(lTpos);
                    }
                }
                else {
                    if (lTpos < 0.32) {
                        lowRot.setPosition(Math.abs(lTpos + 0.32));
                    } else {
                        lowRot.setPosition(lTpos - 0.32);
                    }
                }
            }
            lowTurn.setPosition(lTpos);

            if (gamepad2.x){ //grabb
                lTpos = 0.46;
                lowRot.setPosition(0.53);
                bonk = true;
                clawOpen = true;
            }


            rstickxLast = gamepad2.right_stick_x;


            if (gamepad2.left_bumper && !lbumpLast){
                scrollPoses--;
                scrollPoses = Math.max(scrollPoses,1);
            }
            else if (gamepad2.right_bumper && !rbumpLast){
                scrollPoses++;
                scrollPoses = Math.min(scrollPoses,4);
            }

            if ((gamepad2.right_bumper && !rbumpLast) || (gamepad2.left_bumper && !lbumpLast)) {
                if (scrollPoses == 3) {
                    highTurn.setPosition(0.81); //specgrab
                    clawOpenH = true;
                    highBonkL.setPosition(0.15);
                    highBonkR.setPosition(0.15);

                } else if (scrollPoses == 2) {
                    bonk = false;
                    lTpos = 0.65;
                    lowRot.setPosition(0.65);
                    clawOpenH = true;
                    highBonkL.setPosition(0.31);
                    highBonkR.setPosition(0.31);
                    highTurn.setPosition(0.81);

                } else if (scrollPoses == 1) {
                    highTurn.setPosition(0.24); //blockplace
                    highBonkL.setPosition(0.26);
                    highBonkR.setPosition(0.26);
                } else if (scrollPoses == 4) {
                    highTurn.setPosition(0.13); //specplace
                    highBonkL.setPosition(0.21);
                    highBonkR.setPosition(0.21);
                }
            }
            rbumpLast = gamepad2.right_bumper;
            lbumpLast = gamepad2.left_bumper;







        telemetry.addData("Lclaw",lowClaw.getPosition());
        telemetry.addData("Lrot",lowRot.getPosition());
        telemetry.addData("Lbonk",lowBonk.getPosition());
        telemetry.addData("Lturn",lowTurn.getPosition());
        telemetry.addData("scrollposes ",scrollPoses);

        telemetry.addData("Hclaw",highClaw.getPosition());
        telemetry.addData("Hturn",highTurn.getPosition());
        telemetry.addData("Hbonk",highBonkL.getPosition());

            telemetry.update();
        }
    }
}


