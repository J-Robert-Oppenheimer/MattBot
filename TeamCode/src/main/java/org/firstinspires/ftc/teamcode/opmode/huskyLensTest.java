package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
@TeleOp(name = "huskyTest", group = "Sensor")
public class huskyLensTest extends LinearOpMode {
    DcMotor slide3;
    public Servo lowClaw, lowRot, lowBonk, lowTurn;

    public boolean clawOpen = false, clawRot = false, bonk = false, left = false;
    public boolean aLast = false, rstickpressLast = false, rbumpLast = false, lbumpLast = false;
    public double rstickxLast = 0, rstickyLast = 0, lTpos = 0.46;
    public static int currentColor = 1;
    //dumbMap dumbBot = new dumbMap(this);


    //public dumbMap botboy = new dumbMap(this);
    @Override
    public void runOpMode() throws InterruptedException {
        slide3 = hardwareMap.dcMotor.get("anvilslide"); //0 - -400
        slide3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide3.setDirection(DcMotorSimple.Direction.REVERSE);
        slide3.setTargetPosition(0);
        slide3.setPower(1);
        slide3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lowClaw = hardwareMap.get(Servo.class, "lowClaw" ); // 0.48 (close) 0.23 (open)
        lowRot = hardwareMap.get(Servo.class, "lowRot" ); //0.21 (par) 0.53 (perp)
        lowBonk = hardwareMap.get(Servo.class, "lowBonk" ); //0 (down) 0.36 (up) 0.65 (back)
        lowTurn = hardwareMap.get(Servo.class, "lowTurn" );// L0.13, M0.46, R0.83

        HuskyLens.Block[] list;

        boolean searching = false;

        /*boolean a_on = false;
        boolean aLast = false;
        boolean bLast = false;
        boolean b_on = false;
        boolean y_on = false;
        boolean yLast = false;
        boolean x_on=true;

         */
        boolean found= false;
        boolean foundLast = false;
        boolean vertical = false;
        boolean horizontal= false;
        boolean verticalLast = false;
        boolean psLast = false;
        boolean searchLast = false;
        boolean STOP = false;
        //int pause = 0;
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "Huskylens");
        //ColorRangeSensor dsc = hardwareMap.get(ColorRangeSensor.class, "name");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        //botboy.init();
        waitForStart();
        int newArea;
        int largestArea = 0;
        int position = 0;

        while (opModeIsActive()) {
            slide3.setPower(1);
            list = huskyLens.blocks(currentColor);
            /*for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());    u
                telemetry.addData("Block", blocks[i].height * blocks[i].width);

                /*if (blocks[i].height > blocks[i].width) {
                    sleep(200);
                    cWrist.setPosition(0.9);
                }*/

            /*if (blocks.length > 0) {
                telemetry.addData("Block count", blocks.length);
                for (int i = 0; i < blocks.length; i++) {
                    telemetry.addData("Block", blocks[i].toString());
                    telemetry.addData("Block", blocks[i].height * blocks[i].width);



                /*for (int k = 0; k < blocks.length; k++) {
                    newArea = blocks[k].height * blocks[k].width;
                    if (newArea > largestArea) {
                        largestArea = newArea;
                        position = k;
                    }
                }
                /*if (blocks[position].id == 1) {
                    telemetry.addLine("yellow");
                    telemetry.addData("", claw.getPosition());
                    //claw.setPosition(0.7); open
                }
                if (blocks[position].id == 2) {
                    telemetry.addLine("red");
                    telemetry.addData("", claw.getPosition());
                    //claw.setPosition(0.4261);close
                }
                if (blocks[position].id == 3) {
                    telemetry.addLine("blue");
                    telemetry.addData("", claw.getPosition());
                    claw.setPosition(0.2);
                }*/
                /*if (blocks[i].width < blocks[i].height) {
                    cWrist.setPosition(0.84);
                } else {
                    cWrist.setPosition(0.16);
                }
            }*/
            searchLast = searching;
            found = list.length>0;
            if (found){
                searching = false;
                ///telemetry.addData("HSblocks:", huskyLens.blocks());
                vertical = list[0].height>list[0].width*1.3;
                //horizontal=list[0].height>list[0].width;
                telemetry.addData("h", list[0].height);
                telemetry.addData("w (x1.3)", list[0].width * 1.3);
                telemetry.addData("colorIdno: ", list[0].id);
                telemetry.addData("x: ", list[0].x);
                telemetry.addData("y: ", list[0].y);

            }

        if (/*gamepad2.dpad_down &&*/ found && (!foundLast || (vertical != verticalLast)))
        {
            STOP = true;
            if(lowRot.getPosition()>0.51 && lowRot.getPosition()<0.55)  {
                clawRot=true;
                lowRot.setPosition(0.53);

            }

            else if (lowRot.getPosition()>0.19 && lowRot.getPosition()<0.23) {
                clawRot = false;
                lowRot.setPosition(0.21);


                //if(slide.getCurrentPosition()<800){slideangle.setTargetPosition(-190);}
                }


            }

            verticalLast=vertical;
            telemetry.addData("v", vertical);
            telemetry.addData("f", found);
            telemetry.addData("lowRotpos", lowRot.getPosition());
            telemetry.addData("slide", slide3.getCurrentPosition());
            telemetry.addData("searching: ", searching);

            if (gamepad2.ps && !psLast) {
                searching = !searching;
            }
            psLast = gamepad2.ps;
            if (searching && !STOP && slide3.getTargetPosition() > -400 ){
                slide3.setTargetPosition(slide3.getCurrentPosition()-50);
            }

            /*if (!searching && searchLast && !gamepad2.ps) {
                if (slide.getCurrentPosition()>400) {
                    if (!dumbBot.clawRot) {
                        slide.setTargetPosition(slide.getCurrentPosition() + 250);
                        slideangle.setTargetPosition(0);
                        dumbBot.clawRot = false;
                    } else {
                        lowRot.setPosition(0.73);
                        slide.setTargetPosition(slide.getCurrentPosition() + 300);
                    }
                    telemetry.addLine("THE THING2!!!");
                    sleep(50);
                    slideangle.setTargetPosition(-160);
                }
            }

             */
            if (gamepad2.left_stick_x>0 && !STOP) {
                slide3.setTargetPosition(slide3.getCurrentPosition() - 50);
            }


            else if (gamepad2.left_stick_x<0){
                slide3.setTargetPosition(slide3.getCurrentPosition()+50);
            }
            else if (Math.abs(gamepad2.left_stick_y)>0.9){
                slide3.setTargetPosition(0);
            }



            if(!aLast && gamepad2.a){
                clawOpen = !clawOpen;
            }
            if(clawOpen && aLast && !gamepad2.a){
                slide3.setTargetPosition(0);
                STOP = false;
            }


            if(!aLast && gamepad2.a){
                clawOpen = !clawOpen;
            }
            lowClaw.setPosition(clawOpen? 0.23:0.48);

            aLast = gamepad2.a;

            if (gamepad2.right_stick_button && !rstickpressLast){
                if (Math.abs(lowRot.getPosition() - 0.53) < 0.02) {
                    lowRot.setPosition(0.21);
                    clawRot = true;
                }
                else if (Math.abs(lowRot.getPosition() - 0.21) < 0.02) {
                    lowRot.setPosition(0.53);
                    clawRot = false;
                }
                else {
                    lowRot.setPosition(0.53);
                    clawRot = false;
                }
            }
            //lowRot.setPosition(clawRot? 0.53:0.21);
            rstickpressLast = gamepad2.right_stick_button;



            if(Math.abs(gamepad2.right_stick_y)>0.9 && Math.abs(rstickyLast)<0.9){
                bonk = !bonk;
            }
            lowBonk.setPosition(bonk? 0.08:0.65);
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

            if(gamepad2.dpad_up) {
                currentColor = 1;
            }
            else if(gamepad2.dpad_left) {
                currentColor = 2;
            }
            else if(gamepad2.dpad_right) {
                currentColor = 3;
            }
            if (currentColor == 1){
                telemetry.addLine("current color: YELLOW");
            }
            else if (currentColor == 2){
                telemetry.addLine("current color: RED");
            }
            else if  (currentColor == 3){
                telemetry.addLine("current color: BLUE");
            }



            telemetry.update();
        }
    }
}

