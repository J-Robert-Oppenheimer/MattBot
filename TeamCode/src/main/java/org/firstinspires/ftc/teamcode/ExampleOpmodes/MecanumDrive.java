package org.firstinspires.ftc.teamcode.ExampleOpmodes;



import com.qualcomm.robotcore.hardware.DcMotor;


public class MecanumDrive {

    //variables for movement

    private DcMotor frontLeftDrive = null;

    private DcMotor frontRightDrive = null;

    private DcMotor backLeftDrive = null;

    private DcMotor backRightDrive = null;



    //previous encoders values to see what the target should really be

    //this takes away the need to restart the encoders when setting a new target position,

    //because the inputed target could be used as the change in the encoder value instead

    //of an exact target

    private int prevFL = 0;

    private int prevFR = 0;

    private int prevBL = 0;

    private int prevBR = 0;



    //these values are the actual target values of the encoder, instead of the displacement

    //prev + dist = exact if forward, prev - dist = exact if backward

    private int exactFL = 0;

    private int exactFR = 0;

    private int exactBL = 0;

    private int exactBR = 0;

    private boolean roamMode = false;



    //constructor, input the location of the motors(f - front, b - back, l - left, r - right)

    public MecanumDrive(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){

        //set all the motors

        frontLeftDrive = fl;

        frontRightDrive = fr;

        backLeftDrive = bl;

        backRightDrive = br;

    }



    //tele-op movement

    public void moveInTeleop(double x1, double y1, double x2, double movementPower){

        //allow the robot to move with set power

        setRoamMode();



        //power equals forwards + sideways + rotation

        double fl = -y1 + x1 + x2;

        double fr = -y1 - x1 - x2;

        double bl = -y1 - x1 + x2;

        double br = -y1 + x1 - x2;

        double mag = Math.abs(y1) + Math.abs(x1) + Math.abs(x2);



        //scale each motor power based on the total magnitude

        fl /= mag;

        fr /= mag;

        bl /= mag;

        br /= mag;



        //scale each motor power by the movement power

        fl *= movementPower;

        fr *= movementPower;

        bl *= movementPower;

        br *= movementPower;



        //activate the motion controller

        setPowerCustom(fl, fr, bl, br);

    }



    //correctional movement methods

    //corrects chases where:

    //

    //  -the encoders are stuck, therefore the robot is ramming into an object

    //  -there is an object in front of a distance sensor that the robot must go around

    //

    //if the encoders all end on the correct inputed value, the displacement

    //will be the same no matter what therefore the robot can take any path where

    //the encoders will end like this. The value traveled in an offset direction

    //(in encoder units) can be tracked and undone at a different time to achieve

    //have the encoders end on the correct values





    //basic movement methods

    //these are the methods that will tell the robot to simply move to a certain position

    //it simply tells the robot to move a set amount and a certain distance

    //set to move forwards a set distance

    public void setMoveForward(int dist){

        //make the robot move to a set position
        //calculate the distance the encoder must change to move to the given value

        exactFL = prevFL + dist;

        exactFR = prevFR + dist;

        exactBL = prevBL + dist;

        exactBR = prevBR + dist;



        //make the encoders move the given distance

        frontLeftDrive.setTargetPosition(exactFL);

        frontRightDrive.setTargetPosition(exactFR);

        backLeftDrive.setTargetPosition(exactBL);

        backRightDrive.setTargetPosition(exactBR);

        setTargetMode();
    }



    //move forwards in roam mode

    public void moveForward(double pow){

        //allow the robot to move with set power

        setRoamMode();



        //make the encoders move the given distance

        frontLeftDrive.setPower(pow);

        frontRightDrive.setPower(pow);

        backLeftDrive.setPower(pow);

        backRightDrive.setPower(pow);

    }



    //set to move backwards a set distance

    public void setMoveBackward(int dist){

        //make the robot move to a set position

        setTargetMode();



        //calculate the distance the encoder must change to move to the given value

        exactFL = prevFL - dist;

        exactFR = prevFR - dist;

        exactBL = prevBL - dist;

        exactBR = prevBR - dist;



        //make the encoders move the given distance

        frontLeftDrive.setTargetPosition(exactFL);

        frontRightDrive.setTargetPosition(exactFR);

        backLeftDrive.setTargetPosition(exactBL);

        backRightDrive.setTargetPosition(exactBR);

    }



    //move backwards in roam mode

    public void moveBackward(double pow){

        //allow the robot to move with set power

        setRoamMode();



        //make the encoders move the given distance

        frontLeftDrive.setPower(-pow);

        frontRightDrive.setPower(-pow);

        backLeftDrive.setPower(-pow);

        backRightDrive.setPower(-pow);

    }



    //set to move left a set distance

    public void setMoveLeft(int dist){

        //make the robot move to a set position



        //calculate the distance the encoder must change to move to the given value

        exactFL = prevFL - dist;

        exactFR = prevFR + dist;

        exactBL = prevBL + dist;

        exactBR = prevBR - dist;



        //make the encoders move the given distance

        frontLeftDrive.setTargetPosition(exactFL);

        frontRightDrive.setTargetPosition(exactFR);

        backLeftDrive.setTargetPosition(exactBL);

        backRightDrive.setTargetPosition(exactBR);

        setTargetMode();
    }



    //move right in roam mode

    public void moveLeft(double pow){

        //allow the robot to move with set power

        setRoamMode();



        //make the encoders move the given distance

        frontLeftDrive.setPower(-pow);//was -

        frontRightDrive.setPower(pow);

        backLeftDrive.setPower(pow);

        backRightDrive.setPower(-pow);// was -

    }



    //set to move right a set distance

    public void setMoveRight(int dist){

        //make the robot move to a set position



        //calculate the distance the encoder must change to move to the given value

        exactFL = prevFL + dist;

        exactFR = prevFR - dist;

        exactBL = prevBL - dist;

        exactBR = prevBR + dist;



        //make the encoders move the given distance

        frontLeftDrive.setTargetPosition(exactFL);

        frontRightDrive.setTargetPosition(exactFR);

        backLeftDrive.setTargetPosition(exactBL);

        backRightDrive.setTargetPosition(exactBR);

        setTargetMode();
    }



    //move right in roam mode

    public void moveRight(double pow){

        //allow the robot to move with set power

        setRoamMode();



        //make the encoders move the given distance

        frontLeftDrive.setPower(pow);

        frontRightDrive.setPower(-pow);//was -

        backLeftDrive.setPower(-pow);//was -

        backRightDrive.setPower(pow);

    }



    //set to rotate cw a set distance

    public void setRotateCW(int dist){

        //make the robot move to a set position

        setTargetMode();



        //calculate the distance the encoder must change to move to the given value

        exactFL = prevFL + dist;

        exactFR = prevFR - dist;

        exactBL = prevBL + dist;

        exactBR = prevBR - dist;



        //make the encoders move the given distance

        frontLeftDrive.setTargetPosition(exactFL);

        frontRightDrive.setTargetPosition(exactFR);

        backLeftDrive.setTargetPosition(exactBL);

        backRightDrive.setTargetPosition(exactBR);

    }



    //rotate cw in roam mode

    public void rotateCW(double pow){

        //allow the robot to move with set power

        setRoamMode();



        //make the encoders move the given distance

        frontLeftDrive.setPower(pow);

        frontRightDrive.setPower(-pow);

        backLeftDrive.setPower(pow);

        backRightDrive.setPower(-pow);

    }



    //set to rotate ccw a set distance

    public void setRotateCCW(int dist){

        //make the robot move to a set position

        setTargetMode();



        //calculate the distance the encoder must change to move to the given value

        exactFL = prevFL - dist;

        exactFR = prevFR + dist;

        exactBL = prevBL - dist;

        exactBR = prevBR + dist;



        //make the encoders move the given distance

        frontLeftDrive.setTargetPosition(exactFL);

        frontRightDrive.setTargetPosition(exactFR);

        backLeftDrive.setTargetPosition(exactBL);

        backRightDrive.setTargetPosition(exactBR);

    }



    //rotate ccw in roam mode

    public void rotateCCW(double pow){

        //allow the robot to move with set power

        setRoamMode();



        //make the encoders move the given distance

        frontLeftDrive.setPower(-pow);

        frontRightDrive.setPower(pow);

        backLeftDrive.setPower(-pow);

        backRightDrive.setPower(pow);

    }



    //diagnol movements, mixing forwards or backwards with left or right

    //move forwards and to the left

    public void moveForwardLeft(double pow){

        setRoamMode();

        frontLeftDrive.setPower(0.0);

        frontRightDrive.setPower(pow);

        backLeftDrive.setPower(pow);

        backRightDrive.setPower(0.0);

    }



    //set to move forward-left a set distance

    public void setMoveForwardLeft(int dist){

        //make the robot move to a set position

        setTargetMode();



        //calculate the distance the encoder must change to move to the given value

        exactFL = prevFL;

        exactFR = prevFR + dist;

        exactBL = prevBL + dist;

        exactBR = prevBR;



        //make the encoders move the given distance

        frontLeftDrive.setTargetPosition(exactFL);

        frontRightDrive.setTargetPosition(exactFR);

        backLeftDrive.setTargetPosition(exactBL);

        backRightDrive.setTargetPosition(exactBR);

    }



    //move forwards and to the right

    public void moveForwardRight(double pow){

        setRoamMode();

        frontLeftDrive.setPower(pow);

        frontRightDrive.setPower(0.0);

        backLeftDrive.setPower(0.0);

        backRightDrive.setPower(pow);

    }



    //set to move forward-right a set distance

    public void setMoveForwardRight(int dist){

        //make the robot move to a set position

        setTargetMode();



        //calculate the distance the encoder must change to move to the given value

        exactFL = prevFL + dist;

        exactFR = prevFR;

        exactBL = prevBL;

        exactBR = prevBR + dist;



        //make the encoders move the given distance

        frontLeftDrive.setTargetPosition(exactFL);

        frontRightDrive.setTargetPosition(exactFR);

        backLeftDrive.setTargetPosition(exactBL);

        backRightDrive.setTargetPosition(exactBR);

    }



    //move backwards and to the left

    public void moveBackLeft(double pow){

        setRoamMode();

        frontLeftDrive.setPower(-pow);

        frontRightDrive.setPower(0.0);

        backLeftDrive.setPower(0.0);

        backRightDrive.setPower(-pow);

    }



    //set to move back-left a set distance

    public void setMoveBackLeft(int dist){

        //make the robot move to a set position

        setTargetMode();



        //calculate the distance the encoder must change to move to the given value

        exactFL = prevFL - dist;

        exactFR = prevFR;

        exactBL = prevBL;

        exactBR = prevBR - dist;



        //make the encoders move the given distance

        frontLeftDrive.setTargetPosition(exactFL);

        frontRightDrive.setTargetPosition(exactFR);

        backLeftDrive.setTargetPosition(exactBL);

        backRightDrive.setTargetPosition(exactBR);

    }



    //move backwards and to the right

    public void moveBackRight(double pow){

        setRoamMode();

        frontLeftDrive.setPower(0.0);

        frontRightDrive.setPower(-pow);

        backLeftDrive.setPower(-pow);

        backRightDrive.setPower(0.0);

    }



    //set to move back-right a set distance

    public void setMoveBackRight(int dist){

        //make the robot move to a set position

        setTargetMode();



        //calculate the distance the encoder must change to move to the given value

        exactFL = prevFL;

        exactFR = prevFR - dist;

        exactBL = prevBL - dist;

        exactBR = prevBR;



        //make the encoders move the given distance

        frontLeftDrive.setTargetPosition(exactFL);

        frontRightDrive.setTargetPosition(exactFR);

        backLeftDrive.setTargetPosition(exactBL);

        backRightDrive.setTargetPosition(exactBR);

    }



    //turn off all the motors to stop the robot

    public void stopMoving(){

        //stop all the motors

        frontLeftDrive.setPower(0.0);

        frontRightDrive.setPower(0.0);

        backLeftDrive.setPower(0.0);

        backRightDrive.setPower(0.0);

    }



    //set the power of each motor individually for a unique direction

    public void setPowerCustom(double fl, double fr, double bl, double br){

        setRoamMode();

        //set the power of each motor

        frontLeftDrive.setPower(fl);

        frontRightDrive.setPower(fr);

        backLeftDrive.setPower(bl);

        backRightDrive.setPower(br);

    }



    //allow the robot to move without relying on encoders to RUN_TO_POSITION

    public void setRoamMode(){

        roamMode = true;

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }



    //make the robot run to a set position

    public void setTargetMode(){

        roamMode = false;

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        prevFL = frontLeftDrive.getCurrentPosition();

        prevFR = frontRightDrive.getCurrentPosition();

        prevBL = backLeftDrive.getCurrentPosition();

        prevBR = backRightDrive.getCurrentPosition();

    }



    //set the power of the motors an inputed value (used for RUN_TO_POSITION)

    //if the    motor.getTargetPosition() == motor.getCurrentPosition()   for all

    //wheels on the robot, the robot has reached it's target position

    //

    //reached target - true

    //hasn't reached target - false

    public boolean setPowerToTarget(double pow){

        //check if motors are at their target

        if(withinReasonableRange(frontLeftDrive.getCurrentPosition(), frontLeftDrive.getTargetPosition())

                && withinReasonableRange(frontRightDrive.getCurrentPosition(), frontRightDrive.getTargetPosition())

                && withinReasonableRange(backLeftDrive.getCurrentPosition(), backLeftDrive.getTargetPosition())

                && withinReasonableRange(backRightDrive.getCurrentPosition(), backRightDrive.getTargetPosition())){

            //set the previous values for the next encoder target

            prevFL = frontLeftDrive.getCurrentPosition();

            prevFR = frontRightDrive.getCurrentPosition();

            prevBL = backLeftDrive.getCurrentPosition();

            prevBR = backRightDrive.getCurrentPosition();



            //the targets have been reached, return true

            return true;

        }



        //set the motor power

        frontLeftDrive.setPower(pow);

        frontRightDrive.setPower(pow);

        backLeftDrive.setPower(pow);

        backRightDrive.setPower(pow);



        //the motors aren't at their target

        return false;

    }



    //return true if the robot is in roam mode, false if in target mode

    public boolean isInRoamMode(){

        return roamMode;

    }



    //check if the robot has moved to it's given position

    public boolean movedToTarget(){

        return (withinReasonableRange(frontLeftDrive.getCurrentPosition(), exactFL) &&

                withinReasonableRange(frontRightDrive.getCurrentPosition(), exactFR) &&

                withinReasonableRange(backLeftDrive.getCurrentPosition(), exactBL) &&

                withinReasonableRange(backRightDrive.getCurrentPosition(), exactBR));

    }



    //check if the robot has moved to it's given position

    //(this may work better but I honestly haven't tested it (oops))



    public boolean movedToTarget2(){

        return !frontLeftDrive.isBusy() &&

                !frontRightDrive.isBusy() &&

                !backLeftDrive.isBusy() &&

                !backRightDrive.isBusy();

    }





    //information about the wheels and their targets

    public String wheelDistanceToTarget(){

        String s = "";

        s = s + "FL: " + (Math.abs(frontLeftDrive.getTargetPosition() - frontLeftDrive.getCurrentPosition()));

        s = s + "\nFR: " + (Math.abs(frontRightDrive.getTargetPosition() - frontRightDrive.getCurrentPosition()));

        s = s + "\nBL: " + (Math.abs(backLeftDrive.getTargetPosition() - backLeftDrive.getCurrentPosition()));

        s = s + "\nBR: " + (Math.abs(backRightDrive.getTargetPosition() - backRightDrive.getCurrentPosition()));

        return s;

    }



    //check if the inputed numbers are close to equal

    public boolean withinReasonableRange(int num1, int num2){

        int threshold = 55;

        if(num1 <= num2 + threshold && num1 >= num2 - threshold) return true;

        return false;

    }

}