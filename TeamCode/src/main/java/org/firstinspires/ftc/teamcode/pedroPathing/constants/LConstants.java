package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class LConstants {
    static {
//16.375 wide 13.625 long
        PinpointConstants.forwardY = 3.5;
        PinpointConstants.strafeX = -1.625;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.customEncoderResolution = 13.26291192;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;



//        old values
//        PinpointConstants.forwardY = -3.0625;
//        PinpointConstants.strafeX = 6.8125;
//        PinpointConstants.distanceUnit = DistanceUnit.INCH;
//        PinpointConstants.hardwareMapName = "pinpoint";
//        PinpointConstants.useYawScalar = false;
//        PinpointConstants.yawScalar = 1.0;
//        PinpointConstants.useCustomEncoderResolution = false;
//        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
//        PinpointConstants.customEncoderResolution = 13.26291192;
//    PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
//        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;


    }
}




