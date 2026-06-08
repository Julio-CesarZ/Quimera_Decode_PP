package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            //.useSecondaryTranslationalPIDF(false)
            //.useSecondaryDrivePIDF(false)
            .forwardZeroPowerAcceleration(-54) //-54 - testar valores menores para ver o comportamento da aceleração e desaceleração
            .lateralZeroPowerAcceleration(-80) //-80
            //.centripetalScaling(0)
            //.predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.08, 0.0748824, 0.00128367))
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(1.75, 0, 0.02, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.1, 0.01))
            //Abaixo - Sem o predictive braking
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.012, 0.04))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.01, 0.02))
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0072, 0.0001, 0.00008, 0.6, 0.03))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0001, 0.6, 0.03))
            .centripetalScaling(0.0005)
            .mass(12.3);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants) //forward - X; Strafe - Y
                .build();
    }

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(62)
            .yVelocity(49)
            .useBrakeModeInTeleOp(true)
            .rightFrontMotorName("rf") //0
            .rightRearMotorName("rr") //3
            .leftRearMotorName("lr") //2
            .leftFrontMotorName("lf") //1
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.35)
            .strafePodX(-6.41)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

}
