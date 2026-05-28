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
            /*
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .forwardZeroPowerAcceleration(-57) // reduzir os valores caso o tranco seja muito alto
            .lateralZeroPowerAcceleration(-72) // conectar os caminhos sem o .build();
            .centripetalScaling(0)
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.05, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5,0,0.08,0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0.035))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.01, 0.015))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00001, 0.6, 0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000005, 0.6, 0.01))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.05962, 0.00175))
            */
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryDrivePIDF(true)
            .forwardZeroPowerAcceleration(-54)
            .lateralZeroPowerAcceleration(-80)
            .centripetalScaling(0)
            .headingPIDFCoefficients(new PIDFCoefficients(1.75, 0, 0.02, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.1,0,0.1,0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(0,0,0,0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0, 0, 0))
            .mass(12.3);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.75, 3.5);

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
            .yVelocity(49.4)
            .useBrakeModeInTeleOp(true)
            .rightFrontMotorName("rf") //0
            .rightRearMotorName("rr") //3z
            .leftRearMotorName("lr") //2
            .leftFrontMotorName("lf") //1
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
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
