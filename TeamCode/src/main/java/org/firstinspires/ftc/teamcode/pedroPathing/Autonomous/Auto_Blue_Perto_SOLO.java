package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLUE Auto - Perto", group = "Test")
public class Auto_Blue_Perto_SOLO extends LinearOpMode {

    Follower follower;
    private final Pose startPose = new Pose(108.1, 133.91, Math.toRadians(180));
    private final Pose scorePose = new Pose(93.74, 85.37, Math.toRadians(180));
    private final Pose takePose_1 = new Pose(126.65, 84.76, Math.toRadians(180));
    PathChain scorePreload, take1;

    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx l_right = hardwareMap.get(DcMotorEx.class, "l_right");
        DcMotorEx l_left = hardwareMap.get(DcMotorEx.class, "l_left");
        DcMotorEx tower = hardwareMap.get(DcMotorEx.class, "tower");
        Servo s1 = hardwareMap.get(Servo.class, "s1");

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        l_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        l_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        l_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        l_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotorEx.Direction.REVERSE);
        l_right.setDirection(DcMotorEx.Direction.REVERSE);
        l_left.setDirection(DcMotorEx.Direction.REVERSE);

        PIDFCoefficients coefficientsRightMotor = l_right.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients coefficientsLeftMotor = l_left.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        l_right.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficientsRightMotor.p, coefficientsRightMotor.i, coefficientsRightMotor.d, coefficientsRightMotor.f * 1.5
        ));

        l_left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficientsLeftMotor.p, coefficientsLeftMotor.i, coefficientsLeftMotor.d, coefficientsLeftMotor.f * 1.5
        ));

        tower.setDirection(DcMotorEx.Direction.FORWARD);
        tower.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tower.setTargetPosition(0);
        tower.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        tower.setPower(0);
        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        s1.setPosition(0.82);

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        take1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, takePose_1))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        Command goScore = follow(follower, scorePreload);
        Command toTake_1 = follow(follower, take1);

        Command onIntake = instant(() -> intake.setPower(1));
        Command offIntake = instant(() -> intake.setPower(0));

        Command abrirTrava = instant(() -> s1.setPosition(0.6));
        Command fecharTrava = instant(() -> s1.setPosition(0.82));

        Command mirar = instant(() -> encoder(tower, -320, 0.5));

        Command onShotR = instant(() -> l_right.setVelocity(1450));
        Command onShotL = instant(() -> l_left.setVelocity(1450));
        Command offShotR = instant(() -> l_right.setVelocity(0));
        Command offShotL = instant(() -> l_left.setVelocity(0));

        Command shot_on = parallel(
                onShotR,
                onShotL
        );

        Command shot_off = parallel(
                offShotR,
                offShotL
        );

        Command toShot = parallel(
                goScore,
                shot_on,
                abrirTrava
        );

        Command sequence = sequential(
                parallel(
                        toShot,
                        mirar
                ),
                onIntake,
                waitMs(2000),
                shot_off,
                fecharTrava,
                toTake_1,
                offIntake,
                toShot,
                onIntake,
                waitMs(2000),
                shot_off,
                fecharTrava,
                offIntake
        );

        waitForStart();

        schedule(sequence);

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
        }
    }
    private void encoder(DcMotorEx motor, int novoAlvo, double power) {
        if (motor.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition(novoAlvo);
            motor.setPower(power);
        }
    }
}
