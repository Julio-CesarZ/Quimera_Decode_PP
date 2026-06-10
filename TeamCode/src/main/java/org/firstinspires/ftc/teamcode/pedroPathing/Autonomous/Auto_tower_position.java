package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

@Autonomous(name = "TowerTest", group = "Test")
public class Auto_tower_position extends LinearOpMode {

    Follower follower;

    private final Pose startPose = new Pose(72, 72,  Math.toRadians(180));
    private final Pose startPose2 = new Pose(72, 72,  Math.toRadians(180));
    PathChain startpoint, pose2;

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

        startpoint = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startPose2))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        Command goScore_1 = follow(follower, startpoint);

        Command onIntake = instant(() -> intake.setPower(1));
        Command offIntake = instant(() -> intake.setPower(0));

        Command abrirTrava = instant(() -> s1.setPosition(0.6));
        Command fecharTrava = instant(() -> s1.setPosition(0.82));


        Command mirar = instant(() -> encoder(tower, 300, 0.5));
        Command mirar2 = instant(() -> encoder(tower, 500, 0.5));
        Command mirar1 = instant(() -> encoder(tower, 700, 0.5));

        Command onShotR_1400 = instant(() -> l_right.setVelocity(1400));
        Command onShotL_1400 = instant(() -> l_left.setVelocity(1400));
        Command onShotR_1450 = instant(() -> l_right.setVelocity(1450));
        Command onShotL_1450 = instant(() -> l_left.setVelocity(1450));
        Command offShotR = instant(() -> l_right.setVelocity(0));
        Command offShotL = instant(() -> l_left.setVelocity(0));

        Command shot_on_1400 = parallel(
                onShotR_1400,
                onShotL_1400,
                mirar
        );

        Command toShot_1400 = parallel(
                goScore_1,
                shot_on_1400,
                abrirTrava,
                mirar2
        );

        Command shot_on_1450 = parallel(
                onShotR_1450,
                onShotL_1450,
                mirar1
        );

        Command toShot_1450 = parallel(
                goScore_1,
                shot_on_1450,
                abrirTrava,
                mirar
        );

        Command shot_off = parallel(
                offShotR,
                offShotL
        );

        Command firstShot =sequential(
                waitMs(1100),
                abrirTrava,
                onIntake
        );

        Command sequence = sequential(
                parallel(
                        toShot_1400,
                        mirar,
                        firstShot
                ),
                waitMs(400),
                parallel(
                        sequential(waitMs(300),
                                fecharTrava)
                ),
                waitMs(200),
                offIntake,
                toShot_1450,
                onIntake,
                mirar1,
                waitMs(1500),
                parallel(
                        sequential(waitMs(300),
                                fecharTrava)
                ),
                waitMs(200),
                offIntake,
                abrirTrava,
                onIntake,
                waitMs(2000),
                parallel(
                        offIntake,
                        fecharTrava,
                        shot_off
                )
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
