package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.turnTo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RED Auto - Perto", group = "Auto")
public class Auto_Red_Perto extends LinearOpMode {

    Follower follower;

    private final Pose startPose = new Pose(110.47, 132.68, 0);
    private final Pose scorePose = new Pose(97.12, 83.03, 0);
    private final Pose takePose_1 = new Pose(126, 84.76, 0);
    private final Pose takePose_2 = new Pose(133.5, 58, 0);
    private final Pose takePose_Gate = new Pose(131.6, 59.45, Math.toRadians(30.8));
    private final Pose outPose = new Pose(83.83, 106.49, 0);
    PathChain scoreF, take1, take2, score2, takeG1, scoreG1, out;

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

        intake.setDirection(DcMotorEx.Direction.FORWARD);
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

        s1.setPosition(0.63);

        scoreF = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        take1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, takePose_1))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        take2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(94.205, 54.922),
                                takePose_2
                        )
                )
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                takePose_2,
                                new Pose(94.205, 54.922),
                                scorePose
                        )
                )
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        takeG1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(96.62, 66.82),
                                takePose_Gate
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), takePose_Gate.getHeading())
                .build();
        scoreG1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                takePose_Gate,
                                new Pose(99.6, 68.86),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(takePose_Gate.getHeading(), scorePose.getHeading())
                .build();
        out = follower.pathBuilder()
                .addPath(new BezierLine(takePose_1, outPose))
                .setConstantHeadingInterpolation(outPose.getHeading())
                .build();

        Command goScore_1 = follow(follower, scoreF);
        Command toTake_1 = follow(follower, take1);
        Command toTake_2 = follow(follower, take2);
        Command goScore_2 = follow(follower, score2);
        Command toGate_1 = follow(follower, takeG1);
        Command goScoreG_1 = follow(follower, scoreG1);
        Command outLine = follow(follower, out);

        Command onIntake = instant(() -> intake.setPower(1));
        Command offIntake = instant(() -> intake.setPower(0));

        Command abrirTrava = instant(() -> s1.setPosition(0.52));
        Command fecharTrava = instant(() -> s1.setPosition(0.63));

        Command mirar = instant(() -> encoder(tower, -315, 0.5));
        Command mirarF = instant(() -> encoder(tower, -195, 0.5));
        Command zerar = instant(() -> encoder(tower, 0, 0.5));

        Command onShotR_F = instant(() -> l_right.setVelocity(1400));
        Command onShotL_F = instant(() -> l_left.setVelocity(1400));
        Command onShotR_S = instant(() -> l_right.setVelocity(1500));
        Command onShotL_S = instant(() -> l_left.setVelocity(1500));
        Command offShotR = instant(() -> l_right.setVelocity(0));
        Command offShotL = instant(() -> l_left.setVelocity(0));

        Command shot_on_F = parallel(
                onShotR_F,
                onShotL_F
        );

        Command toShot_F = parallel(
                goScore_1,
                shot_on_F,
                abrirTrava
        );

        Command shot_on_S = parallel(
                onShotR_S,
                onShotL_S
        );

        Command toShot_S = parallel(
                goScore_2,
                shot_on_S
        );

        Command shot_off = parallel(
                offShotR,
                offShotL
        );

        Command firstShot = sequential(
                waitMs(900),
                abrirTrava,
                onIntake
        );

        Command lastShot = sequential(
                waitMs(600),
                abrirTrava
        );

        Command sequence = sequential(
                parallel(
                        toShot_F,
                        mirar,
                        firstShot
                ),
                parallel(
                        sequential(waitMs(300),
                                fecharTrava),
                        race(
                                toTake_2,
                                waitMs(1750)
                        )
                ),
                parallel(
                        sequential(
                                waitMs(800),
                                offIntake
                        ),
                        toShot_S
                ),
                abrirTrava,
                onIntake,
                waitMs(1100),
                parallel(
                        sequential(
                                waitMs(300),
                                fecharTrava
                        ),
                        race(
                                toGate_1,
                                waitMs(1750)
                        )
                ),
                waitMs(1350),
                parallel(
                        sequential(
                                waitMs(800),
                                offIntake
                        ),
                        goScoreG_1
                ),
                abrirTrava,
                onIntake,
                waitMs(1100),
                parallel(
                        sequential(
                                waitMs(300),
                                fecharTrava
                        ),
                        race(
                                toGate_1,
                                waitMs(1750)
                        )
                ),
                waitMs(1450),
                parallel(
                        sequential(
                                waitMs(800),
                                offIntake
                        ),
                        goScoreG_1
                ),
                abrirTrava,
                onIntake,
                waitMs(1250),
                parallel(
                        sequential(
                                waitMs(300),
                                fecharTrava
                        ),
                        race(
                                toGate_1,
                                waitMs(1750)
                        )
                ),
                waitMs(1600),
                parallel(
                        sequential(
                                waitMs(800),
                                offIntake
                        ),
                        goScoreG_1
                ),
                abrirTrava,
                onIntake,
                waitMs(1200),
                parallel(
                        sequential(waitMs(300),
                                fecharTrava),
                        race(
                                toTake_1,
                                waitMs(1500)
                        ),
                        mirarF
                ),
                parallel(
                        lastShot,
                        outLine
                ),
                parallel(
                        shot_off,
                        fecharTrava,
                        offIntake,
                        zerar
                )

        );

        waitForStart();

        schedule(sequence);

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    private void encoder(DcMotorEx motor, int novoAlvo, double power) {
        if (motor.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition(novoAlvo);
            motor.setPower(power);
        }
    }
}
