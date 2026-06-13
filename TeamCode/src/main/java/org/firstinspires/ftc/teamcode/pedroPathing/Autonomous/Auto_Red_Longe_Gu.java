package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.turnTo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "RED Auto - Longe (Gu)", group = "Auto")
public class Auto_Red_Longe_Gu extends LinearOpMode {

    Follower follower;

    private final Pose startPose = new Pose(89.17, 8.27, 0);
    private final Pose scoreShot = new Pose(80.20, 20.19, 0);
    private final Pose takePose_1 = new Pose(128.83, 35.51, 0);
    private final Pose takePose_2 = new Pose(130.2, 5.51, Math.toRadians(-39.5));
    //private final Pose takePose_Gate = new Pose(131, 59, Math.toRadians(30.8));
    //private final Pose outPose = new Pose(83.83, 106.49, 0);
    PathChain shot11, take1, shot12, take2, score2, takeG1, scoreG1, out;

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

        s1.setPosition(0.63);

        shot11 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scoreShot))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        take1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(88.000, 8.000, 0),
                        new Pose(93.6201, 34.49, 0),
                        new Pose(130.833, 60.514, 0)))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        shot12 = follower.pathBuilder()
                .addPath(new BezierLine(takePose_1, scoreShot))
                .setConstantHeadingInterpolation(takePose_1.getHeading())
                .build();
        take2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scoreShot,
                                new Pose(125.5587300, 21.2, Math.toRadians(-39.5)),
                                takePose_2
                        )
                )
                .setConstantHeadingInterpolation(scoreShot.getHeading())
                .build();
        /*score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                takePose_2,
                                new Pose(94.205, 54.922),
                                scoreShot
                        )
                )
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        takeG1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scoreShot,
                                new Pose(96.62, 66.82),
                                takePose_Gate
                        )
                )
                .setLinearHeadingInterpolation(scoreShot.getHeading(), takePose_Gate.getHeading())
                .build();
        scoreG1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                takePose_Gate,
                                new Pose(99.6, 68.86),
                                scoreShot
                        )
                )
                .setLinearHeadingInterpolation(takePose_Gate.getHeading(), scoreShot.getHeading())
                .build();
        out = follower.pathBuilder()
                .addPath(new BezierLine(takePose_1, outPose))
                .setConstantHeadingInterpolation(outPose.getHeading())
                .build(); */

        Command goScore_1 = follow(follower, shot11);
        Command toTake_1 = follow(follower, take1);
        Command goScore_2 = follow(follower, shot12);
        Command toTake_2 = follow(follower, take2);
        Command goScore_3 = follow(follower, score2);
        Command toGate_1 = follow(follower, takeG1);
        Command goScoreG_1 = follow(follower, scoreG1);
        Command outLine = follow(follower, out);

        Command onIntake = instant(() -> intake.setPower(1));
        Command offIntake = instant(() -> intake.setPower(0));

        Command abrirTrava = instant(() -> s1.setPosition(0.55));
        Command fecharTrava = instant(() -> s1.setPosition(0.63));

        Command mirar = instant(() -> encoder(tower, -410, 0.5));
        Command mirarF = instant(() -> encoder(tower, -410, 0.5));
        Command zerar = instant(() -> encoder(tower, 0, 0.5));

        Command onShotR_F = instant(() -> l_right.setVelocity(1350));
        Command onShotL_F = instant(() -> l_left.setVelocity(1350));
        Command onShotR_S = instant(() -> l_right.setVelocity(1780));
        Command onShotL_S = instant(() -> l_left.setVelocity(1780));
        Command offShotR = instant(() -> l_right.setVelocity(0));
        Command offShotL = instant(() -> l_left.setVelocity(0));

        Command shot_on_F = parallel(
                onShotR_F,
                onShotL_F
        );

        /*Command toShot_F = parallel(
                goScore_1,
                shot_on_F,
                abrirTrava
        ); */

        Command shot_on_S = parallel(
                onShotR_S,
                onShotL_S
        );

        Command toShot_S = parallel(
                //goScore_1,
                shot_on_S,
                abrirTrava
        );

        Command shot_off = parallel(
                offShotR,
                offShotL
        );

        Command disableR = parallel(
        );

        Command firstShot = sequential(
                waitMs(900),
                abrirTrava,
                onIntake
        );

        Command lastShot = sequential(
                waitMs(600),
                abrirTrava,
                onIntake
        );

        Command sequence = sequential(
                parallel(
                        toShot_S,
                        mirar,
                        firstShot,
                        goScore_1
                ),
                waitMs(1500),
                parallel(
                        sequential(waitMs(300),
                                fecharTrava
                        )
                ),
                shot_off,
                waitMs(1000),
                offIntake,
                waitMs(300),
                parallel(
                        toTake_1,
                        onIntake
                ),
                waitMs(1000),
                offIntake,
                goScore_1,
                waitMs(1000),
                parallel(
                        toShot_S,
                        mirar,
                        firstShot
                ),
                waitMs(1000),
                parallel(
                        offIntake,
                        shot_off,
                        fecharTrava
                ),
                parallel(
                        onIntake,
                        fecharTrava
                ),
                toTake_2,
                goScore_1,
                parallel(
                        offIntake,
                        shot_off,
                        fecharTrava
                ),
                waitMs(1000),
                zerar
                
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

                /*offIntake,
                toShot_S,
                onIntake,
                waitMs(1050),
                parallel(
                        sequential(waitMs(300),
                                fecharTrava),
                        race(
                                //toGate_1,
                                waitMs(1500)
                        )
                ),
                waitMs(1450),
                offIntake,
                abrirTrava,
               // goScoreG_1,
                onIntake,
                waitMs(1100),
                parallel(
                        sequential(waitMs(300),
                                fecharTrava),
                        race(
                                //toGate_1,
                                waitMs(1500)
                        )
                ),
                waitMs(1450),
                parallel(
                        offIntake//,
                        //goScoreG_1
                ),
                abrirTrava,
                onIntake,
                waitMs(1250),
                parallel(
                        sequential(waitMs(300),
                                fecharTrava),
                        race(
                                //toGate_1,
                                waitMs(1500)
                        )
                ),
                waitMs(1475),
                parallel(
                        offIntake//,
                        //goScoreG_1
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
                        )
                ),
                parallel(
                        offIntake,
                        mirarF,
                        lastShot//,
                        //outLine
                ),
                waitMs(100),
                parallel(
                        offIntake,
                        zerar,
                        shot_off,
                        fecharTrava */
