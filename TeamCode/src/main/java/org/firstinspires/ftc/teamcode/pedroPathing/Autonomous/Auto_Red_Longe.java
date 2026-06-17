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

@Autonomous(name = "RED Auto - Longe", group = "Auto")
public class Auto_Red_Longe extends LinearOpMode {

    Follower follower;

    private final Pose startPose = new Pose(89.17, 8.27, 0);
    private final Pose scorePose = new Pose(89.62, 17.31, 0);
    private final Pose takePose_3 = new Pose(137, 35.86, 0);
    private final Pose takePose_Canto = new Pose(134.41, 16.41, Math.toRadians(-17.47));
    private final Pose takePose_CantoX = new Pose(134.41, 10.03, Math.toRadians(-9.04));
    private final Pose cyclePose_R = new Pose(108.8, 16.09, 0);
    private final Pose cyclePose_R_2 = new Pose(135.5, 16, 0);
    private final Pose cycleTakePose = new Pose(135.5, 16, Math.toRadians(-20));
    private final Pose outPose = new Pose(87, 36.05, 0);
    PathChain scoreF, take3, takeCanto, takeX, score3, scoreX, cycle_R, cycle_R2, cycle_RR, cycleTake, cycleScore, outLine;

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
        take3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(86.51, 39.35),
                                takePose_3
                        )
                )
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        takeCanto = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, takePose_Canto))
                .setLinearHeadingInterpolation(scorePose.getHeading(), takePose_Canto.getHeading())
                .build();
        takeX = follower.pathBuilder()
                .addPath(new BezierLine(takePose_Canto, takePose_CantoX))
                .setLinearHeadingInterpolation(takePose_Canto.getHeading(), takePose_CantoX.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(takePose_3, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        scoreX = follower.pathBuilder()
                .addPath(new BezierLine(cyclePose_R_2, scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        cycle_R = follower.pathBuilder()
                .addPath(new BezierLine(cycleTakePose, cyclePose_R))
                .setLinearHeadingInterpolation(takePose_CantoX.getHeading(), cyclePose_R.getHeading())
                .build();
        cycle_R2 = follower.pathBuilder()
                .addPath(new BezierLine(cyclePose_R, cyclePose_R_2))
                .setLinearHeadingInterpolation(cyclePose_R.getHeading(), cyclePose_R_2.getHeading())
                .build();
        cycle_RR = follower.pathBuilder()
                .addPath(new BezierLine(cyclePose_R_2, scorePose))
                .setConstantHeadingInterpolation(cyclePose_R_2.getHeading())
                .build();
        cycleTake = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(107.04, 30.17),
                                cycleTakePose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), cycleTakePose.getHeading())
                .build();
        cycleScore = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                cycleTakePose,
                                new Pose(107.04, 30.17),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(cycleTakePose.getHeading(), scorePose.getHeading())
                .build();
        outLine = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, outPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        Command goScore_F = follow(follower, scoreF);
        Command toTake_3 = follow(follower, take3);
        Command toTake_Canto = follow(follower, takeCanto);
        Command toTakeX = follow(follower, takeX);
        Command toTakeXR = follow(follower, cycle_R);
        Command toTakeXR_2 = follow(follower, cycle_RR);
        Command goScore_3 = follow(follower, score3);
        Command goScoreX = follow(follower, scoreX);
        Command toTake_Cycle = follow(follower, cycleTake);
        Command goScore_Cycle = follow(follower, cycleScore);
        Command goOut = follow(follower, outLine);

        Command onIntake = instant(() -> intake.setPower(1));
        Command offIntake = instant(() -> intake.setPower(0));

        Command abrirTrava = instant(() -> s1.setPosition(0.52));
        Command fecharTrava = instant(() -> s1.setPosition(0.63));

        Command mirar = instant(() -> encoder(tower, -400, 0.5));
        Command zerar = instant(() -> encoder(tower, 0, 0.5));

        Command onShotR = instant(() -> l_right.setVelocity(1950));
        Command onShotL = instant(() -> l_left.setVelocity(1950));
        Command offShotR = instant(() -> l_right.setVelocity(1000));
        Command offShotL = instant(() -> l_left.setVelocity(1000));
        Command zeroShotR = instant(() -> l_right.setVelocity(0));
        Command zeroShotL = instant(() -> l_left.setVelocity(0));

        Command shot_on = parallel(
                onShotR,
                onShotL
        );

        Command shot_off = parallel(
                offShotR,
                offShotL
        );

        Command shot_zero = parallel(
                zeroShotR,
                zeroShotL
        );

        Command sequence = sequential(
                parallel(
                        shot_on,
                        mirar,
                        goScore_F
                ),
                waitMs(1000),
                abrirTrava,
                onIntake,
                waitMs(1700),
                shot_off,
                parallel(
                        sequential(waitMs(500),
                                fecharTrava),
                        race(
                                toTake_3,
                                waitMs(1750)
                        )
                ),
                waitMs(500),
                parallel(
                        sequential(
                                waitMs(1000),
                                offIntake
                        ),
                        parallel(
                                shot_on,
                                goScore_3
                        )
                ),
                waitMs(500),
                abrirTrava,
                onIntake,
                waitMs(1500),
                shot_off,
                parallel(
                        sequential(
                                waitMs(500),
                                fecharTrava
                        ),
                        race(
                                toTake_Canto,
                                waitMs(1750)
                        )
                ),
                race(
                        toTakeX,
                        waitMs(1000)
                ),
                waitMs(100),
                parallel(
                        sequential(
                                waitMs(1000),
                                offIntake
                        ),
                        parallel(
                                shot_on,
                                goScoreX
                        )
                ),
                abrirTrava,
                onIntake,
                waitMs(1500),
                shot_off,
                parallel(
                        sequential(
                                waitMs(500),
                                fecharTrava
                        ),
                        race(
                                toTake_Cycle,
                                waitMs(2000)
                        )
                ),
                parallel(
                        sequential(
                                waitMs(800),
                                offIntake
                        ),
                        parallel(
                                shot_on,
                                goScore_Cycle
                        )
                ),
                waitMs(100),
                abrirTrava,
                onIntake,
                waitMs(1450),
                shot_off,
                parallel(
                        sequential(
                                waitMs(500),
                                fecharTrava
                        ),
                        race(
                                toTake_Cycle,
                                waitMs(2000)
                        )
                ),
                race(
                        toTakeXR,
                        waitMs(2000)
                ),
                race(
                        toTakeXR_2,
                        waitMs(2000)
                ),
                parallel(
                        sequential(
                                waitMs(1000),
                                offIntake
                        ),
                        parallel(
                                shot_on,
                                goScore_Cycle
                        )
                ),
                abrirTrava,
                onIntake,
                waitMs(1500),
                parallel(
                        goOut,
                        shot_zero,
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
