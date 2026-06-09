package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.Tests;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.turnTo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Movimento", group = "Test")
public class justMovement extends LinearOpMode {

    Follower follower;
    private final Pose startPose = new Pose(109, 133.91, 0);
    private final Pose scorePose = new Pose(93.74, 85.37, 0);
    private final Pose takePose_1 = new Pose(126, 84.76, 0);
    private final Pose takePose_2 = new Pose(130.95, 59.5, 0);
    private final Pose center = new Pose(72, 72, Math.toRadians(180));
    PathChain score1, take1, take2, score2, center72;

    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        score1 = follower.pathBuilder()
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
        center72 = follower.pathBuilder()
                .addPath(
                        new BezierLine(scorePose, center)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), center.getHeading())
                .build();

        Command goScore_1 = follow(follower, score1);
        Command toTake_1 = follow(follower, take1);
        Command toTake_2 = follow(follower, take2);
        Command goScore_2 = follow(follower, score2);
        Command goCenter = follow(follower, center72);

        Command sequence = sequential(
                goScore_1,
                toTake_1,
                goScore_1,
                toTake_2,
                goScore_2,
                goCenter

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
