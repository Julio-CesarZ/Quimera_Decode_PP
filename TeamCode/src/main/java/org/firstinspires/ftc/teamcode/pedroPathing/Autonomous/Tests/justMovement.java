package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Movement", group = "Test")
public class justMovement extends LinearOpMode {

    Follower follower;
    private final Pose startPose = new Pose(108.1, 133.91, 0);
    private final Pose scorePose = new Pose(93.74, 85.37, 0);
    PathChain score1;

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

        Command goScore_1 = follow(follower, score1);

        Command sequence = sequential(
                goScore_1
        );

        waitForStart();

        schedule(sequence);

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
        }
    }
}
