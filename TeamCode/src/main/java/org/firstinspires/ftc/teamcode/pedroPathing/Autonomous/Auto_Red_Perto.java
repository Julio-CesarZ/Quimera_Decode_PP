package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "RED - Auto de Perto", group = "Auto")
public class Auto_Red_Perto extends OpMode {
    private Follower follower;

    private final Pose startPose = new Pose(109.31, 133.91, 0);
    private final Pose scorePose = new Pose(72, 72, Math.toRadians(0));
    private final Pose coletaPose = new Pose(23.6, 24.38, Math.toRadians(0));
    private PathChain scorePreload, score1, score2;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, coletaPose))
                .setConstantHeadingInterpolation(0)
                .build();
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(coletaPose, scorePose))
                .setConstantHeadingInterpolation(0)
                .build();

    }
    public Command autoRoutine() {
        return sequential(
                follow(follower, scorePreload),
                follow(follower, score1, true),
                follow(follower, score2, true)
        );
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        // Feedback to Driver Hub for debugging
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}