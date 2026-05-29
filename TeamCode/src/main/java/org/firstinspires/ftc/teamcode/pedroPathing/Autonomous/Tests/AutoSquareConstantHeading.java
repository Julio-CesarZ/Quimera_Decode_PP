package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.Tests; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Teste de Quadrado Constante", group = "Auto")
public class AutoSquareConstantHeading extends OpMode {
    private Follower follower;

    private final Pose startPose = new Pose(72, 72, Math.toRadians(270));
    private final Pose pose1 = new Pose(72, 23, Math.toRadians(270));
    private final Pose pose2 = new Pose(23, 23, Math.toRadians(270));
    private final Pose pose3 = new Pose(23, 72, Math.toRadians(270));
    private final Pose pose4 = new Pose(72, 72, Math.toRadians(270));
    private PathChain scorePreload;

    public void buildPaths() {

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(
                                new Pose(72, 72),
                                new Pose(72, 23)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(72, 23),
                                new Pose(23, 23)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(
                        new BezierLine(
                                new Pose(23, 23),
                                new Pose(23, 72)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(
                        new BezierLine(
                                new Pose(23, 72),
                                new Pose(72, 72)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

    }
    public Command autoRoutine() {
        return sequential(
                follow(follower, scorePreload)
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