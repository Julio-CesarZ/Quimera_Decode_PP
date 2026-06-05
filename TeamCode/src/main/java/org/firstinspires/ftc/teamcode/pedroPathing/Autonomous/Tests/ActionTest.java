package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.Tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Action Test", group = "Test")
public class ActionTest extends LinearOpMode {

    Follower follower;
    private final Pose startPose = new Pose(108.1, 133.91, 0);
    private final Pose scorePose = new Pose(93.74, 85.37, 0);
    PathChain scorePreload;

    @Override
    public void runOpMode() {
        //Since the scheduler is static, we need to reset it before each OpMode
        //so commands don't carry over from one OpMode to the next
        Scheduler.reset();

        // Initialize hardware
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo claw = hardwareMap.get(Servo.class, "s1");

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        // Define commands
        Command goScore = follow(follower, scorePreload);

       /*
       Command inIntake = Command.build()
               .setExecute(() -> intake.setPower(0.5))
               .setDone(() -> intake.getCurrentPosition() > 1000)
               .setEnd(endCondition -> intake.setPower(0))
               .requiring(intake);
        */

        Command inIntake = instant(() -> intake.setPower(0.5));

        Command openClaw = instant(() -> claw.setPosition(0.6));

        // Compose: raise the arm, wait 200ms, then open the claw
        Command sequence = sequential(
                goScore,
                waitMs(1000),
                inIntake,
                waitMs(200),
                openClaw
        );

        waitForStart();

        // Schedule the sequence when the OpMode starts
        schedule(sequence);

        while (opModeIsActive()) {
            // Run the scheduler each loop
            follower.update();
            Scheduler.execute();
        }
    }
}
