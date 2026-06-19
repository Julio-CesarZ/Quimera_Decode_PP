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

@Autonomous(name = "leave", group = "Auto")
public class leave extends LinearOpMode {

    Follower follower;

    private final Pose startPose = new Pose(54.83, 8.27, Math.toRadians(180));
    private final Pose outPose = new Pose(27, 8.27, Math.toRadians(180));
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


        outLine = follower.pathBuilder()
                .addPath(new BezierLine(startPose, outPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        Command goScore_F = follow(follower, scoreF);
        Command toTake_3 = follow(follower, take3);
        Command toTake_Canto = follow(follower, takeCanto);
        Command toTakeX = follow(follower, takeX);
        Command goScore_3 = follow(follower, score3);
        Command goScoreX = follow(follower, scoreX);
        Command toTake_Cycle = follow(follower, cycleTake);
        Command goScore_Cycle = follow(follower, cycleScore);
        Command goOut = follow(follower, outLine);

        Command onIntake = instant(() -> intake.setPower(1));
        Command offIntake = instant(() -> intake.setPower(0));

        Command abrirTrava = instant(() -> s1.setPosition(0.52));
        Command fecharTrava = instant(() -> s1.setPosition(0.63));

        Command mirar = instant(() -> encoder(tower, 420, 0.5));
        Command zerar = instant(() -> encoder(tower, 0, 0.5));

        Command onShotR = instant(() -> l_right.setVelocity(2200));
        Command onShotL = instant(() -> l_left.setVelocity(2200));
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
                        goOut

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
