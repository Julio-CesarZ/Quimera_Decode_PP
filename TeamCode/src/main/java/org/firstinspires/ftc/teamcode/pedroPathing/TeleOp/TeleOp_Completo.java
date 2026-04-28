package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp (name = "TeleOp", group = "TeleOp")
public class TeleOp_Completo extends LinearOpMode {

    boolean intervalo_a = false;
    boolean intervalo_b = false;
    boolean intervalo_y = false;
    boolean intervalo_RT = false;
    boolean intervalo_bumper = false;
    boolean intakeF = false;
    boolean reverse = false;
    boolean lF = false;
    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();


        follower.startTeleopDrive();

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx l_right = hardwareMap.get(DcMotorEx.class, "l_right");
        DcMotorEx l_left = hardwareMap.get(DcMotorEx.class,"l_left");

        Servo s1 = hardwareMap.get(Servo.class,"s1");
        Servo s2 = hardwareMap.get(Servo.class,"s2");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotor.Direction.REVERSE);
        l_right.setDirection(DcMotor.Direction.REVERSE);
        l_left.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            telemetry.addData("Slow Mode", slowModeMultiplier);

            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            ); else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );

            if(gamepad1.a && !intakeF && !intervalo_a) {
                intake.setPower(1);
                intakeF = !intakeF;
            } else if(gamepad1.a && intakeF && !intervalo_a) {
                intake.setPower(0);
                intakeF = !intakeF;
            }
            intervalo_a = gamepad1.a;

            if(gamepad1.b && !reverse && !intervalo_b) {
                intake.setPower(-1);
                l_right.setPower(-1);
                l_left.setPower(-1);
                reverse = !reverse;
            } else if(gamepad1.b && reverse && !intervalo_b) {
                intake.setPower(0);
                l_right.setPower(0);
                l_left.setPower(0);
                reverse = !reverse;
            }
            intervalo_b = gamepad1.b;

            if(gamepad1.right_trigger > 0.3 && !lF && !intervalo_RT) {
                l_right.setPower(1);
                l_left.setPower(1);
                lF = !lF;
            } else if(gamepad1.right_trigger > 0.3 && lF && !intervalo_RT) {
                l_right.setPower(0);
                l_left.setPower(0);
                lF = !lF;
            }
            intervalo_RT = gamepad1.right_trigger > 0.3;

            if(gamepad1.y && !slowMode && !intervalo_y) {
                slowMode = !slowMode;
            } else if(gamepad1.y && slowMode && !intervalo_y) {
                slowMode = !slowMode;
            }
            intervalo_y = gamepad1.y;

            if(gamepad1.right_bumper && !intervalo_bumper) {
                slowModeMultiplier += 0.1;
            } else if(gamepad1.left_bumper && !intervalo_bumper) {
                slowModeMultiplier -= 0.1;
            }
            slowModeMultiplier = Range.clip(slowModeMultiplier, 0.0, 1.0);

            intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;

            telemetry.update();
        }


    }





}