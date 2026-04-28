package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import android.renderscript.Int2;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Omni_Move;

import java.util.function.Supplier;


@TeleOp(name = "Second Launcher / BasedMode", group = "TeleOp")
public class Laucher11 extends LinearOpMode{
    private Omni_Move Drive;
    private Servo s1;

    //private DcMotor In1;
    //private DcMotor In2;
    //private DcMotor sh1;
    //private DcMotor sh2;
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private DcMotor ex3;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();


        follower.startTeleopDrive();

        Drive = new Omni_Move();

        DcMotorEx c1z = hardwareMap.get(DcMotorEx.class, "c1");
        DcMotorEx c2z = hardwareMap.get(DcMotorEx.class, "c2");
        DcMotorEx c3z = hardwareMap.get(DcMotorEx.class,"c3");
        DcMotorEx c4z = hardwareMap.get(DcMotorEx.class,"c4");

        Servo s1z = hardwareMap.get(Servo.class,"s1");
        Servo s2z = hardwareMap.get(Servo.class,"s2");

        c1z.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        c2z.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        c3z.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        c1z.setDirection(DcMotor.Direction.REVERSE);
        c2z.setDirection(DcMotor.Direction.REVERSE);
        c3z.setDirection(DcMotor.Direction.REVERSE);

        boolean InT1 = false;


        boolean shotZ = false;
        boolean BInT1 = false;
        boolean shotB = false;

        boolean shot1 = false;
        boolean shot2 = false;

        boolean shI = false;
        boolean shI2 = false;

        double sp = 0.7; // sp = shotPower

        boolean Servo = false;

        boolean coreL = false;
        boolean retornoL = false;

        String ativar = "";
        String core = "";

        double currentPowerBase = 0.5;

        double IntakePower = 1;

        waitForStart();
        while (opModeIsActive()) {

            //Call this once per loop
            follower.update();
            telemetryM.update();

            if (!automatedDrive) {
                //Make the last parameter false for field-centric
                //In case the drivers want to use a "slowMode" you can scale the vectors

                //This is the normal version to use in the TeleOp
                if (!slowMode) follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.right_stick_x,
                        -gamepad1.left_stick_x,
                        true // Robot Centric
                );

                    //This is how it looks with slowMode on
                else follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * slowModeMultiplier,
                        -gamepad1.left_stick_x * slowModeMultiplier,
                        -gamepad1.right_stick_x * slowModeMultiplier,
                        true // Robot Centric
                );
            }

            telemetry.addData("Intake1: ", ativar);
            telemetry.addData("Intake1: ", ativar);

            InT1 = gamepad1.right_trigger > 0.3;

            if(InT1) {
                c1z.setPower(1);
                ativar = "Ligado";
            } else {
                c1z.setPower(0);
                ativar = "Desligado";
            }

            BInT1 = gamepad1.left_trigger > 0.3;

            if(BInT1) {
                c1z.setPower(1);
                ativar = "Ligado";
            } else {
                c1z.setPower(0);
                ativar = "Desligado";
            }

            if(gamepad1.y && !shotZ) {
                c2z.setPower(sp);
                c3z.setPower(sp);
                shotZ = true;
                sleep(100);
            } else if(gamepad1.y && shotZ) {
                c2z.setPower(0);
                c3z.setPower(0);
                shotZ = false;
                sleep(100);
            }

            if(gamepad1.x && !shotB) {
                c2z.setPower(-sp);
                c3z.setPower(-sp);
                shotB = true;
                sleep(100);
            } else if(gamepad1.x && shotB) {
                c2z.setPower(0);
                c3z.setPower(0);
                shotB = false;
                sleep(100);
            }

            if(gamepad1.a && !shot1) {
                c2z.setPower(-1);
                shot1 = true;
                sleep(100);
            } else if(gamepad1.a && shot1) {
                c2z.setPower(0);
                shot1 = false;
                sleep(100);
            }

            if(gamepad1.b && !shot2) {
                c3z.setPower(-1);
                shot2 = true;
                sleep(100);
            } else if(gamepad1.b && shot2) {
                c3z.setPower(0);
                shot2 = false;
                sleep(100);
            }


            telemetry.update();
        }
    }


}

