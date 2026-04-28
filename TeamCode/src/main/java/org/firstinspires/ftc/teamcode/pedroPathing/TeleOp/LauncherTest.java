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

@Disabled
@TeleOp(name = "First Launcher / TestMode", group = "TeleOp")
public class LauncherTest extends LinearOpMode{
    private Omni_Move Drive;
    private Servo s1;
    private DcMotor In1;
    private DcMotor In2;
    private DcMotor sh1;
    private DcMotor sh2;
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

        In1 = hardwareMap.get(DcMotor.class,"c1");
        sh1 = hardwareMap.get(DcMotor.class,"c2");
        sh2 = hardwareMap.get(DcMotor.class, "c3");
        ex3 = hardwareMap.get(DcMotor.class, "c4");
        s1 = hardwareMap.get(Servo.class, "s1");

        In1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        In2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sh1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        In1.setDirection(DcMotor.Direction.REVERSE);
        In2.setDirection(DcMotor.Direction.REVERSE);
        sh1.setDirection(DcMotor.Direction.REVERSE);

        boolean InT1 = false;
        boolean InT2 = false;
        boolean BInT1 = false;
        boolean shI = false;
        boolean shI2 = false;
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

            //Automated PathFollowing
            /*if (gamepad1.aWasPressed()) {
                follower.followPath(pathChain.get());
                automatedDrive = true;
            }

            //Stop automated following if the follower is done
            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }

            //Slow Mode
            if (gamepad1.rightBumperWasPressed()) {
                slowMode = !slowMode;
            }

            //Optional way to change slow mode strength
            if (gamepad1.xWasPressed()) {
                slowModeMultiplier += 0.25;
            }

            //Optional way to change slow mode strength
            if (gamepad2.yWasPressed()) {
                slowModeMultiplier -= 0.25;
            }

            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());
            telemetryM.debug("automatedDrive", automatedDrive);
        }
        */
            /*double y = -gamepad1.left_stick_y; // frente/trás
            double x = gamepad1.left_stick_x;  // lateral
            double rot = gamepad1.right_stick_x; // rotação */

        telemetry.addData("Intake1: ", ativar);

        InT1 = gamepad1.right_trigger > 0.3;

        if(InT1) {
            In1.setPower(1);
            ativar = "Ligado";
        } else {
            In1.setPower(0);
            ativar = "Desligado";
        }

        BInT1 = gamepad1.left_trigger > 0.3;

        if(BInT1) {
            In1.setPower(-1);
            ativar = "Ligado";
        } else {
            In1.setPower(0);
            ativar = "Desligado";
        }


        telemetry.update();
    }
}


}

