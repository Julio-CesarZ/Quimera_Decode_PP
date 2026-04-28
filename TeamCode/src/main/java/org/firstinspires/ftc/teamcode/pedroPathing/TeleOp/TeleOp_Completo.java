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
    boolean intervalo_x = false;
    boolean intervalo_RT = false;
    boolean intervalo_bumper = false;
    boolean intakeF = false;
    boolean reverse = false;
    boolean lF = false;
    // ^^ booleans para as funções de intervalo do robô ^^
    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private double shotP = 0.6;
    private double intakeP = 0.6;
    private int change = 0;
    private String changeM = "Movimentação";
    // ^^ implementando motores e outros componentes do robô ^^

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

            telemetry.addData("Troca de poder atual", changeM);
            telemetry.addData("Chassi Power", slowModeMultiplier);
            telemetry.addData("Intake Power", intakeP);
            telemetry.addData("Shot Power", shotP);

            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            ); else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true
            );

            if(gamepad1.a && !intakeF && !intervalo_a) {
                intake.setPower(intakeP);
                intakeF = !intakeF;
            } else if(gamepad1.a && intakeF && !intervalo_a) {
                intake.setPower(0);
                intakeF = !intakeF;
            }
            intervalo_a = gamepad1.a;

            if(gamepad1.b && !reverse && !intervalo_b) {
                intake.setPower(-intakeP);
                l_right.setPower(-shotP);
                l_left.setPower(-shotP);
                reverse = !reverse;
            } else if(gamepad1.b && reverse && !intervalo_b) {
                intake.setPower(0);
                l_right.setPower(0);
                l_left.setPower(0);
                reverse = !reverse;
            }
            intervalo_b = gamepad1.b;

            if(gamepad1.right_trigger > 0.3 && !lF && !intervalo_RT) {
                l_right.setPower(shotP);
                l_left.setPower(shotP);
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

            if(gamepad1.x && change == 0 && !intervalo_x) {
                change = 1;
                changeM = "Intake";
            } else if(gamepad1.x && change == 1 && !intervalo_x) {
                change = 2;
                changeM = "Lançador";
            } else if(gamepad1.x && change == 2 && !intervalo_x) {
                change = 0;
                changeM = "Movimentação";
            }
            intervalo_x = gamepad1.x;

            if(change == 0) {
                if(gamepad1.right_bumper && !intervalo_bumper) {
                    slowModeMultiplier += 0.1;
                } else if(gamepad1.left_bumper && !intervalo_bumper) {
                    slowModeMultiplier -= 0.1;
                }
                slowModeMultiplier = Range.clip(slowModeMultiplier, 0.0, 1.0);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            } else if(change == 1) {
                if(gamepad1.right_bumper && !intervalo_bumper) {
                    intakeP += 0.1;
                } else if(gamepad1.left_bumper && !intervalo_bumper) {
                    intakeP -= 0.1;
                }
                intakeP = Range.clip(intakeP, 0.0, 1.0);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            } else if(change == 2) {
                if(gamepad1.right_bumper && !intervalo_bumper) {
                    shotP += 0.1;
                } else if(gamepad1.left_bumper && !intervalo_bumper) {
                    shotP -= 0.1;
                }
                shotP = Range.clip(shotP, 0.0, 1.0);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            }

            if(intakeF) {
                intake.setPower(intakeP);
            }
            if(reverse) {
                intake.setPower(-intakeP);
                l_right.setPower(-shotP);
                l_left.setPower(-shotP);
            }
            if(lF) {
                l_right.setPower(shotP);
                l_left.setPower(shotP);
            }

            telemetry.update();
        }


    }





}