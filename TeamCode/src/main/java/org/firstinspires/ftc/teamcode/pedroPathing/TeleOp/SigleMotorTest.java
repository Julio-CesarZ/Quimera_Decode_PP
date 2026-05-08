package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp (name = "TeleOp / Teste de Motor único", group = "TeleOp")
public class SigleMotorTest extends LinearOpMode {

    // ^^ booleans para as funções de intervalo do robô ^^
    boolean intervalo_bumper = false;
    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private double slowModeMultiplier = 0.8;

    private String changeM = "Movimentação";
    // ^^ implementando motores e outros componentes do robô ^^
    private ElapsedTime Rtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(0, 0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();


        follower.startTeleopDrive();

        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf");         // .rightFrontMotorName("rf") //0
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rr");       // .rightRearMotorName("rr") //2
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf");       // .leftRearMotorName("lr") //3
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, "lr");       // .leftFrontMotorName("lf") //1


        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rr.setDirection(DcMotor.Direction.FORWARD);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
        lf.setDirection(DcMotor.Direction.FORWARD);
                //.leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                //.leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                //.rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                //.rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            // Telemetria para mostrar a potência dos Motores e Intakes
            telemetry.addLine("Testar Motores Individualmente");
            telemetry.addLine();
            telemetry.addData("Troca de poder atual", changeM);
            telemetry.addData("Chassi Power", slowModeMultiplier);
            telemetry.addLine();
            telemetry.addLine("Valores de Posição");
            telemetry.addLine("A = Right Rear");
            telemetry.addLine("B = left Rear");
            telemetry.addLine("Y = Right Front");
            telemetry.addLine("X = Left Front");
            telemetry.addLine();
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.addLine();
            telemetry.addData("Elapsed Time atual", Rtime.seconds());

            telemetry.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true
            );

            // Gamepads do intakes e lançadores

                if (gamepad1.right_bumper && !intervalo_bumper) {
                    slowModeMultiplier += 0.05;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    slowModeMultiplier -= 0.05;
                }
                slowModeMultiplier = Range.clip(slowModeMultiplier, 0.0, 1.0);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;

            if (gamepad1.a) {
                rr.setPower(1);
            } else if (!gamepad1.a) {
                rr.setPower(0);

            }
            if (gamepad1.y) {
                rf.setPower(1);
            } else if (!gamepad1.y) {
                rf.setPower(0);

            }

            if (gamepad1.x) {
                lf.setPower(1);
            } else if (!gamepad1.x) {
                lf.setPower(0);

            }

            if (gamepad1.b) {
                lr.setPower(1);
            } else if (!gamepad1.b) {
                lr.setPower(0);

            }

        }
    }
}

