package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

@TeleOp (name = "TeleOp / 2 player", group = "TeleOp")
public class TeleOp_Completo_2 extends LinearOpMode {

    boolean intervalo_a = false;
    boolean intervalo_b = false;
    boolean intervalo_y = false;
    boolean intervalo_x = false;
    boolean intervalo_RT = false;
    boolean intervalo_LT = false;
    boolean intervalo_bumper = false;
    boolean intervalo_bpad_up = false;
    boolean intervalo_bpad_right = false;
    boolean intervalo_bpad_left = false;
    boolean intervalo_bpad_down = false;
    boolean intakeF = false;
    boolean reverse = false;
    boolean reverseL = false;
    boolean lF = false;
    boolean targetVisible;
    boolean telemetria = false;
    private double slowModeMultiplier = 0.5;
    private double shotP = 2000;
    private double intakeP = 1;
    private double towerP = 0.5;
    private double kP = 0.08;
    private double limitPL = 0.8;
    private int change = 0;
    private int target = 0;
    private double tick_intervalo = 0;
    private double intervalo = 200;
    private double position = 0.5;
    ElapsedTime elapsedIntervalo = new ElapsedTime();
    private String changeM = "Movimentação";
    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(0, 0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();

        follower.startTeleopDrive();

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx l_right = hardwareMap.get(DcMotorEx.class, "l_right");
        DcMotorEx l_left = hardwareMap.get(DcMotorEx.class, "l_left");
        DcMotorEx tower = hardwareMap.get(DcMotorEx.class, "tower");

        Servo s1 = hardwareMap.get(Servo.class, "s1");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        l_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotor.Direction.FORWARD);
        l_right.setDirection(DcMotor.Direction.REVERSE);
        l_left.setDirection(DcMotor.Direction.FORWARD);

        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setTargetPosition(0);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(50);
        limelight.start();
        limelight.pipelineSwitch(1);

        s1.setPosition(position);

        elapsedIntervalo.reset();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            targetVisible = (result != null && result.isValid());

            follower.update();

            follower.setTeleOpDrive(
                    -gamepad2.left_stick_y * slowModeMultiplier,
                    -gamepad2.left_stick_x * slowModeMultiplier,
                    -gamepad2.right_stick_x * slowModeMultiplier,
                    true
            );

            if(gamepad1.y) {
                position = 0.1;
                s1.setPosition(position);
            } else {
                position = 0.4;
                s1.setPosition(position);
            }
            intervalo_y = gamepad1.y;

            if (gamepad1.a && !intakeF && !intervalo_a) {
                intake.setPower(intakeP);
                intakeF = !intakeF;
            } else if (gamepad1.a && intakeF && !intervalo_a) {
                intake.setPower(0);
                intakeF = !intakeF;
            }
            intervalo_a = gamepad1.a;

            if (gamepad1.b && !reverse && !intervalo_b) {
                intake.setPower(-intakeP * 0.5);
                l_right.setVelocity(-shotP * 0.8);
                l_left.setVelocity(-shotP * 0.8);
                reverse = true;
                lF = false;
            } else if (gamepad1.b && reverse && !intervalo_b) {
                intake.setPower(0);
                l_right.setVelocity(0);
                l_left.setVelocity(0);
                reverse = false;
            }
            intervalo_b = gamepad1.b;

            if (gamepad1.right_trigger > 0.3 && !lF && !intervalo_RT) {
                l_right.setVelocity(shotP);
                l_left.setVelocity(shotP);
                lF = true;
                reverse = false;
            } else if (gamepad1.right_trigger > 0.3 && lF && !intervalo_RT) {
                l_right.setVelocity(0);
                l_left.setVelocity(0);
                lF = false;
            }
            intervalo_RT = gamepad1.right_trigger > 0.3;


            if (gamepad1.left_trigger > 0.3 && !intervalo_LT) {
                telemetria = !telemetria;
            }
            intervalo_LT = gamepad1.left_trigger > 0.3;

            /*
            if (gamepad1.dpad_up && !intervalo_bpad_up) {

            }
            intervalo_bpad_up = gamepad1.dpad_up;

            if (gamepad1.dpad_right && !intervalo_bpad_right) {

            }
            intervalo_bpad_right = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !intervalo_bpad_left) {

            }
            intervalo_bpad_left = gamepad1.dpad_left;
             */

            int limiteRotativo = 2000;

            if (!targetVisible) {

                int novoTarget = target;

                if (gamepad1.dpad_left && target > -limiteRotativo && elapsedIntervalo.milliseconds() >= tick_intervalo) {
                    novoTarget -= 8;
                    tick_intervalo = elapsedIntervalo.milliseconds() + intervalo;
                } else if (gamepad1.dpad_right && target < limiteRotativo && elapsedIntervalo.milliseconds() >= tick_intervalo) {
                    novoTarget += 8;
                    tick_intervalo = elapsedIntervalo.milliseconds() + intervalo;
                } else if (gamepad1.dpad_down && !intervalo_bpad_down) {
                    novoTarget = 0;
                    encoder(tower, target, towerP);
                }
                intervalo_bpad_down = gamepad1.dpad_down;

                if (Math.abs(novoTarget - tower.getCurrentPosition()) > 3) {
                    target = novoTarget;
                    encoder(tower, target, towerP);
                }

            } else {
                double tx = result.getTx();
                double ta = result.getTa();

                if (Math.abs(tx) > 1.0 && ta <= 1.5) {

                    target = tower.getCurrentPosition() + (int)(tx * 3); // menor -> sensibilidade maior

                    target = Math.max(-limiteRotativo, Math.min(limiteRotativo, target));

                    double power = tx * kP;
                    power = Math.max(-limitPL, Math.min(limitPL, power));

                    encoder(tower, target, Math.abs(power));
                }
            }

            if (gamepad1.x && change == 0 && !intervalo_x) {
                change = 1;
                changeM = "Intake";
            } else if (gamepad1.x && change == 1 && !intervalo_x) {
                change = 2;
                changeM = "Lançador";
            } else if(gamepad1.x && change == 2 && !intervalo_x) {
                change = 3;
                changeM = "Torre";
            } else if (gamepad1.x && change == 3 && !intervalo_x) {
                change = 0;
                changeM = "Movimentação";
            }
            intervalo_x = gamepad1.x;

            if (change == 0) {
                if (gamepad1.right_bumper && !intervalo_bumper) {
                    slowModeMultiplier += 0.05;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    slowModeMultiplier -= 0.05;
                }
                slowModeMultiplier = Range.clip(slowModeMultiplier, 0.0, 1.0);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            } else if (change == 1) {
                if (gamepad1.right_bumper && !intervalo_bumper) {
                    intakeP += 0.05;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    intakeP -= 0.05;
                }
                intakeP = Range.clip(intakeP, 0.0, 1.0);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            } else if (change == 2) {
                if (gamepad1.right_bumper && !intervalo_bumper) {
                    shotP += 100;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    shotP -= 100;
                }
                shotP = Range.clip(shotP, 0, 2700); //Utilizando 90% -> ticks por revolução - 28; RPM máximo - 6000

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            } else if (change == 3) {
                if (gamepad1.right_bumper && !intervalo_bumper) {
                    towerP += 0.05;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    towerP -= 0.05;
                }
                towerP = Range.clip(towerP, 0.0, 1.0);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            }

            if(telemetria) {
                telemetry.addLine("Telemetria Ativada");
                telemetry.addLine();
                telemetry.addLine("Valores de Potência");
                telemetry.addLine();
                telemetry.addData("Troca de poder atual", changeM);
                telemetry.addData("Chassi Power", slowModeMultiplier);
                telemetry.addData("Intake Power", intakeP);
                telemetry.addData("Shot Power", shotP);
                telemetry.addData("Tower Power", towerP);
                telemetry.addData("Servo", position);
                telemetry.addLine();
                telemetry.addLine("Valores de Posição");
                telemetry.addLine();
                telemetry.addData("X", follower.getPose().getX()).addData("Y", follower.getPose().getY()).addData("Heading", follower.getPose().getHeading());
                telemetry.addLine();
                telemetry.addData("Posição Atual", tower.getCurrentPosition()).addData("Alvo Encoder", target);
                telemetry.addLine();
                telemetry.addLine("Lógica da câmera");
                telemetry.addLine();
                if (targetVisible) {
                    telemetry.addData("Alvo Detectado", "Sim");
                    telemetry.addData("TX (Graus)", result.getTx()).addData("TY (Graus)", result.getTy()).addData("TA (Área)", result.getTa());
                } else {
                    telemetry.addData("Alvo Detectado", "Não");
                }

                telemetry.update();
            } else {
                telemetry.addLine("Telemetria Desativada");
                telemetry.update();
            }
        }
    }

    private void encoder(DcMotor motor, int novoAlvo, double power) {
        motor.setTargetPosition(novoAlvo);
        motor.setPower(power);
    }
}

