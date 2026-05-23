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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    boolean telemetria = true;
    boolean mode_2 = false;
    private double slowModeMultiplier = 0.7;
    private double shotP = 2400;
    private double velocityShot = 0;
    private double intakeP = 1;
    private double towerP = 0.5;
    final double kP = 0.08;
    private int change = 0;
    private int target = 0;
    private double tick_intervalo = 0;
    private double position = 0.3;
    private double velocityAtual = 0;
    ElapsedTime elapsedIntervalo = new ElapsedTime();
    ElapsedTime elapsedIntervaloServo = new ElapsedTime();
    ElapsedTime elapsedSuavizador = new ElapsedTime();
    private String changeM = "Movimentação";
    private Follower follower;
    public static Pose startingPose;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        Supplier<PathChain> pathChain = () -> follower.pathBuilder()
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

        PIDFCoefficients coefficientsRightMotor = l_right.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients coefficientsLeftMotor = l_left.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        l_right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficientsRightMotor.p,
                coefficientsRightMotor.i,
                coefficientsRightMotor.d,
                coefficientsRightMotor.f * 1.25
        ));

        l_left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficientsLeftMotor.p,
                coefficientsLeftMotor.i,
                coefficientsLeftMotor.d,
                coefficientsLeftMotor.f * 1.25
        ));

        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setTargetPosition(0);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(50);
        limelight.start();
        limelight.pipelineSwitch(0);

        s1.setPosition(position);

        elapsedIntervalo.reset();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad2.id != -1) {
                mode_2 = true;
            } else {
                mode_2 = false;
            }

            LLResult result = limelight.getLatestResult();
            targetVisible = (result != null && result.isValid());

            follower.update();

            double forward;
            double strafe;
            double turn;
            if (mode_2) {
                forward = Math.pow(-gamepad2.left_stick_y * slowModeMultiplier, 3);
                strafe = Math.pow(-gamepad2.left_stick_x * slowModeMultiplier, 3);
                turn = Math.pow(-gamepad2.right_stick_x * slowModeMultiplier, 3);
            } else{
                forward = Math.pow(-gamepad1.left_stick_y * slowModeMultiplier, 3);
                strafe = Math.pow(-gamepad1.left_stick_x * slowModeMultiplier, 3);
                turn = Math.pow(-gamepad1.right_stick_x * slowModeMultiplier, 3);
            }

            follower.setTeleOpDrive(forward, strafe, turn, true);

            double deltaTime = elapsedSuavizador.seconds();
            elapsedSuavizador.reset();

            if(lF || gamepad1.left_trigger > 0.3) {
                if(elapsedIntervaloServo.milliseconds() > 500) {
                    position = 0.1;
                    s1.setPosition(position);
                }
            } else {
                position = 0.3;
                s1.setPosition(position);
            }

            intervalo_LT = gamepad1.left_trigger > 0.3;

            if (gamepad1.a && !intakeF && !intervalo_a) {
                intake.setPower(intakeP);
                intakeF = !intakeF;
            } else if (gamepad1.a && intakeF && !intervalo_a) {
                intake.setPower(0);
                intakeF = !intakeF;
            }
            intervalo_a = gamepad1.a;

            if(gamepad1.b && !intakeF && !lF) {
                intake.setPower(-intakeP * 0.5);
                velocityShot = -shotP * 0.8;
                reverse = true;
            } else if(!gamepad1.b && reverse) {
                intake.setPower(0);
                velocityShot = 0;
                reverse = false;
            }
            /*
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
             */
            intervalo_b = gamepad1.b;

            if (gamepad1.right_trigger > 0.3 && !lF && !intervalo_RT) {
                velocityShot = shotP;
                elapsedIntervaloServo.reset();
                lF = true;
                reverse = false;
            } else if (gamepad1.right_trigger > 0.3 && lF && !intervalo_RT) {
                velocityShot = 0;
                lF = false;
            }
            intervalo_RT = gamepad1.right_trigger > 0.3;

            double erroGeral = velocityShot - velocityAtual;

            double maxAccelerationPerS = 500;
            double maxChange = maxAccelerationPerS * deltaTime;

            if(Math.abs(erroGeral) <= maxChange) {
                velocityAtual = velocityShot;
            } else {
                velocityAtual += Math.signum(erroGeral) * maxChange;
            }

            l_right.setVelocity(velocityAtual);
            l_left.setVelocity(velocityAtual);

            if (gamepad1.y && !intervalo_y) {
                telemetria = !telemetria;
            }
            intervalo_y = gamepad1.y;

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

                double intervalo = 200;
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
                    double limitPL = 0.8;
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
                shotP = Range.clip(shotP, 0, 2800); //Utilizando 100% -> ticks por revolução - 28; RPM máximo - 6000
                if(lF) {
                    velocityShot = shotP;
                }

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

            if(mode_2) {
                telemetry.addLine("Modo para dois jogadores");
                telemetry.addLine("Gamepad 1 - Controle das funções. Gamepad 2 - Controle do Chassi");
            } else {
                telemetry.addLine("Modo para um jogador");
                telemetry.addLine("Gamepad 1 - Controle Geral");
            }
            telemetry.addLine();
            if(telemetria) {
                telemetry.addLine("Telemetria Ativada");
                telemetry.addLine();
                telemetry.addData("Troca de poder atual", changeM);
                telemetry.addData("Chassi Power", slowModeMultiplier);
                telemetry.addData("Intake Power", intakeP);
                telemetry.addData("Variável ShotP", shotP);
                telemetry.addData("Valor de Referência do Motores", velocityAtual);
                telemetry.addData("Velocidade do Motor da Direita: ", l_right.getVelocity());
                telemetry.addData("Velocidade do Motor da Esquerda: ", l_left.getVelocity());
                telemetry.addData("Tower Power", towerP);
                telemetry.addData("Servo", position);
                telemetry.addLine();
                telemetry.addData("X", follower.getPose().getX()).addData("Y", follower.getPose().getY()).addData("Heading", follower.getPose().getHeading());
                telemetry.addLine();
                telemetry.addData("Posição da Torre", tower.getCurrentPosition()).addData("Alvo da Torre", target);
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
