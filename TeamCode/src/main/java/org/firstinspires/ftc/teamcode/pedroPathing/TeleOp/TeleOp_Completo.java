package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import static org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.Auto_Red_Perto_SOLO.center;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp_Completo extends LinearOpMode {

    boolean intervalo_a = false;
    boolean intervalo_x = false;
    boolean intervalo_RT = false;
    boolean intervalo_LT = false;
    boolean intervalo_y = false;
    boolean intervalo_bumper = false;
    boolean intervalo_dpad_up = false;
    boolean intervalo_stick = false;
    boolean intakeF = false;
    boolean intakeSS = false;
    boolean reverse = false;
    boolean reverseL = false;
    boolean lF = false;
    boolean targetVisible;
    boolean telemetria = true;
    boolean mode_2 = false;
    boolean modo_TorreA = true;
    boolean modo_ShotPA = true;
    boolean camera = true;
    boolean suavizador = false;

    private double velocityMultipleir = 0.9;
    private double velocityShot = 0;
    private double tx = 0;
    private double positionS = 0.63;
    private double velocityAtual = 0;
    private double lastStamp = 0;
    final double kP = 0.05;
    final double towerP = 0.5;
    private int shotP = 1450;
    private int change = 0;
    private int target = 0;
    final int maxChangeTick = 10;
    final int limiteRotativo = 750;
    final int intakeP = 1;

    ElapsedTime elapsedIntervaloServo = new ElapsedTime();
    ElapsedTime elapsedSuavizador = new ElapsedTime();
    ElapsedTime elapsedintervaloL = new ElapsedTime();
    ElapsedTime elapsedintervaloIntakeSS = new ElapsedTime();
    ElapsedTime elapsedIntervaloC = new ElapsedTime();

    private String changeM = "Movimentação";
    private final Pose startingPoseTeleop = new Pose(110.47, 132.68, 0);
    private final Pose centerGol = new Pose(144, 144);

    @Override
    public void runOpMode() throws InterruptedException {

        Follower follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(center == null ? startingPoseTeleop : center);
        follower.setStartingPose(startingPoseTeleop);
        follower.update();
        follower.startTeleopDrive();

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx l_right = hardwareMap.get(DcMotorEx.class, "l_right");
        DcMotorEx l_left = hardwareMap.get(DcMotorEx.class, "l_left");
        DcMotorEx tower = hardwareMap.get(DcMotorEx.class, "tower");

        Servo s1 = hardwareMap.get(Servo.class, "s1");

        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        l_right.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        l_left.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        l_right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        l_left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotorEx.Direction.REVERSE);
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

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();
        limelight.pipelineSwitch(1);

        s1.setPosition(positionS);

        elapsedintervaloL.reset();
        elapsedIntervaloServo.reset();
        elapsedSuavizador.reset();
        elapsedintervaloIntakeSS.reset();
        elapsedIntervaloC.reset();

        while (!isStarted() && !isStopRequested()) {

            mode_2 = gamepad1.getUser() != null && gamepad2.getUser() != null;

            if (mode_2) {
                telemetry.addLine("Modo para dois jogadores");
                telemetry.addLine("Gamepad 1 - Funções | Gamepad 2 - Chassi");
            } else {
                telemetry.addLine("Modo para um jogador");
                telemetry.addLine("Gamepad 1 - Controle Geral");
            }
            telemetry.addLine();

            if (gamepad1.y && !intervalo_y) {
                camera = !camera;
            }
            intervalo_y = gamepad1.y;

            telemetry.addLine(camera ? "Câmera Ativada" : "Câmera Desativada");
            telemetry.addLine("Pressione [Y] para alternar a câmera\n");

            if (gamepad1.left_trigger > 0.3 && !intervalo_LT) {
                modo_TorreA = !modo_TorreA;
            }
            intervalo_LT = gamepad1.left_trigger > 0.3;

            telemetry.addLine(modo_TorreA ? "Pressione [LT] para DESATIVAR o modo Auto da Torre" : "Pressione [LT] para ATIVAR o modo Auto da Torre");
            telemetry.addLine("Change precisa ser igual a Movimentação\n");

            if (gamepad1.right_trigger > 0.3 && !intervalo_RT) {
                modo_ShotPA = !modo_ShotPA;
            }
            intervalo_RT = gamepad1.right_trigger > 0.3;

            telemetry.addLine(modo_ShotPA ? "Pressione [RT] para DESATIVAR o modo Auto do Shot" : "Pressione [RT] para ATIVAR o modo Auto do Shot");
            telemetry.addLine("Change precisa ser igual ao Lançador\n");

            if (gamepad1.right_bumper && !intervalo_bumper) {
                telemetria = !telemetria;
            }
            intervalo_bumper = gamepad1.right_bumper;

            telemetry.addLine(telemetria ? "Telemetria: Ativada" : "Telemetria: Desativada");
            telemetry.update();
        }

        while (opModeIsActive()) {
            follower.update();

            double heading = Math.toDegrees(follower.getPose().getHeading());
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();

            double xGol = centerGol.getX();
            double yGol = centerGol.getY();

            double distanciaM = Math.hypot(x - xGol, y - yGol) / 39.37;

            double ticks = (veloV0_RPM(distanciaM, 60.1, 0.311, 1.179, 0.075, 1.11) * 28) / 60;

            LLResult result = limelight.getLatestResult();
            targetVisible = (result != null && result.isValid());

            double forward, strafe, turn;
            if (mode_2) {
                forward = Math.pow(-gamepad2.left_stick_y * velocityMultipleir, 3);
                strafe = Math.pow(-gamepad2.left_stick_x * velocityMultipleir, 3);
                turn = Math.pow(-gamepad2.right_stick_x * velocityMultipleir, 3);
            } else {
                forward = Math.pow(-gamepad1.left_stick_y * velocityMultipleir, 3);
                strafe = Math.pow(-gamepad1.left_stick_x * velocityMultipleir, 3);
                turn = Math.pow(-gamepad1.right_stick_x * velocityMultipleir, 3);
            }

            follower.setTeleOpDrive(forward, strafe, turn, true);

            if (lF || gamepad1.left_trigger > 0.3) {
                if (elapsedIntervaloServo.seconds() > 0.1 && velocityAtual + 100 >= shotP && velocityAtual - 100 <= shotP) {
                    positionS = 0.55;
                    s1.setPosition(positionS);
                }
            } else {
                positionS = 0.63;
                s1.setPosition(positionS);
            }

            suavizarAcelleration(y);

            if (!reverse && !reverseL && elapsedintervaloL.seconds() >= 1) {
                l_right.setVelocity(velocityAtual);
                l_left.setVelocity(velocityAtual);
            }

            if (!lF) {
                if (gamepad1.y && !intakeF && !intakeSS && !reverse) {
                    l_right.setVelocity(-1000);
                    l_left.setVelocity(-1000);
                    reverseL = true;
                } else if (!gamepad1.y && reverseL) {
                    l_right.setVelocity(0);
                    l_left.setVelocity(0);
                    velocityShot = 0;
                    reverseL = false;
                    elapsedintervaloL.reset();
                }
                if (gamepad1.b && !intakeF && !lF && !intakeSS && !reverseL) {
                    intake.setPower(-intakeP);
                    l_right.setVelocity(-1000);
                    l_left.setVelocity(-1000);
                    reverse = true;
                } else if (!gamepad1.b && reverse) {
                    intake.setPower(0);
                    l_right.setVelocity(0);
                    l_left.setVelocity(0);
                    reverse = false;
                    elapsedintervaloL.reset();
                }
            }

            if (gamepad1.a && !intakeF && !intervalo_a && !intakeSS) {
                intake.setPower(intakeP);
                intakeF = true;
            } else if (gamepad1.a && intakeF && !intervalo_a) {
                intake.setPower(0);
                intakeF = false;
            }
            intervalo_a = gamepad1.a;

            double intervaloSS = 300;
            if (gamepad1.dpad_up && !intakeSS && !intervalo_dpad_up) {
                intakeSS = true;
                elapsedintervaloIntakeSS.reset();
            } else if (gamepad1.dpad_up && intakeSS && !intervalo_dpad_up) {
                intake.setPower(0);
                intakeSS = false;
                elapsedintervaloIntakeSS.reset();
            }
            intervalo_dpad_up = gamepad1.dpad_up;

            if (intakeSS) {
                if (elapsedintervaloIntakeSS.milliseconds() % (intervaloSS * 2) <= intervaloSS) {
                    intake.setPower(intakeP);
                } else {
                    intake.setPower(0);
                }
            }

            if (!targetVisible) {
                if (!lF) {
                    if (modo_TorreA) {
                        target = Range.clip(torreAuto(x, y, heading), -limiteRotativo, limiteRotativo);
                        encoder(tower, target, towerP);
                    } else {
                        torreManual(tower);
                    }
                }
            } else {
                if (camera) {
                    camera(result, y, tower);
                }
            }

            boolean sticksPressionados = gamepad1.right_stick_button && gamepad1.left_stick_button;
            if (sticksPressionados && !intervalo_stick && !lF) {
                if (change == 0) modo_TorreA = !modo_TorreA;
                if (change == 1) modo_ShotPA = !modo_ShotPA;
            }
            intervalo_stick = sticksPressionados;

            if (modo_ShotPA) {
                //shotPA(x, y);
                shotP = (int) ticks;
                if (lF) {
                    velocityAtual = shotP;
                }
            }

            if (gamepad1.x && !intervalo_x) {
                if (change == 0) {
                    change = 1;
                    changeM = "Lançador";
                } else {
                    change = 0;
                    changeM = "Movimentação";
                }
            }
            intervalo_x = gamepad1.x;

            if ((gamepad1.right_bumper || gamepad1.left_bumper) && !intervalo_bumper) {
                if (change == 0) {
                    velocityMultipleir += gamepad1.right_bumper ? 0.05 : -0.05;
                    velocityMultipleir = Range.clip(velocityMultipleir, 0.0, 1.0);
                } else if (change == 1) {
                    shotP += gamepad1.right_bumper ? 50 : -50;
                    shotP = Range.clip(shotP, 0, 2800);
                    if (lF) velocityShot = shotP;
                }
            }
            intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;

            telemetria(l_right, l_left, tower, x, y, heading);
        }
    }

    private void telemetria(DcMotorEx l_right, DcMotorEx l_left, DcMotorEx tower, double x, double y, double heading) {
        if (telemetria) {
            if (targetVisible) {
                telemetry.addLine("🟢 Telemetria  Ativada 🟢");
            } else {
                telemetry.addLine("🔴 Telemetria  Ativada 🔴");
            }
            telemetry.addLine();

            if (camera) {
                telemetry.addData("TX (Graus)", tx);
            } else {
                telemetry.addLine("Câmera Desativada");
            }
            telemetry.addLine();

            telemetry.addData("Troca de poder atual", changeM);
            telemetry.addData("Chassi Power", velocityMultipleir);
            telemetry.addData("Variável ShotP", shotP);
            telemetry.addData("Ticks/s Dir", l_right.getVelocity());
            telemetry.addData("Ticks/s Esq", l_left.getVelocity());
            telemetry.addData("Posição Torre", tower.getCurrentPosition());
            telemetry.addData("Alvo Torre", target);
            telemetry.addData("Target Torre", tower.getTargetPosition());
            telemetry.addLine();
            telemetry.addData("X", x).addData("Y", y);
            telemetry.addData("Heading", heading);
            telemetry.addLine();
            telemetry.addData("Modo Automático da Torre", modo_TorreA);
            telemetry.addData("Modo Automático de shotP", modo_ShotPA);
            telemetry.addLine();
            telemetry.update();
        } else {
            telemetry.addLine("Telemetria Desativada");
            telemetry.update();
        }
    }

    private void suavizarAcelleration(double y) {
        double deltaTime = elapsedSuavizador.seconds();
        elapsedSuavizador.reset();

        if (gamepad1.right_trigger > 0.3 && !lF && !reverse && !reverseL && !intervalo_RT) {
            velocityShot = shotP;
            elapsedIntervaloServo.reset();
            lF = true;
        } else if (gamepad1.right_trigger > 0.3 && lF && !reverse && !reverseL && !intervalo_RT) {
            velocityShot = 0;
            lF = false;
        }
        intervalo_RT = gamepad1.right_trigger > 0.3;

        double erroGeral = velocityShot - velocityAtual;
        double maxAccelerationPerS = (y > 60) ? 800 : 1500;
        double maxChange = maxAccelerationPerS * deltaTime;

        if (Math.abs(erroGeral) <= maxChange || !suavizador) {
            velocityAtual = velocityShot;
        } else {
            velocityAtual += Math.signum(erroGeral) * maxChange;
        }
    }

    private void camera(LLResult result, double y, DcMotorEx tower) {

        if (result.getTimestamp() != lastStamp) {
            lastStamp = result.getTimestamp();

            tx = result.getTx();
            int atual = tower.getCurrentPosition();

            if (Math.abs(tx) > 1) {
                int position = (y > 125) ? atual - 62 : atual;
                int alvo = position + (int) (tx * 7);
                alvo = Range.clip(alvo, -limiteRotativo, limiteRotativo);

                int delta = Range.clip(alvo - target, -maxChangeTick, maxChangeTick);
                target += delta;
                target = Range.clip(target, -limiteRotativo, limiteRotativo);

                double power = Range.clip(tx * kP, 0.1, 1.0);

                encoder(tower, target, Math.abs(power));
            } else {
                encoder(tower, atual, 0.3);
            }
        }
    }

    private void shotPA(double x, double y) {
        if (82 <= x && x < 104 && 98 <= y && y <= 135) shotP = 1350;
        else if (46 <= x && x < 82 && 66 <= y && y <= 135) shotP = 1600;
        else if (32 <= x && x < 46 && 100 <= y && y <= 135) shotP = 1700;
        else if (74 <= x && 7 <= y && y <= 30) shotP = 1850;
        else if (64 <= x && x < 74 && 7 <= y && y <= 30) shotP = 1900;
        else if (54 <= x && x < 64 && 7 <= y && y <= 20) shotP = 2000;
    }

    private void torreManual(DcMotorEx tower) {
        int novoTarget = target;
        if (elapsedIntervaloC.milliseconds() >= 50) {
            if (gamepad1.dpad_left && target > -limiteRotativo) {
                novoTarget -= 20;
                elapsedIntervaloC.reset();
            } else if (gamepad1.dpad_right && target < limiteRotativo) {
                novoTarget += 20;
                elapsedIntervaloC.reset();
            }
        }
        if (novoTarget != target && Math.abs(novoTarget - tower.getCurrentPosition()) > 5) {
            target = Range.clip(novoTarget, -limiteRotativo, limiteRotativo);
            encoder(tower, target, towerP);
        }
    }

    private int torreAuto(double x, double y, double heading) {
        if (y > 115) {
            if (heading > -10 && heading < 10) target = -20;
            else if (heading > 35 && heading < 55) target = 230;
            else if (heading > 80 && heading < 100) target = 500;
            else if (heading > 125 && heading < 145) target = 690;
            else if (heading > 160 || heading < -150) target = 750;
            else if (heading > -55 && heading < -35) target = -290;
            else if (heading > -100 && heading < -80) target = -560;
            else if (heading > -145 && heading < -115) target = -750;
        } else if (y > 60 && x > 72) {
            if (heading > -10 && heading < 10) target = -240;
            else if (heading > 35 && heading < 55) target = 15;
            else if (heading > 80 && heading < 100) target = 270;
            else if (heading > 125 && heading < 145) target = 490;
            else if (heading > 160 || heading < -140) target = 750;
            else if (heading > -55 && heading < -35) target = -560;
            else if (heading > -100 && heading < -80) target = -750;
        } else if (y > 60 && x <= 72) {
            if (heading > -10 && heading < 10) target = -110;
            else if (heading > 35 && heading < 55) target = 120;
            else if (heading > 80 && heading < 100) target = 370;
            else if (heading > 125 && heading < 145) target = 670;
            else if (heading > 160 || heading < -160) target = 750;
            else if (heading > -55 && heading < -35) target = -360;
            else if (heading > -150 && heading < -80) target = -720;
        } else if (y <= 60) {
            if (heading > -10 && heading < 10) target = -330;
            else if (heading > 35 && heading < 55) target = -70;
            else if (heading > 80 && heading < 100) target = 150;
            else if (heading > 125 && heading < 145) target = 450;
            else if (heading > 170 || heading < -140) target = 715;
            else if (heading > -55 && heading < -35) target = -630;
            else if (heading > -135 && heading < -60) target = -750;
        }
        return target;
    }

    private void encoder(DcMotorEx motor, int novoAlvo, double power) {
        if (motor.getMode() == DcMotorEx.RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition(novoAlvo);
            motor.setPower(power);
        }
    }

    private double veloV0_RPM(double x, double graus, double y0, double y, double wheel_D, double motor_E) {
        double grausRadians = Math.toRadians(graus);
        double numerador = 9.81 * Math.pow(x, 2);
        double cosTheta = Math.cos(grausRadians);
        double cosQuadrado = Math.pow(cosTheta, 2);
        double termoParenteses = y0 + (x * Math.tan(grausRadians)) - y;
        double denominador = 2 * cosQuadrado * termoParenteses;
        double raiz = Math.sqrt(numerador / denominador) * 2;
        double raiz_Motor = raiz * motor_E;
        double RPM = (raiz_Motor * 60) / (Math.PI * wheel_D);
        return RPM;
    }
}