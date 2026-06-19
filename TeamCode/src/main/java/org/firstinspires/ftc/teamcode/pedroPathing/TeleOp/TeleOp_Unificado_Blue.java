package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

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

@TeleOp(name = "TeleOp Unificado Blue", group = "TeleOp")
public class TeleOp_Unificado_Blue extends LinearOpMode {

    boolean intervalo_a = false;
    boolean intervalo_x = false;
    boolean intervalo_RT = false;
    boolean intervalo_LT = false;
    boolean intervalo_y = false;
    boolean intervalo_bumper = false;
    boolean intervalo_dpad_up = false;
    boolean intervalo2_dpad_down = false;
    boolean intervalo2_Ltrigger_pressed = false;
    boolean intervalo2_Rtrigger_pressed = false;
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
    boolean neutro = false;
    boolean azul = true;

    private double velocityMultipleir = 0.9;
    private double tx = 0;
    private double positionS = 0.63;
    private double velocityAtual = 0;
    private double lastStamp = 0;
    private double lastTx = 0;
    final double kP = 0.2;
    final double kD = 0.021;
    final double towerP = 1;
    private int shotP = 0;
    private int change = 0;
    private int target = 0;
    private int TeleOp_K = 0;
    final int maxChangeTick = 10;
    final int limiteRotativo = 690;
    final int intakeP = 1;

    ElapsedTime elapsedIntervaloServo = new ElapsedTime();
    ElapsedTime elapsedSuavizador = new ElapsedTime();
    ElapsedTime elapsedintervaloL = new ElapsedTime();
    ElapsedTime elapsedintervaloIntakeSS = new ElapsedTime();
    ElapsedTime elapsedIntervaloC = new ElapsedTime();

    private String changeM = "Movimentação";
    private String changeT = "TeleOp Neutro";

    private final Pose startingPoseTeleopBC = new Pose(60.17, 106.49, Math.toRadians(180));
    private final Pose startingPoseTeleopBF = new Pose(57, 36.05, Math.toRadians(180));
    private final Pose startingPoseTeleopRC = new Pose(83.83, 106.49, 0);
    private final Pose startingPoseTeleopRF = new Pose(87, 36.05, 0);
    private final Pose startingPoseTeleopCenterR = new Pose(72, 72, 0);
    private final Pose startingPoseTeleopCenterB = new Pose(72, 72, Math.toRadians(180));
    private final Pose centerGolR = new Pose(144, 144);
    private final Pose centerGolB = new Pose(0, 144);

    @Override
    public void runOpMode() throws InterruptedException {

        Follower follower = Constants.createFollower(hardwareMap);
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

        intake.setDirection(DcMotorEx.Direction.FORWARD);
        l_right.setDirection(DcMotorEx.Direction.REVERSE);
        l_left.setDirection(DcMotorEx.Direction.REVERSE);

        PIDFCoefficients coefficientsRightMotor = l_right.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients coefficientsLeftMotor = l_left.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        l_right.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficientsRightMotor.p, coefficientsRightMotor.i, coefficientsRightMotor.d, coefficientsRightMotor.f * 1.7
        ));

        l_left.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficientsLeftMotor.p, coefficientsLeftMotor.i, coefficientsLeftMotor.d, coefficientsLeftMotor.f * 1.7
        ));

        tower.setDirection(DcMotorEx.Direction.FORWARD);
        tower.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        tower.setTargetPosition(0);
        tower.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        tower.setPower(0);
        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);

        if (azul) {
            limelight.pipelineSwitch(1);
        } else {
            limelight.pipelineSwitch(0);
        }

        limelight.start();

        s1.setPosition(positionS);

        elapsedintervaloL.reset();
        elapsedIntervaloServo.reset();
        elapsedSuavizador.reset();
        elapsedintervaloIntakeSS.reset();
        elapsedIntervaloC.reset();

        while (!isStarted() && !isStopRequested()) {

            //mode_2 = gamepad1.getUser() != null && gamepad2.getUser() != null; - desativando a função para o torneio

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

            if (!neutro) {
                if (azul) {
                    if (TeleOp_K == 0) {
                        if (gamepad2.left_trigger_pressed && !intervalo2_Ltrigger_pressed) {
                            TeleOp_K = 1;
                        }
                        changeT = "TeleOp de Longe Blue";
                    } else if (TeleOp_K == 1) {
                        if (gamepad2.left_trigger_pressed && !intervalo2_Ltrigger_pressed) {
                            TeleOp_K = 0;
                        }
                        changeT = "TeleOp de Perto Blue";
                    }
                } else {
                    if (TeleOp_K == 0) {
                        if (gamepad2.left_trigger_pressed && !intervalo2_Ltrigger_pressed) {
                            TeleOp_K = 1;
                        }
                        changeT = "TeleOp de Longe Red";
                    } else if (TeleOp_K == 1) {
                        if (gamepad2.left_trigger_pressed && !intervalo2_Ltrigger_pressed) {
                            TeleOp_K = 0;
                        }
                        changeT = "TeleOp de Perto Red";
                    }
                }

                if (gamepad2.right_trigger_pressed && !intervalo2_Rtrigger_pressed) {
                    neutro = true;
                }

                modo_TorreA = true;
                camera = true;
            } else {
                if (gamepad2.right_trigger_pressed && !intervalo2_Rtrigger_pressed) {
                    neutro = false;
                }

                if (azul) {
                    changeT = "TeleOp Nulo Blue";
                } else {
                    changeT = "TeleOp Nulo Red";
                }

                modo_TorreA = false;
                camera = false;
            }

            intervalo2_Rtrigger_pressed = gamepad2.right_trigger_pressed;
            intervalo2_Ltrigger_pressed = gamepad2.left_trigger_pressed;

            follower.update();

            telemetry.addLine(changeT);
            telemetry.update();

            sleep (100);
        }

        if (!neutro) {
            if (azul) {
                if (TeleOp_K == 0) {
                    follower.setStartingPose(startingPoseTeleopBF);
                } else if (TeleOp_K == 1) {
                    follower.setStartingPose(startingPoseTeleopBC);
                }
            } else {
                if (TeleOp_K == 0) {
                    follower.setStartingPose(startingPoseTeleopRF);
                } else if (TeleOp_K == 1) {
                    follower.setStartingPose(startingPoseTeleopRC);
                }
            }
        } else {
            if (azul) {
                follower.setStartingPose(startingPoseTeleopCenterB);
            } else {
                follower.setStartingPose(startingPoseTeleopCenterR);
            }
        }

        while (opModeIsActive()) {
            follower.update();

            double rawHeading = Math.toDegrees(follower.getPose().getHeading());
            double heading = (rawHeading % 360 + 360) % 360;
            double normalHeading = heading + 180;

            if (normalHeading > 360) {
                normalHeading -= 360;
            }

            double x = follower.getPose().getX();
            double y = follower.getPose().getY();

            double xGol;
            double yGol;
            if (azul) {
                xGol = centerGolB.getX();
                yGol = centerGolB.getY();
            } else {
                xGol = centerGolR.getX();
                yGol = centerGolR.getY();
            }

            double distanciaM = Math.hypot(x - xGol, y - yGol) / 39.37;

            double ticks = (veloV0_RPM(distanciaM, 60.1, 0.311, 1.079, 0.075, 1.1) * 28) / 60;

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
                if (elapsedIntervaloServo.seconds() > 1 && velocityAtual + 200 >= shotP && velocityAtual - 200 <= shotP) {
                    positionS = 0.52;
                    s1.setPosition(positionS);
                }
            } else {
                positionS = 0.63;
                s1.setPosition(positionS);
            }

            if (gamepad1.right_trigger > 0.3 && !lF && !reverse && !reverseL && !intervalo_RT) {
                velocityAtual = shotP;
                elapsedIntervaloServo.reset();
                lF = true;
            } else if (gamepad1.right_trigger > 0.3 && lF && !reverse && !reverseL && !intervalo_RT) {
                velocityAtual = 0;
                lF = false;
                intake.setPower(0);
                intakeF = false;
            }
            intervalo_RT = gamepad1.right_trigger > 0.3;

            if (y < 48) {
                if (elapsedIntervaloServo.seconds() > 1.5 && lF) {
                    intake.setPower(intakeP);
                }
            } else {
                if (elapsedIntervaloServo.seconds() > 1.2 && lF) {
                    intake.setPower(intakeP);
                }
            }

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
                    velocityAtual = 0;
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
                lF = false;
                velocityAtual = 0;
            } else if (gamepad1.a && intakeF && !intervalo_a) {
                intake.setPower(0);
                intakeF = false;
            }
            intervalo_a = gamepad1.a;

            double intervaloSS = 200;
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
                        if (azul) {
                            target = Range.clip(torreAuto(y, normalHeading), -limiteRotativo, limiteRotativo);
                        } else {
                            target = Range.clip(torreAuto(y, heading), -limiteRotativo, limiteRotativo);
                        }

                        encoder(tower, target, towerP);
                    } else {
                        torreManual(tower);
                    }
                } else {
                    tower.setPower(0);
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
                if (y < 18) {
                    shotP = (int) ticks + 1000;
                } else if (y >= 18 && y < 40) {
                    shotP = (int) ticks + 1000;
                } else {
                    shotP = (int) ticks + 75;
                }
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
                    if (lF) velocityAtual = shotP;
                }
            }
            intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;

            telemetria(l_right, l_left, tower, x, y, heading);

            if(gamepad2.dpad_down && !intervalo2_dpad_down) {
                if (azul) {
                    follower.setPose(new Pose(133.79, 6.78, Math.toRadians(180)));
                } else {
                    follower.setPose(new Pose(10.21, 6.78, 0));
                }
                follower.update();
                tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                target = 0;
                modo_TorreA = true;
                camera = true;
            }
            intervalo2_dpad_down = gamepad2.dpad_down;
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

    private void camera(LLResult result, double y, DcMotorEx tower) {
        double currentStamp = result.getTimestamp();

        if (currentStamp != lastStamp) {
            tx = result.getTx();
            int atual = tower.getCurrentPosition();

            if (Math.abs(tx) > 1) {
                int position = 0;
                if (azul) {
                    position = (y > 125) ? atual + 50 : atual;
                } else {
                    position = (y > 125) ? atual - 50 : atual;
                }
                int alvo = position + (int) (tx * 7);
                alvo = Range.clip(alvo, -limiteRotativo, limiteRotativo);

                int delta = Range.clip(alvo - target, -maxChangeTick, maxChangeTick);
                target += delta;
                target = Range.clip(target, -limiteRotativo, limiteRotativo);

                double derivative = 0;

                if (lastStamp != 0) {
                    double dt = (currentStamp - lastStamp) / 1000.0;

                    if (dt > 0) {
                        double deltaError = tx - lastTx;
                        derivative = deltaError / dt;
                    }
                }

                double pidOutput = (tx * kP) + (derivative * kD);
                double power = Range.clip(Math.abs(pidOutput), 0.1, 1.0);

                encoder(tower, target, power);

                lastTx = tx;
                lastStamp = currentStamp;

            } else {
                tower.setPower(0);

                lastTx = 0;
                lastStamp = 0;
            }
        }
    }

    private void torreManual(DcMotorEx tower) {
        int novoTarget = target;
        if (elapsedIntervaloC.milliseconds() >= 50) {
            if ((gamepad1.dpad_left || gamepad2.dpad_left) && target > -limiteRotativo) {
                novoTarget -= 20;
                elapsedIntervaloC.reset();
            } else if ((gamepad1.dpad_right || gamepad2.dpad_right) && target < limiteRotativo) {
                novoTarget += 20;
                elapsedIntervaloC.reset();
            }
        }
        if (novoTarget != target && Math.abs(novoTarget - tower.getCurrentPosition()) > 5) {
            target = Range.clip(novoTarget, -limiteRotativo, limiteRotativo);
            encoder(tower, target, towerP);
        }
    }

    public static class Waypoint {
        double heading;
        double target;

        public Waypoint(double heading, double target) {
            this.heading = heading;
            this.target = target;
        }
    }

    private final Waypoint[] pontosSituacao1_R = {
            new Waypoint(0, -15),
            new Waypoint(30, 170),
            new Waypoint(60, 300),
            new Waypoint(90, 490),
            new Waypoint(120, 675),
            new Waypoint(130, 750),
            new Waypoint(187.5, -750),
            new Waypoint(245, -730),
            new Waypoint(295, -750),
            new Waypoint(300, -385),
            new Waypoint(330, -200),
            new Waypoint(360, -15),
    };

    private final Waypoint[] pontosSituacao2_R = {
            new Waypoint(0, -230),
            new Waypoint(30, -90),
            new Waypoint(60, 120),
            new Waypoint(90, 300),
            new Waypoint(120, 460),
            new Waypoint(150, 645),
            new Waypoint(165, 730),
            new Waypoint(222.5, -750),
            new Waypoint(280, -740),
            new Waypoint(300, -616),
            new Waypoint(330, -425),
            new Waypoint(360, -230),
    };

    private final Waypoint[] pontosSituacao3_R = {
            new Waypoint(0, -355),
            new Waypoint(30, -150),
            new Waypoint(60, 20),
            new Waypoint(90, 195),
            new Waypoint(120, 370),
            new Waypoint(150, 550),
            new Waypoint(180, 735),
            new Waypoint(240, 750),
            new Waypoint(295, -750),
            new Waypoint(300, -616),
            new Waypoint(330, -505),
            new Waypoint(360, -335),
    };

    private final Waypoint[] pontosSituacao1_B = {
            new Waypoint(0, 40),
            new Waypoint(30, 190),
            new Waypoint(60, 370),
            new Waypoint(90, 550),
            new Waypoint(120, 750),
            new Waypoint(115, -750),
            new Waypoint(240, -650),
            new Waypoint(270, -490),
            new Waypoint(300, -280),
            new Waypoint(330, -130),
            new Waypoint(360, 40),
    };

    private final Waypoint[] pontosSituacao2_B = {
            new Waypoint(0, 305),
            new Waypoint(30, 440),
            new Waypoint(60, 660),
            new Waypoint(75, 750),
            new Waypoint(131.5, -750),
            new Waypoint(190, -700),
            new Waypoint(220, -530),
            new Waypoint(250, -346),
            new Waypoint(280, -165),
            new Waypoint(310, -20),
            new Waypoint(340, 190),
            new Waypoint(360, 305),
    };

    private final Waypoint[] pontosSituacao3_B = {
            new Waypoint(0, 375),
            new Waypoint(30, 570),
            new Waypoint(60, 750),
            new Waypoint(115, -750),
            new Waypoint(175, -700),
            new Waypoint(205, -545),
            new Waypoint(245, -335),
            new Waypoint(275, -185),
            new Waypoint(305, 85),
            new Waypoint(335, 260),
            new Waypoint(360, 375),
    };

    private double interpola(Waypoint[] pontos, double heading) {
        if (heading <= pontos[0].heading) return pontos[0].target;
        if (heading >= pontos[pontos.length - 1].heading) return pontos[pontos.length - 1].target;

        for (int i = 0; i < pontos.length - 1; i++) {
            if (heading >= pontos[i].heading && heading <= pontos[i + 1].heading) {
                double t = (heading - pontos[i].heading) / (pontos[i + 1].heading - pontos[i].heading);
                return pontos[i].target + t * (pontos[i + 1].target - pontos[i].target);
            }
        }
        return pontos[pontos.length - 1].target;
    }

    private int torreAuto(double y, double heading) {

        if (azul) {
            if (y >= 115) {
                target = (int) interpola(pontosSituacao1_B, heading);

            } else if (y < 115 && y > 45) {
                target = (int) interpola(pontosSituacao2_B, heading);

            } else if (y <= 45) {
                target = (int) interpola(pontosSituacao3_B, heading);
            }
        } else {
            if (y >= 115) {
                target = (int) interpola(pontosSituacao1_R, heading);

            } else if (y < 115 && y > 45) {
                target = (int) interpola(pontosSituacao2_R, heading);

            } else if (y <= 45) {
                target = (int) interpola(pontosSituacao3_R, heading);
            }
        }

        return Range.clip(target, -limiteRotativo, limiteRotativo);
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