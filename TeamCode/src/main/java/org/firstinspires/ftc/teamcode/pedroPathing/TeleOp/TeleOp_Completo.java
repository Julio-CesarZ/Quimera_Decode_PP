package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    boolean intervalo_bumper = false;
    boolean intervalo_dpad_down = false;
    boolean intervalo_dpad_up = false;
    boolean intervalo_dpad_left = false;
    boolean intervalo_dpad_right = false;
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
    boolean notFieldCentric = true;
    boolean modo_ShotPA = true;
    boolean camera = true;
    private double velocityMultipleir = 0.7;
    private double shotP = 1450;
    private double velocityShot = 0;
    private double intakeP = 1;
    private double towerP = 1;
    private double tx = 0;
    private double ta = 0;
    private double position = 0.75;
    private double velocityAtual = 0;
    private double lastStamp = 0;
    final double kP = 0.08;
    private int change = 2;
    private int target = 0;
    final int maxChangeTick = 10;
    ElapsedTime elapsedIntervaloServo = new ElapsedTime();
    ElapsedTime elapsedSuavizador = new ElapsedTime();
    ElapsedTime elapsedintervaloL = new ElapsedTime();
    ElapsedTime elapsedintervaloIntakeSS = new ElapsedTime();
    ElapsedTime elapsedIntervaloC = new ElapsedTime();
    private String changeM = "Lançador";
    private final Pose startingPose = new Pose(108.63, 134.58, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        follower.startTeleopDrive();
        follower.update();

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
        l_left.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients coefficientsRightMotor = l_right.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients coefficientsLeftMotor = l_left.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        l_right.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficientsRightMotor.p,
                coefficientsRightMotor.i,
                coefficientsRightMotor.d,
                coefficientsRightMotor.f * 1.5
        ));

        l_left.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficientsLeftMotor.p,
                coefficientsLeftMotor.i,
                coefficientsLeftMotor.d,
                coefficientsLeftMotor.f * 1.5
        ));

        tower.setDirection(DcMotorSimple.Direction.REVERSE);
        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tower.setTargetPosition(target);
        tower.setPower(0);
        tower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(60);
        limelight.start();
        limelight.pipelineSwitch(1);

        s1.setPosition(position);

        elapsedintervaloL.reset();
        elapsedIntervaloServo.reset();
        elapsedSuavizador.reset();
        elapsedintervaloIntakeSS.reset();
        elapsedIntervaloC.reset();

        while (!isStarted() && !isStopRequested()) {

            mode_2 = gamepad2.id != -1;

            if (mode_2) {
                telemetry.addLine("Modo para dois jogadores");
                telemetry.addLine("Gamepad 1 - Controle das funções. Gamepad 2 - Controle do Chassi");
            } else {
                telemetry.addLine("Modo para um jogador");
                telemetry.addLine("Gamepad 1 - Controle Geral");
            }

            if (notFieldCentric) {
                telemetry.addLine();
                telemetry.addLine("Pressione [RB] e [LB] simultaneamente para definir a orientação para a Arena");
                telemetry.addLine();

                if (gamepad1.right_bumper && gamepad1.left_bumper) {
                    notFieldCentric = false;
                    sleep(500);
                }

                telemetry.addLine("Orientação atual: Robô");
            } else {
                telemetry.addLine();
                telemetry.addLine("Pressione [RB] e [LB] simultaneamente para definir a orientação para o robô");
                telemetry.addLine();

                if (gamepad1.right_bumper && gamepad1.left_bumper) {
                    notFieldCentric = true;
                    sleep(500);
                }

                telemetry.addLine("Orientação atual: Arena");
            }

            if (camera) {
                if (!modo_TorreA) {
                    telemetry.addLine();
                    telemetry.addLine("Pressione [LT] para ativar o modo Auto da Torre");
                    telemetry.addLine("Também é possível pressionar os analógicos simultaneamente durante o loop para alternar");

                    if (gamepad1.left_trigger > 0.3) {
                        modo_TorreA = true;
                        sleep(500);
                    }

                } else {
                    telemetry.addLine();
                    telemetry.addLine("Pressione [LT] para desativar o modo Auto da Torre");
                    telemetry.addLine("Também é possível pressionar os analógicos simultaneamente durante o loop para alternar");

                    if (gamepad1.left_trigger > 0.3) {
                        modo_TorreA = false;
                        sleep(500);
                    }
                }
            }

            if (camera) {
                telemetry.addLine();
                telemetry.addLine("Câmera Ativada");
                telemetry.addLine();
            } else {
                telemetry.addLine();
                telemetry.addLine("Câmera Desativada");
                telemetry.addLine();
            }

            if (gamepad1.y && camera) {
                camera = false;
                sleep(500);
            } else if (gamepad1.y && !camera) {
                camera = true;
                sleep(500);
            }

            telemetry.addLine("Pressione [Y] para ativar ou desativar a câmera");

            telemetry.addLine();
            telemetry.addLine("Pressione [RT] para definir o estado da telemetria");
            telemetry.addLine();

            if (gamepad1.right_trigger > 0.3 && !telemetria) {
                telemetria = true;
                sleep(500);
            } else if (gamepad1.right_trigger > 0.3 && telemetria) {
                telemetria = false;
                sleep(500);
            }

            if (telemetria) {
                telemetry.addLine("Telemetria: Ativada");
            } else {
                telemetry.addLine("Telemetria: Desativada");
            }

            telemetry.update();
        }

        sleep(200);

        while (opModeIsActive()) {
            double heading = Math.toDegrees(follower.getPose().getHeading());

            LLResult result = limelight.getLatestResult();
            targetVisible = (result != null && result.isValid());

            double forward;
            double strafe;
            double turn;
            if (mode_2) {
                forward = Math.pow(gamepad2.left_stick_y * velocityMultipleir, 3);
                strafe = Math.pow(-gamepad2.left_stick_x * velocityMultipleir, 3);
                turn = Math.pow(-gamepad2.right_stick_x * velocityMultipleir, 3);
            } else {
                forward = Math.pow(gamepad1.left_stick_y * velocityMultipleir, 3);
                strafe = Math.pow(-gamepad1.left_stick_x * velocityMultipleir, 3);
                turn = Math.pow(-gamepad1.right_stick_x * velocityMultipleir, 3);
            }

            follower.setTeleOpDrive(forward, strafe, turn, notFieldCentric);
            follower.update();

            if (lF || gamepad1.left_trigger > 0.3) {
                if (elapsedIntervaloServo.milliseconds() > 500) {
                    position = 0.65;
                    s1.setPosition(position);
                }
            } else {
                position = 0.82;
                s1.setPosition(position);
            }

            double deltaTime = elapsedSuavizador.seconds();
            elapsedSuavizador.reset();

            if (gamepad1.right_trigger > 0.3 && !lF && !reverse && !reverseL && !intervalo_RT) {
                velocityShot = shotP;
                elapsedIntervaloServo.reset();
                lF = true;
            } else if (gamepad1.right_trigger > 0.3 && lF && !intervalo_RT) {
                velocityShot = 0;
                lF = false;
            }
            intervalo_RT = gamepad1.right_trigger > 0.3;

            double erroGeral = velocityShot - velocityAtual;

            double maxAccelerationPerS = 1500;
            double maxChange = maxAccelerationPerS * deltaTime;

            if (Math.abs(erroGeral) <= maxChange) {
                velocityAtual = velocityShot;
            } else {
                velocityAtual += Math.signum(erroGeral) * maxChange;
            }

            if (!reverse && !reverseL && elapsedintervaloL.milliseconds() >= 500) {
                l_right.setVelocity(velocityAtual);
                l_left.setVelocity(velocityAtual);
            }

            if (gamepad1.y && !intakeF && !lF && !intakeSS && !reverse) {
                l_right.setVelocity(-1000);
                l_left.setVelocity(-1000);
                reverseL = true;
            } else if (!gamepad1.y && reverseL) {
                velocityShot = 0;
                reverseL = false;
                elapsedintervaloL.reset();
            }

            if (gamepad1.a && !intakeF && !intervalo_a && !intakeSS) {
                intake.setPower(intakeP);
                intakeF = !intakeF;
            } else if (gamepad1.a && intakeF && !intervalo_a) {
                intake.setPower(0);
                intakeF = !intakeF;
            }
            intervalo_a = gamepad1.a;

            if (gamepad1.b && !intakeF && !lF && !intakeSS && !reverseL) {
                intake.setPower(-intakeP);
                l_right.setVelocity(-1000);
                l_left.setVelocity(-1000);
                reverse = true;
            } else if (!gamepad1.b && reverse) {
                intake.setPower(0);
                velocityShot = 0;
                reverse = false;
                elapsedintervaloL.reset();
            }

            double intervaloSS = 300;
            if (gamepad1.dpad_up && !intakeSS && !intervalo_dpad_up) {
                intakeSS = true;
                elapsedintervaloIntakeSS.reset();
            } else if (gamepad1.dpad_up && intakeSS && !intervalo_dpad_up) {
                intake.setPower(0);
                intakeSS = false;
            }
            intervalo_dpad_up = gamepad1.dpad_up;

            if (intakeSS) {
                if (elapsedintervaloIntakeSS.milliseconds() % (intervaloSS * 2) <= intervaloSS) {
                    intake.setPower(intakeP);
                } else {
                    intake.setPower(0);
                }
            }

            int limiteRotativo = 650;
            if (!targetVisible) {

                if (!modo_TorreA) {
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

                    intervalo_dpad_down = gamepad1.dpad_down;
                    intervalo_dpad_left = gamepad1.dpad_left;
                    intervalo_dpad_right = gamepad1.dpad_right;

                    if (novoTarget != target) {
                        if (Math.abs(novoTarget - tower.getCurrentPosition()) > 5) {
                            target = novoTarget;
                            encoder(tower, target, towerP);
                        }
                    }
                } else {

                    if (camera) {
                        if (heading < 15 && heading > -15) {
                            target = -224;
                        } else if (heading < 105 && heading > 85) {
                            target = 298;
                        } else if (heading < 60 && heading > 40) {
                            target = 19;
                        } else if (heading < -40 && heading > -60) {
                            target = -502;
                        } else if (heading < 150 && heading > 130) {
                            target = 503;
                        } else if (heading < -165 || heading > 165) {
                            target = 650;
                        }

                        encoder(tower, target, towerP);
                    }
                }

            } else {
                if (camera) {
                    tx = result.getTx();
                    ta = result.getTa();

                    if (Math.abs(tx) > 1) {

                        int position = tower.getCurrentPosition();
                        int alvo = position + (int) (tx * 7);

                        alvo = Math.max(-limiteRotativo, Math.min(limiteRotativo, alvo));

                        int delta = alvo - target;

                        delta = Range.clip(delta, -maxChangeTick, maxChangeTick);

                        target += delta;

                        target = Math.max(-limiteRotativo, Math.min(limiteRotativo, target));

                        double power = tx * kP;
                        double limitPL = 1;
                        power = Math.max(-limitPL, Math.min(limitPL, power));

                        if (result.getTimestamp() != lastStamp) {
                            encoder(tower, target, Math.abs(power));
                        }

                        lastStamp = result.getTimestamp();

                    } else {
                        tower.setPower(0);
                    }

                    //todo: Aumentar a flexibilidade da lógica abaixo com o objetivo de permitir um controle mais fino de potência por toda a arena

                    /*
                    if (modo_ShotPA) {
                        if (ta <= 1) {
                            shotP = 2000;
                        } else if (ta > 1) {
                            shotP = 1450;
                        }
                    }
                    */
                }
            }

            if ((gamepad1.right_stick_button || gamepad1.left_stick_button) && modo_TorreA && !intervalo_stick) {
                modo_TorreA = false;
            } else if ((gamepad1.right_stick_button || gamepad1.left_stick_button) && !modo_TorreA && !intervalo_stick) {
                modo_TorreA = true;
            }
            intervalo_stick = gamepad1.right_stick_button || gamepad1.left_stick_button;

            if (gamepad1.x && change == 0 && !intervalo_x) {
                change = 1;
                changeM = "Intake";
            } else if (gamepad1.x && change == 1 && !intervalo_x) {
                change = 2;
                changeM = "Lançador";
            } else if (gamepad1.x && change == 2 && !intervalo_x) {
                change = 3;
                changeM = "Torre";
            } else if (gamepad1.x && change == 3 && !intervalo_x) {
                change = 0;
                changeM = "Movimentação";
            }
            intervalo_x = gamepad1.x;

            if (change == 0) {
                if (gamepad1.right_bumper && !intervalo_bumper) {
                    velocityMultipleir += 0.05;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    velocityMultipleir -= 0.05;
                }
                velocityMultipleir = Range.clip(velocityMultipleir, 0.0, 1.0);

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
                    shotP += 50;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    shotP -= 50;
                }
                shotP = Range.clip(shotP, 0, 2800); //Utilizando 100% -> ticks por revolução - 28; RPM máximo - 6000
                if (lF) {
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

            if (telemetria) {
                telemetry.addLine("Telemetria Ativada");
                telemetry.addLine();
                telemetry.addData("Troca de poder atual", changeM);
                telemetry.addData("Chassi Power", velocityMultipleir);
                telemetry.addData("Intake Power", intakeP);
                telemetry.addData("Variável ShotP", shotP);
                telemetry.addData("Valor de Referência dos Motores", velocityAtual);
                telemetry.addData("Ticks/s do Motor da Direita", l_right.getVelocity());
                telemetry.addData("Ticks/s do Motor da Esquerda", l_left.getVelocity());
                telemetry.addData("RPM da Roda do Lançador", Math.max(l_right.getVelocity(), l_left.getVelocity()) * 2.857);
                telemetry.addData("Tower Power", towerP);
                telemetry.addData("Servo", position);
                telemetry.addData("Posição da Torre", tower.getCurrentPosition()).addData("Alvo da Torre", target);
                telemetry.addLine();
                telemetry.addData("X", follower.getPose().getX()).addData("Y", follower.getPose().getY()).addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
                telemetry.addLine();

                if (camera) {
                    if (targetVisible) {
                        telemetry.addData("Alvo Detectado", "Sim");
                        telemetry.addData("TX (Graus)", tx);
                        telemetry.addData("TA (Area)", ta);
                    } else {
                        telemetry.addData("Alvo Detectado", "Não");
                    }
                } else {
                    telemetry.addLine("Câmera Desativada");
                }

                telemetry.addLine();
                telemetry.addData("Modo Automático da Torre", modo_TorreA);
                telemetry.addLine();
                telemetry.addData("Modo Automático da calibração de shotP", modo_ShotPA);

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