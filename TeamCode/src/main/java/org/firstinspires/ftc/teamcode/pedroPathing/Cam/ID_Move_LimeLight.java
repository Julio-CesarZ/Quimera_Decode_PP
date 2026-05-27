package org.firstinspires.ftc.teamcode.pedroPathing.Cam;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@Autonomous(name = "Limelight_Encoder_Test", group = "Cam")
public class ID_Move_LimeLight extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotorEx tower;

    @Override
    public void runOpMode() {
        // Configuração do Hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        tower = hardwareMap.get(DcMotorEx.class, "tower");

        // Configuração do Motor
        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setTargetPosition(0);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Variáveis de Controle
        int target = 0;
        double kP = 0.08; // Ajustado para trabalhar com graus (tx) em vez de pixels
        boolean guide = false;
        boolean lastRT = false; // Para evitar toggle infinito

        // Inicialização da Limelight
        limelight.setPollRateHz(50);; // Frequência de atualização (Hz)
        limelight.start();
        limelight.pipelineSwitch(1); // Escolha o pipeline de AprilTag ou Color

        telemetry.addData("Status", "Limelight Pronta");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            boolean targetVisible = (result != null && result.isValid());

            // Lógica de Toggle para o Guia (Botão gatilho direito)
            boolean currentRT = gamepad1.right_trigger > 0.3;
            if (currentRT && !lastRT) {
                guide = !guide;
            }
            lastRT = currentRT;

            if (!guide) {
                // MODO MANUAL (Bumpers controlam o encoder)
                if (gamepad1.x && target > -100) {
                    target -= 10;
                    encoder(tower, target, 0.2);
                    sleep(150);
                } else if (gamepad1.b && target < 100) {
                    target += 10;
                    encoder(tower, target, 0.2);
                    sleep(150);
                } else if (gamepad1.a) {
                    target = 0;
                    encoder(tower, target, 0.2);
                }
            } else {
                // MODO AUTOMÁTICO (Limelight)
                if (targetVisible) {
                    double tx = result.getTx(); // Desvio horizontal em graus (-31 a 31 aprox)

                    // Se o erro for considerável (zona morta de 1 grau)
                    if (Math.abs(tx) > 1.0) {
                        // Calcula o novo alvo baseado no erro horizontal
                        // Ajuste o multiplicador 2 para aumentar a sensibilidade do movimento
                        target = tower.getCurrentPosition() + (int)(tx * 2.5);

                        // Limita o alvo para segurança
                        target = Math.max(-180, Math.min(180, target));

                        double power = Math.abs(tx) * kP;
                        power = Math.max(-0.2, Math.min(0.2, power)); // Clamp de potência

                        encoder(tower, target, power);
                    }
                } else {
                    tower.setPower(0); // Para o motor se perder o alvo
                }
            }

            // Telemetria
            telemetry.addData("Modo Guia", guide);
            if (targetVisible) {
                telemetry.addData("Alvo Detectado", "Sim");
                telemetry.addData("TX (Graus)", result.getTx());
                telemetry.addData("TY (Graus)", result.getTy());
            } else {
                telemetry.addData("Alvo Detectado", "Não");
            }
            telemetry.addData("Posição Atual", tower.getCurrentPosition());
            telemetry.addData("Alvo Encoder", target);
            telemetry.update();
        }
    }

    private void encoder(DcMotor motor, int novoAlvo, double power) {
        motor.setTargetPosition(novoAlvo);
        motor.setPower(power);
    }
}