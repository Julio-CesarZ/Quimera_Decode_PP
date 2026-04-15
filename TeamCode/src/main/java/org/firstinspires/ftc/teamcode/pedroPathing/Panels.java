package org.firstinspires.ftc.teamcode.pedroPathing;
import com.bylazar.configurables.annotations.Configurable;
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
import com.bylazar.panels.core.Panels;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "Ativar Panels", group = "TeleOp")
public abstract class Panels extends OpMode {
    @TeleOp(name = "Teste Panels")
    public class MeuRoboTeleOp extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            // 1. INICIALIZAÇÃO (Antes do waitForStart)
            Panels panels = Panels.getInstance(hardwareMap);

            // Crie seus componentes aqui para que apareçam assim que o código carregar
            ConfigurableBoolean myToggle = new ConfigurableBoolean("Meu Botão", false);
            panels.addConfigurable(myToggle);

            waitForStart();

            while (opModeIsActive()) {
                // 2. ATUALIZAÇÃO (Dentro do loop principal)
                // O Panels geralmente atualiza sozinho, mas você pode ler os valores aqui:
                if (myToggle.get()) {
                    // Faz algo se o botão no site estiver ligado
                }

                panels.update(); // Garante que os dados sejam enviados ao navegador
            }
        }
    }
}