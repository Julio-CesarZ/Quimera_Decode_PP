package org.firstinspires.ftc.teamcode.pedroPathing.Cam;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;
@Disabled

@TeleOp (name = "LimeLight3", group = "Cam")
public class LimeLight3 extends LinearOpMode {

    Limelight3A limelight;
    private Follower follower;
    private Pose targetPose;
    private int    lastTagId    = -1;
    private String lastTagLabel = "Nenhuma";
    private double lastTx       = 0.0;
    private double lastTy       = 0.0;
    private double lastTa       = 0.0;
    private int    tagCount     = 0;
    private static final int[] DECODE_TAG_IDS = {24};
    private double ticksPorGrau = 0.5; // Ajuste este valor (sensibilidade)
    private boolean guide = true;      // Ativador do alinhamento automático

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            follower.updatePose();
            Pose posicaoAtual = follower.getPose();
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                processAprilTagResults(result);
            } else {
                tagCount = 0;
            }

            double x = posicaoAtual.getX();
            double y = posicaoAtual.getY();
            double angulo = posicaoAtual.getHeading();

            displayTelemetry(result);
            telemetry.update();

        }
    }

    private void processAprilTagResults(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

        tagCount = 0;
        LLResultTypes.FiducialResult primaryTag = null;

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();

            if (isDecodeTag(id)) {
                tagCount++;
                if (primaryTag == null) {
                    primaryTag = fiducial;
                }
            }
        }

        if (primaryTag != null) {
            lastTagId    = primaryTag.getFiducialId();
            lastTagLabel = getTagLabel(lastTagId);
            lastTx       = primaryTag.getTargetXDegrees();
            lastTy       = primaryTag.getTargetYDegrees();
            lastTa       = result.getTa();

            onAprilTagDetected(lastTagId, primaryTag);
        }
    }

    private boolean isDecodeTag(int id) {
        for (int validId : DECODE_TAG_IDS) {
            if (id == validId) return true;
        }
        return false;
    }

    private void onAprilTagDetected(int tagId, LLResultTypes.FiducialResult fiducial) {
        if (!guide) return;

        // tx é o erro horizontal da tag em relação ao centro da câmera (em graus)
        double erroX = fiducial.getTargetXDegrees();

        // Margem de erro (deadzone) para evitar que o robô trema
        if (Math.abs(erroX) > 1.5) {

            // Pegamos a pose atual do robô
            Pose currentPose = follower.getPose();

        /*
           LÓGICA: Criamos uma "Target Pose" (Pose Alvo)
           Aqui, estamos pedindo para o robô rotacionar para compensar o erroX.
           O Pedro Pathing trabalha em radianos, então convertemos o erro.
        */
            double novoHeading = currentPose.getHeading() + Math.toRadians(erroX * -1);

            // Criamos um caminho curto (uma linha) até a nova orientação
            // Se quiser que ele apenas gire no lugar:
            follower.holdPoint(new Pose(currentPose.getX(), currentPose.getY(), novoHeading));
        }
    }


    private void displayTelemetry(LLResult result) {
        telemetry.setMsTransmissionInterval(11);

        LLStatus status = limelight.getStatus();
        telemetry.addLine("=== LIMELIGHT 3A ===");
        telemetry.addData("Pipeline ativa",
                result != null ? result.getPipelineIndex() : "---");
        telemetry.addData("FPS",
                status != null ? String.format("%.1f", status.getFps()) : "---");
        telemetry.addData("Temp (°C)",
                status != null ? String.format("%.1f", status.getTemp()) : "---");

        telemetry.addLine("");
        telemetry.addLine("=== APRILTAGS DECODE (Pipeline 1) ===");

        if (result != null && result.isValid() && tagCount > 0) {
            telemetry.addData("🟢 Tags detectadas", tagCount);
            telemetry.addData("Tag primária",  lastTagLabel);
            telemetry.addData("Posição X (Tx)", String.format("%.2f°", lastTx));
            telemetry.addData("Posição Y (Ty)", String.format("%.2f°", lastTy));
            telemetry.addData("Área (Ta)",      String.format("%.2f%%", lastTa));
            Pose robotPose = follower.getPose();
            telemetry.addLine("=== POSIÇÃO DO ROBÔ ===");
            telemetry.addData("X (Polegadas)", String.format("%.2f", robotPose.getX()));
            telemetry.addData("Y (Polegadas)", String.format("%.2f", robotPose.getY()));
            telemetry.addData("Heading (Graus)", String.format("%.2f", Math.toDegrees(robotPose.getHeading())));

        } else {
            telemetry.addLine("🔴 Nenhuma AprilTag detectada");
            if (lastTagId != -1) {
                telemetry.addData("Última lida", lastTagLabel);
            }
        }

        telemetry.addLine("");
    }

    private String getTagLabel(int id) {
        switch (id) {
            case 24: return "ID 24 — Goal (Vermelho)";
            default: return "ID " + id + " — Desconhecida";
        }
    }
}

