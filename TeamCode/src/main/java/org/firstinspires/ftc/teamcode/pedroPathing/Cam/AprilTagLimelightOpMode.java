package org.firstinspires.ftc.teamcode.pedroPathing.Cam;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * ============================================================
 *  AprilTagLimelightOpMode
 * ============================================================
 *  OpMode para leitura de AprilTags do jogo DECODE (2025-2026)
 *  utilizando a Limelight 3A com pipeline de Fiducial Markers.
 *
 *  IMPORTANTE — O ERRO ANTERIOR:
 *  O código anterior usava getBarcodeResults() (pipeline Barcode/QR),
 *  que não detecta AprilTags. AprilTags exigem pipeline do tipo
 *  "Fiducial Markers" e o método getFiducialResults().
 *
 *  PRÉ-REQUISITOS (interface web — http://limelight.local:5801):
 *  - Pipeline 4 configurada como "Fiducial Markers"
 *  - Na aba "Standard": family = "AprilTag Classic 36h11"
 *  - Limelight nomeada como "limelight" na config. do robô
 *
 *  AprilTags do DECODE detectadas por este OpMode:
 *  ID 20 -> Goal (localização)
 *  ID 21 -> Obelisk (motif)
 *  ID 22 -> Obelisk (motif)
 *  ID 23 -> Obelisk (motif)
 *  ID 24 -> Goal (localização)
 *
 *  Documentação oficial:
 *  https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming
 * ============================================================
 */
@TeleOp(name = "AprilTag Reader DECODE - Limelight 3A", group = "Sensor")
public class AprilTagLimelightOpMode extends LinearOpMode {

    // -------------------------------------------------------
    // CONSTANTES DE CONFIGURAÇÃO
    // -------------------------------------------------------

    /** Pipeline configurada como "Fiducial Markers" (AprilTag) */
    private static final int APRILTAG_PIPELINE_INDEX = 4;

    /** Taxa de atualização de dados da Limelight */
    private static final int POLL_RATE_HZ = 100;

    /** Nome do dispositivo na configuração do robô */
    private static final String LIMELIGHT_NAME = "limelight";

    /**
     * IDs das AprilTags do jogo DECODE (2025-2026).
     * IDs 20 e 24 -> Goals (recomendados para localização).
     * IDs 21, 22, 23 -> Obelisk (identificação de Motif).
     */
    private static final int[] DECODE_TAG_IDS = {20, 21, 22, 23, 24};

    // -------------------------------------------------------
    // VARIÁVEIS DE ESTADO
    // -------------------------------------------------------

    private Limelight3A limelight;

    private int    lastTagId    = -1;
    private String lastTagLabel = "Nenhuma";
    private double lastTx       = 0.0;
    private double lastTy       = 0.0;
    private double lastTa       = 0.0;
    private int    tagCount     = 0;

    // -------------------------------------------------------
    // MÉTODO PRINCIPAL
    // -------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        initLimelight();

        telemetry.addLine("✅ Limelight 3A inicializada.");
        telemetry.addLine("   Pipeline 4 (Fiducial/AprilTag) ativa.");
        telemetry.addLine("   Aguardando PLAY...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                processAprilTagResults(result);
            } else {
                tagCount = 0;
            }

            displayTelemetry(result);
            handleGamepadInput();
            telemetry.update();
        }

        limelight.stop();
    }

    // -------------------------------------------------------
    // INICIALIZAÇÃO
    // -------------------------------------------------------

    /**
     * Inicializa a Limelight 3A e ativa a pipeline de AprilTag.
     *
     * ATENÇÃO: a pipeline 4 DEVE estar configurada como
     * "Fiducial Markers" na interface web da Limelight,
     * caso contrário nenhuma AprilTag será detectada.
     */
    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.start();
    }

    // -------------------------------------------------------
    // PROCESSAMENTO DAS APRILTAGS
    // -------------------------------------------------------

    /**
     * Extrai os dados de AprilTag a partir do LLResult.
     *
     * getFiducialResults() retorna todas as AprilTags detectadas.
     * Cada FiducialResult contém:
     *   - getFiducialId()       -> ID numérico da tag
     *   - getTargetXDegrees()   -> deslocamento horizontal (graus)
     *   - getTargetYDegrees()   -> deslocamento vertical (graus)
     *   - getRobotPoseTargetSpace() -> pose do robô relativa à tag (3D)
     *
     * Apenas tags com IDs válidos do DECODE são processadas.
     *
     * @param result LLResult válido da Limelight
     */
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

    /**
     * Verifica se o ID pertence a uma AprilTag do jogo DECODE.
     */
    private boolean isDecodeTag(int id) {
        for (int validId : DECODE_TAG_IDS) {
            if (id == validId) return true;
        }
        return false;
    }

    /**
     * Retorna o rótulo descritivo de cada AprilTag do DECODE.
     *
     * IDs 20 e 24 -> Goals (mira e localização).
     * IDs 21-23   -> Obelisk (Motif sorteado).
     */
    private String getTagLabel(int id) {
        switch (id) {
            case 20: return "ID 20 — Goal (Azul)";
            case 21: return "ID 21 — Obelisk Motif A";
            case 22: return "ID 22 — Obelisk Motif B";
            case 23: return "ID 23 — Obelisk Motif C";
            case 24: return "ID 24 — Goal (Vermelho)";
            default: return "ID " + id + " — Desconhecida";
        }
    }

    // -------------------------------------------------------
    // CALLBACK — LÓGICA DE JOGO
    // -------------------------------------------------------

    /**
     * Chamado a cada frame em que uma AprilTag DECODE é detectada.
     * Substitua o conteúdo conforme a estratégia da sua equipe.
     *
     * Exemplos de uso:
     *   ID 20 ou 24 -> alinhar robô ao Goal para lançar ARTIFACTS
     *   ID 21/22/23 -> identificar o Motif sorteado no Obelisk
     *
     * @param tagId   ID da AprilTag primária detectada
     * @param fiducial Objeto com dados 2D e 3D da tag
     */
    private void onAprilTagDetected(int tagId, LLResultTypes.FiducialResult fiducial) {

        // --- Exemplo: identificar o Motif do Obelisk ---
        // if (tagId == 21) { motifAtivo = "MOTIF_A"; }
        // if (tagId == 22) { motifAtivo = "MOTIF_B"; }
        // if (tagId == 23) { motifAtivo = "MOTIF_C"; }

        // --- Exemplo: mira no Goal ---
        // if (tagId == 20 || tagId == 24) {
        //     double erro = fiducial.getTargetXDegrees();
        //     double velocidadeStrafe = erro * 0.03;
        //     setVelocidadeStrafe(velocidadeStrafe);
        // }

        // --- Exemplo: distância via pose 3D (requer Full 3D na pipeline) ---
        // double[] pose = fiducial.getRobotPoseTargetSpace().toArray();
        // double distancia = -pose[2]; // eixo Z em metros
    }

    // -------------------------------------------------------
    // ENTRADA DO GAMEPAD
    // -------------------------------------------------------

    /** Botão X: reseta o estado do último alvo detectado. */
    private void handleGamepadInput() {
        if (gamepad1.x) {
            lastTagId    = -1;
            lastTagLabel = "Nenhuma";
            lastTx = lastTy = lastTa = 0.0;
            tagCount = 0;
        }
    }

    // -------------------------------------------------------
    // TELEMETRIA
    // -------------------------------------------------------

    /**
     * Exibe todas as informações relevantes no Driver Station.
     */
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
        telemetry.addLine("=== APRILTAGS DECODE (Pipeline 4) ===");

        if (result != null && result.isValid() && tagCount > 0) {
            telemetry.addData("🟢 Tags detectadas", tagCount);
            telemetry.addData("Tag primária",  lastTagLabel);
            telemetry.addData("Posição X (Tx)", String.format("%.2f°", lastTx));
            telemetry.addData("Posição Y (Ty)", String.format("%.2f°", lastTy));
            telemetry.addData("Área (Ta)",      String.format("%.2f%%", lastTa));
        } else {
            telemetry.addLine("🔴 Nenhuma AprilTag detectada");
            if (lastTagId != -1) {
                telemetry.addData("Última lida", lastTagLabel);
            }
        }

        telemetry.addLine("");
        telemetry.addLine("[ X ] Resetar última tag");
    }
}