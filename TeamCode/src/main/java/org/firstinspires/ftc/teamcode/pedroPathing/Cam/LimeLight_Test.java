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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp (name = "LimeLightTest", group = "Cam")
public class LimeLight_Test extends LinearOpMode {

    Limelight3A limelight;
    private Follower follower;
    private Pose targetPose;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            LLResult result = limelight.getLatestResult();

            // 1. Verifica se a Limelight está vendo algo válido
            if (result != null && result.isValid()) {
                // Pega a posição relativa ao robô (Botpose)
                Pose3D botpose = result.getBotpose();

                if (botpose != null) {
                    // 2. Converte a leitura da Limelight para o sistema do Pedro Pathing
                    // Nota: Limelight usa metros, Pedro costuma usar polegadas (ajuste se necessário)
                    targetPose = new Pose(botpose.getPosition().x, botpose.getPosition().y, botpose.getOrientation().getYaw());

                    // 3. Só cria o caminho se não estiver seguindo ou se o alvo se moveu muito
                    if (!follower.isBusy()) {
                        follower.followPath(
                                follower.pathBuilder()
                                        .addPath(new BezierLine(follower.getPose(), targetPose))
                                        .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                                        .build()
                        );
                    }
                }
            } else {
                // Se perder o alvo, você pode optar por parar o robô
                if (follower.isBusy()) {
                    follower.breakFollowing();
                }
            }

            // Telemetria para diagnóstico
            telemetry.addData("Status", follower.isBusy() ? "Seguindo Alvo" : "Procurando...");
            telemetry.addData("Pose Atual", follower.getPose().toString());
            telemetry.update();

        }
    }
}

