package org.firstinspires.ftc.teamcode.pedroPathing.Cam;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TeleOp / Limelight follower", group = "TeleOp")
public class AprilFollower extends OpMode {
    Limelight3A limelight;
    IMU imu; // O IMU interno do Control Hub
    private double distance;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(4);

        // 1. Inicializa o IMU interno. No Driver Station, o nome deve ser "imu"
        imu = hardwareMap.get(IMU.class, "imu");

        // 2. Ajuste conforme a posição física do seu Hub no robô!
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Opcional: Resetar o ângulo inicial
        imu.resetYaw();
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop(){
        // 3. Obtém a orientação do giroscópio interno
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        // 4. Envia o Yaw (Z) para a Limelight para melhorar o MT2 (MegaTag2)
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            // MegaTag2 usa o giroscópio para estabilizar a posição
            Pose3D botPose = llResult.getBotpose_MT2();

            distance = getDistanceFromTage(llResult.getTa());

            telemetry.addData("Distância (Aprox)", distance);
            telemetry.addData("Yaw Robô", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("BotPose X", botPose.getPosition().x);
            telemetry.addData("BotPose Y", botPose.getPosition().y);
            telemetry.update();
        }
    }

    public double getDistanceFromTage(double ta) {
        if (ta == 0) return 0;
        double scale = 1.5; // Ajuste este valor conforme testes reais
        return (scale / Math.sqrt(ta)); // Ta é área, usar raiz ajuda na linearidade
    }
}
