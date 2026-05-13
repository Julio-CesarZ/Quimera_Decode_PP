package org.firstinspires.ftc.teamcode.pedroPathing.Cam;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TeleOp / Limelight2", group = "TeleOp")
public class LimeLight2 extends OpMode {
    Limelight3A limelight;
    private double distance;
    IMU pinpoint;


    @Override
    public void init(){
        limelight  = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(4);
        pinpoint = hardwareMap.get(IMU.class,"pinpoint");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        pinpoint.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    @Override
    public void start(){
        limelight.start();
    }
    @Override
    public void loop(){
        YawPitchRollAngles orientation = pinpoint.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTage(llResult.getTa());
            telemetry.addData("Distancia", distance);
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy()    );
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addLine();
            telemetry.addData("BotPose", botPose.toString());
            telemetry.addData("Taw", botPose.getOrientation().getYaw());

            telemetry.update();
        }
    }

    public double getDistanceFromTage(double ta) {
        double scale = 1.5; //precisa de calculo
        double distance = (scale / ta);
        return distance;


    }
}
