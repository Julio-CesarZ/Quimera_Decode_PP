package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Coordenadas Fundo", group = "Test")
public class Coordenadas_Fundo extends LinearOpMode {
    private Pose startingPose;
    private boolean intervalo_y = false;
    private boolean startPose = true;

    @Override
    public void runOpMode() throws InterruptedException {

        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.y && !intervalo_y) {
                startPose = !startPose;
            }
            intervalo_y = gamepad1.y;

            telemetry.addLine(startPose ? "Inicial" : "72, 72");
            telemetry.addLine("Pressione [Y] para alternar a pose Inicial\n");
            telemetry.update();
        }

        startingPose = startPose ? new Pose(89.17, 8.27, 0) : new Pose(72, 72, 0);
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            follower.update();

            double heading = Math.toDegrees(follower.getPose().getHeading());
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();

            telemetria(x, y, heading);

            double forward = Math.pow(-gamepad1.left_stick_y * 0.6, 3);
            double strafe = Math.pow(-gamepad1.left_stick_x * 0.6, 3);
            double turn = Math.pow(-gamepad1.right_stick_x * 0.6, 3);

            follower.setTeleOpDrive(forward, strafe, turn, true);
        }
    }

    private void telemetria(double x, double y, double heading) {
        telemetry.addData("X", x).addData("Y", y);
        telemetry.addData("Heading", heading);
        telemetry.update();
    }
}