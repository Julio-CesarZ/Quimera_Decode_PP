package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Teste Single Motor", group = "Tests")
public class SingleMotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //DcMotorEx tower = hardwareMap.get(DcMotorEx.class, "tower");

        double power = 0.8;

        boolean right = false;
        boolean left = false;

        int direcaoAtual = 0;

        ElapsedTime intervalo = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.right_bumper && !right) {
                power += 0.1;
                power = Range.clip(power, 0, 1.0);
            }
            right = gamepad1.right_bumper;
            if(gamepad1.left_bumper && !left) {
                power -= 0.1;
                power = Range.clip(power, 0, 1.0);
            }
            left = gamepad1.left_bumper;

            if (gamepad1.b) {
                if(direcaoAtual >= 0 && intervalo.milliseconds() > 300) {
                    //tower.setPower(power);
                    direcaoAtual = 1;
                } else {
                    //tower.setPower(0);
                    direcaoAtual = 0;
                }
            } else if (gamepad1.x) {
                if(direcaoAtual <= 0 && intervalo.milliseconds() > 300) {
                    //tower.setPower(-power);
                    direcaoAtual = -1;
                } else {
                    //tower.setPower(0);
                    direcaoAtual = 0;
                }
            } else {
                //tower.setPower(0);
                direcaoAtual = 0;
                intervalo.reset();
            }

            telemetry.addLine("Pressione b ou x");
            telemetry.addData("Power:", power);
            if(direcaoAtual != 0) {
                telemetry.addLine("Estado do motor: Movendo");
            } else {
                telemetry.addLine("Estado do motor: Parado");
            }

            telemetry.update();
        }
    }

}

