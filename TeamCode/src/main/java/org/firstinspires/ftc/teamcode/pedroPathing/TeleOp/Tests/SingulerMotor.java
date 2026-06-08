package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Singular MOTOR", group = "TeleOp")
public class SingulerMotor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rr");
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, "lr");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf");

        double power = 0.8;

        boolean right = false;
        boolean left = false;

        int direcaoAtual = 0;

        ElapsedTime intervalo = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                lf.setPower(1);
            } else {
                lf.setPower(0);
            }

            if (gamepad1.b){
                lr.setPower(1);
            } else {
                lr.setPower(0);
            }

            if (gamepad1.x){
                rf.setPower(1);
            } else {
                rf.setPower(0);
            }

            if (gamepad1.y){
                rr.setPower(1);
            } else {
                rr.setPower(0);
            }

            telemetry.addLine("A = Esquerda da Frente Lf");
            telemetry.addLine("B = Esquerda de Trás Lr");
            telemetry.addLine("X = Direita da Frente rf");
            telemetry.addLine("Y = Direita de Trás Lf");
            telemetry.addLine();
            telemetry.update();
        }
    }

}

