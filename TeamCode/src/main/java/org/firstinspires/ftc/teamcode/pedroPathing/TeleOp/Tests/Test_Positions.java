package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Positions MOTOR", group = "Tests")
public class Test_Positions extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "lf");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "rr");
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, "lr");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "rf");

        double p = 0.5;

        boolean intervalo_bumper = false;

        waitForStart();

        while (opModeIsActive()) {

            if ((gamepad1.right_bumper || gamepad1.left_bumper) && !intervalo_bumper) {
                    p += gamepad1.right_bumper ? 0.05 : -0.05;
                    p = Range.clip(p, 0, 1);
            }
            intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;

            if (gamepad1.a){
                lf.setPower(p);
            } else {
                if (!gamepad1.right_trigger_pressed) {
                    lf.setPower(0);
                }
            }

            if (gamepad1.b){
                lr.setPower(p);
            } else {
                if (!gamepad1.right_trigger_pressed) {
                    lr.setPower(0);
                }
            }

            if (gamepad1.x){
                rf.setPower(p);
            } else {
                if (!gamepad1.right_trigger_pressed) {
                    rf.setPower(0);
                }
            }

            if (gamepad1.y){
                rr.setPower(p);
            } else {
                if (!gamepad1.right_trigger_pressed) {
                    rr.setPower(0);
                }
            }

            if (gamepad1.right_trigger_pressed) {
                lf.setPower(p);
                lr.setPower(p);
                rf.setPower(p);
                rr.setPower(p);
            }

            telemetry.addLine("A = Esquerda da Frente lf");
            telemetry.addLine("B = Esquerda de Trás lr");
            telemetry.addLine("X = Direita da Frente rf");
            telemetry.addLine("Y = Direita de Trás rr");
            telemetry.addData("p", p);
            telemetry.addLine();
            telemetry.update();
        }
    }

}

