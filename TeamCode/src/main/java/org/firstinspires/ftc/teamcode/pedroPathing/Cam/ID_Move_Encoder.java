package org.firstinspires.ftc.teamcode.pedroPathing.Cam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name = "ID_Encoder_CameraTest", group = "Cam")
public class ID_Move_Encoder extends LinearOpMode {
    private HuskyLens huskyLens;
    //definir a programação da huskylens

    private DcMotorEx hex;


    private final int READ_PERIOD = 50;

    @Override
    public void runOpMode() {
        // "huskylens" must match the name in your Robot Configuration
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        hex = hardwareMap.get(DcMotorEx.class, "hex1");

        int KDistance = 5400;
        int x = 0;
        int y = 0;

        double ticksPorPixel = 0.25;

        double pw = 0;

        boolean guide = false;

        int target = 0;

        int erroAnterior = 0;

        double kP = 0.005;
        double kD = 0.01;

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.MILLISECONDS);
        rateLimit.expire();

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        hex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hex.setTargetPosition(0);

        hex.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Check if the device is connected
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "HuskyLens not found!");
        } else {
            telemetry.addData(">>", "HuskyLens connected.");
        }

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                int Distance = KDistance / blocks[i].width;
                x = blocks[i].x;
                y = blocks[i].y;
                telemetry.addData("Distance in CM: ", Distance);
                telemetry.addData("x: ", x); //160 centro
                telemetry.addData("y: ", y); //120 centro
            }

            telemetry.addData("Alvo: ", target);

            if(gamepad1.right_trigger > 0.3 && !guide) {
                guide = true;
            } else if(gamepad1.right_trigger > 0.3 && guide) {
                guide = false;
            }

            telemetry.addData("AutoCam: ", guide);

            if(!guide) {

                pw = 0.2;

                if(gamepad1.right_bumper && target > -100) {
                    target = target - 10;
                    encoder(hex, target, pw);
                    sleep(200);
                } else if(gamepad1.left_bumper && target < 100) {
                    target = target + 10;
                    encoder(hex, target, pw);
                    sleep(200);
                } else if(gamepad1.b) {
                    target = 0;
                    encoder (hex, target, 0.1);
                    sleep(200);
                }
            } else if (guide && blocks.length > 0) {
                int erroX = 160 - blocks[0].x;

                int derivada = erroX - erroAnterior;

                if (Math.abs(erroX) > 10 && target > -100 && target < 100) {

                    int posAtual = hex.getCurrentPosition();

                    int novoAlvo = posAtual + (int)(erroX * ticksPorPixel);

                    target = novoAlvo;

                    //double power = (erroX * kP) + (derivada * kD);

                    double power = erroX * kP;

                    erroAnterior = erroX;

                    pw = Math.max(-0.15, Math.min(0.15, power));

                    encoder(hex, target, pw);

                } else if(target <= -100 || target >= 100) {
                    target = 0;
                    encoder(hex, target, 0.3);

                } else {
                    hex.setPower(0);
                }
            } else {
                hex.setPower(0);
            }

            telemetry.update();

        }

    }

    private void encoder(DcMotor motor, int novoAlvo, double power) {
        motor.setTargetPosition(novoAlvo);
        motor.setPower(power);
    }


}