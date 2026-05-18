package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "Teste Single Motor", group = "TeleOp")
public class SingleMotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx tower = hardwareMap.get(DcMotorEx.class, "tower");

        double power = 0.5;

        boolean right = false;
        boolean left = false;

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.right_bumper && !right) {
                power =+ 1;
            }
            right = gamepad1.right_bumper;
            if(gamepad1.left_bumper && !left) {
                power =- 1;
            }
            left = gamepad1.left_bumper;

            if (gamepad1.b) {
                tower.setPower(-power);
            } else if (gamepad1.x) {
                tower.setPower(power);
            } else {
                tower.setPower(0);
            }
        }
    }

}

