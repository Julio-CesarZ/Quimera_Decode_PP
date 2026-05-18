package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Teste Single Motor", group = "TeleOp")
public class SingleMotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx tower = hardwareMap.get(DcMotorEx.class, "tower");

        double power = 1;

        boolean right = false;
        boolean left = false;

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.right_bumper && !right) {
                power =+ 0.1;
                power = Range.clip(power, 0.0, 1.0);
            }
            right = gamepad1.right_bumper;
            if(gamepad1.left_bumper && !left) {
                power =- 0.1;
                power = Range.clip(power, 0.0, 1.0);
            }
            left = gamepad1.left_bumper;

            if (gamepad1.b) {
                tower.setPower(-power);
            } else if (gamepad1.x) {
                tower.setPower(power);
            } else {
                tower.setPower(0);
            }

            telemetry.addLine("Pressione b ou x");
            telemetry.addData("Power:", power);

            telemetry.update();
        }
    }

}

