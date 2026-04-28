package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp (name = "Move / Omni_Mover", group = "TeleOp")
public class Omni_Move extends LinearOpMode {

    boolean press = false;
    boolean pressr = false;
    boolean pressr1 = false;
    boolean pressr2 = false;
    boolean tdc = false;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx c1 = hardwareMap.get(DcMotorEx.class, "c1");
        DcMotorEx c2 = hardwareMap.get(DcMotorEx.class, "c2");
        DcMotorEx c3 = hardwareMap.get(DcMotorEx.class,"c3");
        DcMotorEx c4 = hardwareMap.get(DcMotorEx.class,"c4");

        double currentPowerBase = 1;

        //follower.startTeleopDrive();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a && !press) {
                c1.setPower(1);
                press = true;
                sleep(100);
            } else if(gamepad1.a && press) {
                c1.setPower(0);
                press = false;
                sleep(100);
            }

            if(gamepad1.b && !pressr) {
                c1.setPower(-1);
                pressr = true;
                sleep(100);
            } else if(gamepad1.b && pressr) {
                c1.setPower(0);
                pressr = false;
                sleep(100);
            }

            if(gamepad1.a && !tdc){
                c3.setPower(1);
                tdc = true;
                sleep(100);
            } else if(gamepad1.a && tdc) {
                c3.setPower(0);
                tdc = false;
                sleep(100);
            }

            if(gamepad1.y && !pressr1) {
                c2.setPower(1);
                c3.setPower(1);
                pressr1 = true;
                sleep(100);
            } else if(gamepad1.y && pressr1) {
                c2.setPower(0);
                c3.setPower(0);
                pressr1 = false;
                sleep(100);
            }

            if(gamepad1.x && !pressr2) {
                c2.setPower(-1);
                c3.setPower(-1);
                pressr2 = true;
                sleep(100);
            } else if(gamepad1.x && pressr2) {
                c2.setPower(0);
                c3.setPower(0);
                pressr2 = false;
                sleep(100);
            }

        }


    }





}