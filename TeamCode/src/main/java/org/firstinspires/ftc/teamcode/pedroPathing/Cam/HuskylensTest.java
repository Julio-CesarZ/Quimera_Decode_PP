package org.firstinspires.ftc.teamcode.pedroPathing.Cam;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@TeleOp (name = "HuskylensTEst", group = "Test")
public class HuskylensTest extends LinearOpMode {
    private HuskyLens huskylens;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx lf = hardwareMap.get(DcMotorEx.class,"lf");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class,"rf");
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class,"lr");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class,"rr");
        lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        huskylens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        int Kdistance = 5400; //Possivel distancia do robô com a Apriltag (???) 
        int x = 0;
        int y = 0;   //Valor de distancia

        double kP = 0.005;
        double kD = 0.01;       //REVISAR ESSE PARTE

        double tickspPixel = 0.25;  //Valor de movimento por ticks

        boolean follow = false;

        int target = 0;

        while (!isStarted() && !isStopRequested()) {

        }

        while (opModeIsActive()) {

            if (!huskylens.knock()) {
                telemetry.addData(">>", "HuskyLens not found!");
            } else {
                telemetry.addData(">>", "HuskyLens connected.");
            }

            telemetry.addData("Alvo: ", target);
            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);

            telemetry.update();
        }

        waitForStart();
    }
}
