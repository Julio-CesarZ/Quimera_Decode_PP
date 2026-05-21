package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp (name = "TeleOp / 1 player", group = "TeleOp")
public class TeleOp_Completo extends LinearOpMode {

    boolean intervalo_a = false;
    boolean intervalo_b = false;
    boolean intervalo_y = false;
    boolean intervalo_x = false;
    boolean intervalo_RT = false;
    boolean intervalo_LT = false;
    boolean intervalo_bumper = false;
    boolean intervalo_bpad_up = false;
    boolean intervalo_bpad_right = false;
    boolean intervalo_bpad_left = false;
    boolean intervalo_bpad_down = false;
    boolean intakeF = false;
    boolean reverse = false;
    boolean reverseL = false;
    boolean lF = false;
    boolean targetVisible;
    ElapsedTime temporizadorPulsoIntake = new ElapsedTime();

    // ^^ booleans para as funções de intervalo do robô ^^
    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private double slowModeMultiplier = 0.5;
    private double shotP = 2000;
    private double intakeP = 1;
    private double towerP = 0.5;
    double kP = 0.08;
    double limitPP = 1;
    double limitPL = 1;
    private int change = 0;
    private int target = 0;
    private String changeM = "Movimentação";
    // ^^ implementando motores e outros componentes do robô ^^
    private double tick_intervalo = 0;
    private double intervalo = 200;
    ElapsedTime elapsedIntervalo = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(0, 0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();


        follower.startTeleopDrive();

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx l_right = hardwareMap.get(DcMotorEx.class, "l_right");
        DcMotorEx l_left = hardwareMap.get(DcMotorEx.class, "l_left");
        DcMotorEx tower = hardwareMap.get(DcMotorEx.class, "tower");

        Servo s1 = hardwareMap.get(Servo.class, "s1");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        l_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        l_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setDirection(DcMotor.Direction.FORWARD);
        l_right.setDirection(DcMotor.Direction.REVERSE);
        l_left.setDirection(DcMotor.Direction.FORWARD);

        tower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tower.setTargetPosition(0);
        tower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(50);
        limelight.start();
        limelight.pipelineSwitch(1); //1 -> oficial 2 -> teste

        elapsedIntervalo.reset();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            targetVisible = (result != null && result.isValid());

            follower.update();

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true
            );

            // Gamepads do intakes e lançadores
            if (gamepad1.a && !intakeF && !intervalo_a) {
                intakeF = !intakeF;
            } else if (gamepad1.a && intakeF && !intervalo_a) {
                intake.setPower(0);
                intakeF = !intakeF;
            }
            intervalo_a = gamepad1.a;

            if (gamepad1.b && !reverse && !intervalo_b) {
                reverse = !reverse;
            } else if (gamepad1.b && reverse && !intervalo_b) {
                intake.setPower(0);
                l_right.setPower(0);
                l_left.setPower(0);
                reverse = !reverse;
            }
            intervalo_b = gamepad1.b;

            if (gamepad1.right_trigger > 0.3 && !lF && !intervalo_RT) {
                lF = !lF;
            } else if (gamepad1.right_trigger > 0.3 && lF && !intervalo_RT) {
                l_right.setVelocity(0);
                l_left.setVelocity(0);
                lF = !lF;
            }
            intervalo_RT = gamepad1.right_trigger > 0.3;

            if (gamepad1.left_trigger > 0.3 && !reverseL) {
                reverseL = !reverseL;
            }
            intervalo_LT = gamepad1.left_trigger > 0.3;

            if (gamepad1.dpad_up && !intervalo_bpad_up) {

            }
            intervalo_bpad_up = gamepad1.dpad_up;

            int limiteRotativo = 280;
            if (!targetVisible) {
                if (gamepad1.dpad_left && target > -limiteRotativo && elapsedIntervalo.milliseconds() >= tick_intervalo) {
                    target -= 1;
                    encoder(tower, target, towerP);
                    tick_intervalo = elapsedIntervalo.milliseconds() + intervalo;
                } else if (gamepad1.dpad_right && target < limiteRotativo && elapsedIntervalo.milliseconds() >= tick_intervalo) {
                    target += 1;
                    encoder(tower, target, towerP);
                    tick_intervalo = elapsedIntervalo.milliseconds() + intervalo;
                } else if (gamepad1.dpad_down && !intervalo_bpad_down) {
                    target = 0;
                    encoder(tower, target, towerP);
                }
                intervalo_bpad_down = gamepad1.dpad_down;
            } else {
                double tx = result.getTx();
                double ta = result.getTa();

                if (Math.abs(tx) > 1.0 && ta <= 1.5) {

                    target = tower.getCurrentPosition() + (int)(tx * 2); // menor -> sensibilidade maior

                    target = Math.max(-limiteRotativo, Math.min(limiteRotativo, target));

                    double power = Math.abs(tx) * kP;
                    power = Math.max(-limitPL, Math.min(limitPL, power));

                    encoder(tower, target, power);
                } else if (Math.abs(tx) > 1.0 && ta > 1.5) {

                    target = tower.getCurrentPosition() + (int)(tx * 2); // menor -> sensibilidade maior

                    target = Math.max(-limiteRotativo, Math.min(limiteRotativo, target));
                    //

                    double power = Math.abs(tx) * kP;
                    power = Math.max(-limitPP, Math.min(limitPP, power));

                    encoder(tower, target, power);
                }
            }

            if (gamepad1.x && change == 0 && !intervalo_x) {
                change = 1;
                changeM = "Intake";
            } else if (gamepad1.x && change == 1 && !intervalo_x) {
                change = 2;
                changeM = "Lançador";
            } else if(gamepad1.x && change == 2 && !intervalo_x) {
                change = 3;
                changeM = "Torre";
            } else if (gamepad1.x && change == 3 && !intervalo_x) {
                change = 0;
                changeM = "Movimentação";
            }
            intervalo_x = gamepad1.x;

            if (change == 0) {
                if (gamepad1.right_bumper && !intervalo_bumper) {
                    slowModeMultiplier += 0.05;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    slowModeMultiplier -= 0.05;
                }
                slowModeMultiplier = Range.clip(slowModeMultiplier, 0.0, 1.0);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            } else if (change == 1) {
                if (gamepad1.right_bumper && !intervalo_bumper) {
                    intakeP += 0.05;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    intakeP -= 0.05;
                }
                intakeP = Range.clip(intakeP, 0.0, 1.0);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            } else if (change == 2) {
                if (gamepad1.right_bumper && !intervalo_bumper) {
                    shotP += 100;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    shotP -= 100;
                }
                shotP = Range.clip(shotP, 0.0, 2800);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            } else if (change == 3) {
                if (gamepad1.right_bumper && !intervalo_bumper) {
                    towerP += 0.05;
                } else if (gamepad1.left_bumper && !intervalo_bumper) {
                    towerP -= 0.05;
                }
                towerP = Range.clip(towerP, 0.0, 1.0);

                intervalo_bumper = gamepad1.right_bumper || gamepad1.left_bumper;
            }

            if (intakeF) {
                intake.setPower(intakeP);
            }
            if (reverse) {
                intake.setPower(-intakeP * 0.5);
                l_right.setPower(-shotP * 0.5);
                l_left.setPower(-shotP * 0.5);
            }
            if (lF) {
                l_right.setVelocity(shotP);
                l_left.setVelocity(shotP);
            }

            // Telemetria para mostrar a potência dos Motores e Intakes
            telemetry.addLine("Valores de Potência");
            telemetry.addLine();
            telemetry.addData("Troca de poder atual", changeM);
            telemetry.addData("Chassi Power", slowModeMultiplier);
            telemetry.addData("Intake Power", intakeP);
            telemetry.addData("Shot Power", shotP);
            telemetry.addData("Tower Power", towerP);
            telemetry.addLine();
            telemetry.addLine("Valores de Posição");
            telemetry.addLine();
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine("Lógica da câmera");
            telemetry.addLine();
            if (targetVisible) {
                telemetry.addData("Alvo Detectado", "Sim");
                telemetry.addData("TX (Graus)", result.getTx());
                telemetry.addData("TY (Graus)", result.getTy());
                telemetry.addData("TA (Área)", result.getTa());
            } else {
                telemetry.addData("Alvo Detectado", "Não");
            }
            telemetry.addData("Posição Atual", tower.getCurrentPosition());
            telemetry.addData("Alvo Encoder", target);

            telemetry.update();
        }
    }

    private void encoder(DcMotor motor, int novoAlvo, double power) {
        motor.setTargetPosition(novoAlvo);
        motor.setPower(power);
    }
}

