package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Tele Op", group = "")
public class FreightFrenzyTeleOp extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack, intakeMotor, pivotMotor, duckWheel;
    private final double DUCK_WHEEL_SPEED = 0.8;

    @Override
    public void runOpMode() {
        initHardwareMap();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive() || !isStopRequested()) {
                mecanumDrive();

                pivotMotor.setPower(-gamepad2.right_stick_y);

                if (gamepad2.right_trigger > 0) {
                    intakeMotor.setPower(-gamepad2.right_trigger);
                } else if (gamepad2.left_trigger > 0) {
                    intakeMotor.setPower(gamepad2.left_trigger);
                }
                if (gamepad1.x) {
                    while (gamepad1.x || !isStopRequested()) {
                        duckWheel.setPower(DUCK_WHEEL_SPEED);
                    }

                    duckWheel.setPower(0);
                }

                telemetry.addData("Position of the arm", pivotMotor.getCurrentPosition());
                telemetry.update();

            }
        }
    }

    public void initHardwareMap() {
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
        duckWheel = hardwareMap.dcMotor.get("duckWheel");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void mecanumDrive() {
        double r = Math.hypot(gamepad1.left_stick_x * -1, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x * -1) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x * -1;

        double lFront = r * Math.cos(robotAngle) + rightX;
        double rFront = r * Math.sin(robotAngle) - rightX;
        double lRear = r * Math.sin(robotAngle) + rightX;
        double rRear = r * Math.cos(robotAngle) - rightX;
        leftFront.setPower(lFront);
        rightFront.setPower(rFront);
        leftBack.setPower(lRear);
        rightBack.setPower(rRear);

        double strafe_left_power = gamepad1.left_trigger;
        double strafe_right_power = gamepad1.right_trigger;

        if (gamepad1.left_trigger > 0) {
            leftFront.setPower(strafe_left_power);
            rightFront.setPower(-strafe_left_power);
            leftBack.setPower(-strafe_left_power);
            rightBack.setPower(strafe_left_power);
        }
        if (gamepad1.right_trigger > 0) {
            leftFront.setPower(-strafe_right_power);
            rightFront.setPower(strafe_right_power);
            leftBack.setPower(strafe_right_power);
            rightBack.setPower(-strafe_right_power);
        }
    }
    public void mecanumDriveTwo() {
        double motor_power = gamepad1.left_stick_y;
        if (gamepad1.left_stick_y != 0) {
            leftFront.setPower(motor_power);
            rightFront.setPower(motor_power);
            leftBack.setPower(motor_power);
            rightBack.setPower(motor_power);
        }

        double strafe_left_power = gamepad1.left_trigger;
        double strafe_right_power = gamepad1.right_trigger;

        if (gamepad1.left_trigger > 0) {
            leftFront.setPower(-strafe_left_power);
            rightFront.setPower(strafe_left_power);
            leftBack.setPower(strafe_left_power);
            rightBack.setPower(-strafe_left_power);
        }
        if (gamepad1.right_trigger > 0) {
            leftFront.setPower(strafe_right_power);
            rightFront.setPower(-strafe_right_power);
            leftBack.setPower(-strafe_right_power);
            rightBack.setPower(strafe_right_power);
        }

        double turn_power = gamepad2.right_stick_x;

        if (gamepad2.right_stick_x > 0) {
            leftFront.setPower(turn_power);
            rightFront.setPower(-turn_power);
            leftBack.setPower(turn_power);
            rightBack.setPower(-turn_power);
        } else if (gamepad1.right_stick_x < 0) {
            leftFront.setPower(-turn_power);
            rightFront.setPower(turn_power);
            leftBack.setPower(-turn_power);
            rightBack.setPower(turn_power);
            }


    }
}

