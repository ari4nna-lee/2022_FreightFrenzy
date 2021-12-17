package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Mode", group = "")
public class TestTeleOp extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    private DcMotor duckWheelMotor;
    private DcMotor armMotor;
    private DcMotor intakeMotor;

    private Servo armServo;

    @Override
    public void runOpMode() {
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        duckWheelMotor = hardwareMap.dcMotor.get("duckWheelMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        armServo = hardwareMap.servo.get("armServo");

        if (opModeIsActive()) {
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

            while (opModeIsActive()) {
                duckWheelMotor.setPower(gamepad2.right_trigger);
                armMotor.setPower(gamepad2.right_stick_y);
                intakeMotor.setPower(gamepad2.left_stick_y);

                if (gamepad1.a) {
                    armServo.setPosition(0);
                } else if (gamepad1.b) {
                    armServo.setPosition(1);
                }

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
                telemetry.addData("leftFront", lFront);
                telemetry.addData("rightFront", rFront);
                telemetry.addData("leftRear", lRear);
                telemetry.addData("rightRear", rRear);

                telemetry.addData("lFront Position", leftFront.getCurrentPosition());
                telemetry.addData("rFront Position", rightFront.getCurrentPosition());
                telemetry.addData("lBack Position", leftBack.getCurrentPosition());

                telemetry.update();
            }
        }
    }


}
