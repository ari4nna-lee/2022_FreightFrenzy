package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MECANUM DRIVE", group = "")
public class MecanumOp extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor leftFront;
    private DcMotor leftBack;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE); // Actual robot
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE); // Program bot
            // Put run blocks here.

            while (opModeIsActive()) {

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
}
