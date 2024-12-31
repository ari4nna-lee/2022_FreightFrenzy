package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

import java.util.List;

@Disabled
@Autonomous(name = "RED Auto", group = "Autonomous Modes")
public class RedAutonomous extends LinearOpMode {

    private DcMotor pivotMotor, intakeMotor;
    private Servo cameraServo;

    private static final double SERVO_FIRST_POS = 0.45;
    private static final double SERVO_SECOND_POS = 0.525;

    private int TSM_Position;

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/RedCupTSE.tflite";
    private static final String[] LABELS = {
            "Shipping element", "Shippi", "Shipping elem"
    };
    private static final String VUFORIA_KEY =
            "AUgLF0r/////AAABme3TJAJ9xkFXqQanGuvAG80MUHIB7KLDWdGB2wR9lFc/TbFj4oDCEw3YsiLBQwzd6LAqyKG0yKPlrOQxINXiyrttKI3uG8Fs5zuAnIMw+ebKxkg7oLLQCJeFBxu9pjYyu8hE3Tg7kmTJSA0JRGaZYGoBllbNlmAXrjdcNfhYL5WQL3tFwaIs4PLOla8JjMofAdCZvoSmtGYXzBhreJoIkD7SX+dldjI3yEWsoLN8E9Xex8YRSRbylJivW8dxxzWcfPvqsunn8Vbf4ZkaPQD9cbPomyHI3ooLSLad6CYMgnuuLTep990W0jrX+QpxAK8cQzM7CPDKwV/un3xgkS1gmvJuYYp2LyML21CSCvCrsTvk";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    Pose2d startPose = new Pose2d(-40, -62, Math.toRadians(180));

    @Override
    public void runOpMode() {
        initHardwareMap();
        initVuforia();
        initTfod();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);
        Trajectory move_away = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-13, -52))
                .build();

        Trajectory first_level = drive.trajectoryBuilder(move_away.end())
                .lineToSplineHeading(new Pose2d(-6, -37, Math.toRadians(90)))
                .build();

        Trajectory secondthird_level = drive.trajectoryBuilder(move_away.end())
                .lineToSplineHeading(new Pose2d(-6, -40, Math.toRadians(90)))
                .build();
        /*
        Trajectory return_to_position = drive.trajectoryBuilder(second_level.end())
                .lineToSplineHeading(new Pose2d(-40, -60, Math.toRadians(180)))
                .build();
         */
        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(2, 16.0 / 9.0);
        }

        cameraServo.setPosition(SERVO_FIRST_POS);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        detectTeamShippingElement();
        drive.followTrajectory(move_away);
        if (TSM_Position == 1) {
            drive.followTrajectory(first_level);
            ElapsedTime elapsedTime = new ElapsedTime();
            while (elapsedTime.milliseconds() <= 1500 && !isStopRequested()) {
                intakeMotor.setPower(-1);
            }
            intakeMotor.setPower(0);
        } else if (TSM_Position == 2) {
            drive.followTrajectory(secondthird_level);
            while (pivotMotor.getCurrentPosition() <= 620 && !isStopRequested()) {
                pivotMotor.setPower(1);
            }
            pivotMotor.setPower(0);
            ElapsedTime elapsedTime = new ElapsedTime();
            while (elapsedTime.milliseconds() <= 1500 && !isStopRequested()) {
                intakeMotor.setPower(-1);
            }
            intakeMotor.setPower(0);
        } else {
            drive.followTrajectory(secondthird_level);
            while (pivotMotor.getCurrentPosition() <= 1100 && !isStopRequested()) {
                pivotMotor.setPower(1);
            }
            pivotMotor.setPower(0);
            ElapsedTime elapsedTime = new ElapsedTime();
            while (elapsedTime.milliseconds() <= 1500 && !isStopRequested()) {
                intakeMotor.setPower(-0.5);
            }
            intakeMotor.setPower(0);
        }

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("TSM", TSM_Position);
            telemetry.addData("ARM POSITION", pivotMotor.getCurrentPosition());
            telemetry.update();
        }


        telemetry.update();
    }


    private void initHardwareMap() {
        pivotMotor = hardwareMap.dcMotor.get("pivotMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        cameraServo = hardwareMap.servo.get("cameraServo");

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void detectTeamShippingElement() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() != 0) {
                    telemetry.addData("TFOD", "TSM in first position");
                    TSM_Position = 1;
                    telemetry.update();
                } else if (updatedRecognitions.size() == 0) {
                    cameraServo.setPosition(SERVO_SECOND_POS);
                    sleep(2000);
                    List<Recognition> updatedRecognitions_2 = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions_2.size() >= 1) {
                        telemetry.addData("# Object Detected", updatedRecognitions_2.size());
                        telemetry.addData("TFOD", "TSM in second position");
                        TSM_Position = 2;
                        telemetry.update();
                    } else {
                        telemetry.addData("# Object Detected", updatedRecognitions_2.size());
                        telemetry.addData("TFOD", "TSM in third position");
                        TSM_Position = 3;
                        telemetry.update();
                    }


                }

            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}
