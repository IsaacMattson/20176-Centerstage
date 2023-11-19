package org.firstinspires.ftc.teamcode.auto;

// NOTE: THIS ONLY IS GOOD FOR LEFT BLUE START

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class TestTfod extends LinearOpMode {

    // Cool ass Variables
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private DcMotor extend = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private Servo rotator = null;

    @Override
    public void runOpMode() {

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Declaring things, self explanatory
        leftFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorFour");
        leftBackDrive = hardwareMap.get(DcMotor.class, "driveMotorOne");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "driveMotorThree");
        rightBackDrive = hardwareMap.get(DcMotor.class, "driveMotorTwo");
        arm = hardwareMap.get(DcMotor.class, "armMotorOne");
        extend = hardwareMap.get(DcMotor.class, "armMotorTwo");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rotator = hardwareMap.get(Servo.class, "rotator");

        // Wheel Behaviour
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm Things
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Status", "Initialized");
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // START OF MOVEMENT CODE, ROBOT MUST START FACING SCOREBOARD THING


                forwardDrive(50); // align self with left blue strip
                rightShift(100); // move closer

                sleep(1000); // sleep for One second, give time for Tflow?
                if (tfod.getRecognitions().size() > 0){ // Condition 1, Prop is on LEFT strip.
                    // TODO: Code related to putting orange pixel on LEFT strip and then moving to standard position

                } else{ // Not there, so move to middle
                    backwardsDrive(50); // Must be the same "ms" value as fDrive() earlier
                    rightShift(100); // move even closer

                    sleep(1000); // sleep for One second, give time for Tflow
                    if (tfod.getRecognitions().size() > 0){ // Condition 2, Prop is on MIDDLE strip.
                        // TODO: Code related to putting orange pixel on MIDDLE strip and then moving to standard position
                    } else{ // Last Condition, prop is on RIGHT strip
                        // TODO: Code related to putting orange pixel on RIGHT strip and then moving to standard position
                    }
                }

                /* We now placed our purple (or Orange???) pixel, and are in a position common to all 3 paths. */





                /* Share the CPU.
                sleep(20); // We *maybe* need this, idk */
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()


    private void forwardDrive(int ms){
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(1);
        leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(1);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void backwardsDrive(int ms){
        leftBackDrive.setPower(-1);
        rightBackDrive.setPower(-1);
        leftFrontDrive.setPower(-1);
        rightFrontDrive.setPower(-1);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void rightShift(int ms){
        leftBackDrive.setPower(0.4);
        rightBackDrive.setPower(-0.4);
        leftFrontDrive.setPower(-0.4);
        rightFrontDrive.setPower(0.4);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void leftShift(int ms){
        leftBackDrive.setPower(-0.4);
        rightBackDrive.setPower(0.4);
        leftFrontDrive.setPower(0.4);
        rightFrontDrive.setPower(-0.4);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void rightTurn(int ms){ // Not sure what values to make negative, will test
        leftBackDrive.setPower(0.4);
        rightBackDrive.setPower(-0.4);
        leftFrontDrive.setPower(0.4);
        rightFrontDrive.setPower(-0.4);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

    private void leftTurn(int ms){
        leftBackDrive.setPower(-0.4);
        rightBackDrive.setPower(0.4);
        leftFrontDrive.setPower(-0.4);
        rightFrontDrive.setPower(0.4);
        sleep(ms);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
    }

}   // end class
