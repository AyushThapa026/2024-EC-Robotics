package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.*;

@Autonomous(name="Auto", group="Robot")
public class RobotAuto extends LinearOpMode {
    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  rearLeft    = null;
    public DcMotor  rearRight   = null;
    public Servo clawPush = null;
    public Servo clawDrop = null;
    public TouchSensor touchSensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.3;
    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
        frontLeft  = hardwareMap.dcMotor.get("left_front_drive");
        frontRight = hardwareMap.dcMotor.get("right_front_drive");
        rearLeft = hardwareMap.dcMotor.get("left_rear_drive");
        rearRight = hardwareMap.dcMotor.get("right_rear_drive");
        //clawPush = hardwareMap.servo.get("pixel_push");

        touchSensor = hardwareMap.touchSensor.get("touch_sensor");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                          frontLeft.getCurrentPosition(),
                          frontRight.getCurrentPosition(),
                          rearLeft.getCurrentPosition(),
                          rearRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runAutonomous();
    }

    private void turn(double radians){
        encoderDrive(TURN_SPEED, -(radians * 20.043971751969), (radians * 20.043971751969), 1000);
    }

    private void move(double inches){
        encoderDrive(DRIVE_SPEED, inches, inches, 30);
    }

    private double radiansToCounts(double r) {
        return (COUNTS_PER_MOTOR_REV / (2 * Math.PI)) * r;
    }

    public void pushPixel(){
        move(4);
        clawPush.setPosition(0); // TODO: Find the actual position
        move(-4);
    }

    private void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRearLeftTarget = rearLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRearRightTarget = rearRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            rearLeft.setTargetPosition(newRearLeftTarget);
            rearRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            rearLeft.setPower(Math.abs(speed));
            rearRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontLeft.isBusy() && frontRight.isBusy() && rearRight.isBusy() && rearLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", frontLeft.getTargetPosition(),  newFrontLeftTarget);
                //telemetry.addData("Runtime: ", runtime.seconds());

                telemetry.addData("Currently at",  " at %7d :%7d",
                                            frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), rearRight.getCurrentPosition(), rearLeft.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontRight.setPower(0);
            frontLeft.setPower(0);
            rearRight.setPower(0);
            rearLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    private boolean isTouching() { // returns if the touch sensor is pressed
        return touchSensor.isPressed();
    }

    private void findTeamObject() {
        move(14);
        turn(Math.PI/2.62);
        move(6);
        turn(-Math.PI/2.62);
        move(12);
        if(isTouching()==true){

            turn(-Math.PI/2.62);
            move(6);
            turn(Math.PI/2.62);
            move(-24);
        }else{
            if(isTouching()==true){

                turn(-Math.PI/2.62);
                turn(-Math.PI/2.62);
                move(6);
                turn(-Math.PI/2.62);
                move(-24);
            }else{

                turn(-Math.PI/2.62);
                move(6);
                turn(-Math.PI/2.62);
                move(-24);
            }
        }

    }

    private void pickTeamObject() {

    }

    private void goToBackboard() {

    }

    private void placePixelOnBackboard() {

    }

    private void moveOutOfWay() {

    }

    private void runAutonomous() {
        // Locate team object
        findTeamObject();
        // Pick up team object

        // Go to backboard
        goToBackboard();
        // Place pixel on backboard
        placePixelOnBackboard();
        // Move out of the way
        moveOutOfWay();
    }
}
