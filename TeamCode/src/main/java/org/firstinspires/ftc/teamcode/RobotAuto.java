package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.tensorflow.lite.task.vision.detector.Detection;

import java.util.*;
@Autonomous(name="Auto", group="Robot")
public class RobotAuto extends LinearOpMode {

    /* Declare OpMode members. */

    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  rearLeft    = null;
    public DcMotor  rearRight   = null;
    public DcMotor leftLinearSlide = null;
    public DcMotor rightLinearSlide = null;
    public DcMotor upperArmJoint = null;
    public DcMotor lowerArmJoint = null;
    public DcMotor rootArmJoint = null;
    public Servo clawPush = null;
    public Servo clawDrop = null;

    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    private AprilTagProcessor aprilTag;

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.3;

    BNO055IMU imu;

    Orientation angles;

    @Override
    public void runOpMode() {

            VisionClass.initAprilTag(hardwareMap.get(WebcamName.class, "Webcam 1"));

            // Wait for the DS start button to be touched.
            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();

            if (opModeIsActive()) {
                    while (opModeIsActive()) {

                            VisionClass.AprilTag();

                            // Push telemetry to the Driver Station.
                            telemetry.update();

                            // Save CPU resources; can resume streaming when needed.
                            if (gamepad1.dpad_down) {
                                    visionPortal.stopStreaming();
                            } else if (gamepad1.dpad_up) {
                                    visionPortal.resumeStreaming();
                            }

                            // Share the CPU.
                            sleep(20);
                    }
            }

            // Save more CPU resources when camera is no longer needed.
            visionPortal.close();

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

            imu = hardwareMap.get(BNO055IMU.class, "ROBOT");
            imu.initialize(parameters);

            // Initialize the drive system variables.
            frontLeft  = hardwareMap.dcMotor.get("left_front_drive");
            frontRight = hardwareMap.dcMotor.get("right_front_drive");
            rearLeft = hardwareMap.dcMotor.get("left_rear_drive");
            rearRight = hardwareMap.dcMotor.get("right_rear_drive");
            //upperArmJoint = hardwareMap.get(DcMotor.class, "arm_upper_joint");
            //lowerArmJoint = hardwareMap.get(DcMotor.class, "arm_lower_joint");

            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            rearLeft.setDirection(DcMotor.Direction.REVERSE);
            rearRight.setDirection(DcMotor.Direction.FORWARD);

            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //upperArmJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //lowerArmJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //upperArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //lowerArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            tfod = TfodProcessor.easyCreateWithDefaults();
            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
            visionPortal.stopLiveView();

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Starting at",  "%7d :%7d",
            frontLeft.getCurrentPosition(),
            frontRight.getCurrentPosition(),
            rearLeft.getCurrentPosition(),
            rearRight.getCurrentPosition());
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            //encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
            //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            //encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
            //sleep(500);
            //move(30);
        /*
        turn(Math.PI/2);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
        while(opModeIsActive()){
            telemetry.addData("pixelInFront: ", Boolean.toString(objectInFront()));
            telemetry.update();
        }
         */

            auto();
            }


        /**
         * Add telemetry about AprilTag detections.
         */
    public void turn(double radians){
            encoderDrive(TURN_SPEED, -(radians * 20.043971751969), (radians * 20.043971751969), 1000);
            }

    public void turnRight(){
            turn(Math.PI/2);
            }

    public void turnLeft(){
            turn(-Math.PI/2);
            }

    public void move(double inches){
            encoderDrive(DRIVE_SPEED, inches, inches, 30);
            }

    private double radiansToCounts(double r) {
            return (COUNTS_PER_MOTOR_REV / (2 * Math.PI)) * r;
            }

    public void updateAngles(){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //angles.firstAngle is the heading angle
            }

    public void encoderStrafe(double speed,
            double inches, // negative is left, positive is right, probably
            double timeoutS) {
            int newFrontLeftTarget;
            int newFrontRightTarget;
            int newRearLeftTarget;
            int newRearRightTarget;

            // Ensure that the OpMode is still active
            if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRearLeftTarget = rearLeft.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newRearRightTarget = rearRight.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

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

            //sleep(10000);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
            (runtime.seconds() < timeoutS) &&
            (frontLeft.isBusy() && frontRight.isBusy() && rearRight.isBusy() && rearLeft.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to",  " %7d :%7d", frontLeft.getTargetPosition(),  newFrontLeftTarget);
            //telemetry.addData("Runtime: ", runtime.seconds());
            telemetry.addData("front left power: ", frontLeft.getPower());
            telemetry.addData("front right power: ", frontRight.getPower());
            telemetry.addData("rear left power: ", rearLeft.getPower());
            telemetry.addData("rear right power: ", rearRight.getPower());

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

    public void strafe(int inches){
            encoderStrafe(0.3, inches, 1000);
            }

    /*
    public void encoderArm(double upperJointRadians, double lowerJointRadians, double speed,
                           double timeoutS) {
        if (opModeIsActive()) {
            int upperJointCounts = (int) radiansToCounts(upperJointRadians);
            int lowerJointCounts = (int) radiansToCounts(lowerJointRadians);

            int targetUpperArmPosition = upperArmJoint.getCurrentPosition() + upperJointCounts;
            int targetLowerArmPosition = lowerArmJoint.getCurrentPosition() + lowerJointCounts;
            upperArmJoint.setTargetPosition(targetUpperArmPosition);
            lowerArmJoint.setTargetPosition(targetLowerArmPosition);

            upperArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lowerArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            upperArmJoint.setPower(Math.abs(speed));
            lowerArmJoint.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && rearRight.isBusy() && rearLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", upperJointCounts,  lowerJointCounts);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.update();
            }

            upperArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lowerArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
     */

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void pushPixel(){

            }

    public void dropPixel(){

            }
    public void encoderDrive(double speed,
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

            //sleep(10000);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
            (runtime.seconds() < timeoutS) &&
            (frontLeft.isBusy() && frontRight.isBusy() && rearRight.isBusy() && rearLeft.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Running to",  " %7d :%7d", frontLeft.getTargetPosition(),  newFrontLeftTarget);
            //telemetry.addData("Runtime: ", runtime.seconds());
            telemetry.addData("front left power: ", frontLeft.getPower());
            telemetry.addData("front right power: ", frontRight.getPower());
            telemetry.addData("rear left power: ", rearLeft.getPower());
            telemetry.addData("rear right power: ", rearRight.getPower());

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


    /*    public void placeObject(){
        //TODO: Finish this
    }

    public void moveServo(){
        //TODO: Finish this
    }
*/
    public void auto(){
            int pos;
            move(18);
            if (VisionClass.objectInFront(telemetry, tfod)){
            pos = 1;
            pushPixel();
            turn(-Math.PI/2);
            move(23);
            } else {
            turn(-Math.PI/2);
            move(22);
            turn(Math.PI/2);
            if(VisionClass.objectInFront(telemetry, tfod)){
            pos = 2;
            pushPixel();
            turn(-Math.PI/2);
            } else {
            move(13);
            turn(Math.PI/2);
            move(6);
            pos = 3;
            pushPixel();
            turn(-Math.PI/2);
            move(-13);
            turn(-Math.PI/2);
            move(6);
            }

            }

            }}
