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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7*4;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.3;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft  = hardwareMap.get(DcMotor.class, "left_front_drive");
        frontRight = hardwareMap.get(DcMotor.class, "right_front_drive");
        rearLeft = hardwareMap.get(DcMotor.class, "left_rear_drive");
        rearRight = hardwareMap.get(DcMotor.class, "right_rear_drive");
        //upperArmJoint = hardwareMap.get(DcMotor.class, "arm_upper_joint");
        //lowerArmJoint = hardwareMap.get(DcMotor.class, "arm_lower_joint");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

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
<<<<<<< Updated upstream
        upperArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
=======
        //upperArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
>>>>>>> Stashed changes
        //lowerArmJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
<<<<<<< Updated upstream
        sleep(500);
        move(30);
        //sleep(100);
        turn(Math.PI);
        //sleep(100);
        move(30);

=======

        timeDrive(0.05, 12);
        sleep(5000);
        timeDrive(0.05, -12);
>>>>>>> Stashed changes

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

<<<<<<< Updated upstream
    public void turn(double radians){
        encoderDrive(TURN_SPEED, -(radians * 20.043971751969), (radians * 20.043971751969), 1000);
    }

=======
>>>>>>> Stashed changes
    public void move(double inches){
        encoderDrive(DRIVE_SPEED, inches, inches, 30);
    }

    private double radiansToCounts(double r) {
        return (COUNTS_PER_MOTOR_REV / (2 * Math.PI)) * r;
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
    public void timeDrive(double speed, double inches){
        double time = 5.14*inches/(12*speed);
        if(inches < 0)
            speed*=-1;
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        rearRight.setPower(speed);
        rearLeft.setPower(speed);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time){
            telemetry.addData("time calculated:", time);
            telemetry.update();
        }
        frontRight.setPower(0);
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        telemetry.addLine("running encoder drive");
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
<<<<<<< Updated upstream
}
=======

    //TODO: Test the turn and move functions
    public void turn(double radians){
        encoderDrive(TURN_SPEED, -(radians * Math.sqrt(550)) / 2.54, (radians * Math.sqrt(550)) / 2.54, 1000);
    }

    public boolean objectInFront(){
        return false;
        //TODO: Finish this
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
        if (objectInFront()){
            pos = 1;
            pushPixel();
            turn(-Math.PI/2);
            move(23);
        } else {
            turn(-Math.PI/2);
            move(22);
            turn(Math.PI/2);
            if(objectInFront()){
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

    }
}
>>>>>>> Stashed changes
