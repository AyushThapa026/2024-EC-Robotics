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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.*;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop", group="Robot")
public class RobotTeleopMecanumDrive extends OpMode{

    /* Declare OpMode members. */

    public DcMotor  frontLeft   = null;
    public DcMotor  frontRight  = null;
    public DcMotor  rearLeft    = null;
    public DcMotor  rearRight   = null;
    public Servo clawRotation = null;

    public DcMotor suspensionMotor = null;
    public Servo claw = null;

    public Servo airplaneServo = null;

    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public double driveSpeed = 0.5;

    public boolean isSuspended = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors

        //TODO: Set up hardwareMap
        //drive base

        frontLeft  = hardwareMap.dcMotor.get("left_front_drive");
        frontRight = hardwareMap.dcMotor.get("right_front_drive");
        rearLeft = hardwareMap.dcMotor.get("left_rear_drive");
        rearRight = hardwareMap.dcMotor.get("right_rear_drive");

        airplaneServo = hardwareMap.servo.get("airplane_servo");
        suspensionMotor = hardwareMap.dcMotor.get("suspension_motor");

        //this needs to be corrected with testing, this is just and example
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        suspensionMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        suspensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        suspensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */

    private int issueLevel = 0;
    @Override
    public void init_loop() {
        telemetry.addData("Issue level: ", issueLevel);
        telemetry.update();
        issueLevel++;
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {}

    public void servoMovementLoop() {
        if (gamepad1.x) {
            airplaneServo.setPosition(1);
            telemetry.addData("x DOWN", "");
        }
        telemetry.update();
    }

    public void sprintInput() {
        if (gamepad1.left_bumper) {
            driveSpeed = 1;
        } else {
            driveSpeed = 0.5;
        }
    }

    public void wheelMovementLoop() {
        // Using trig to set the motor speeds so that the bot can move in all directions
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        double leftFrontWheelPower = r * Math.cos(robotAngle) * Math.sqrt(2) + rightX;
        double rightFrontWheelPower = r * Math.sin(robotAngle) * Math.sqrt(2) - rightX;
        double leftRearWheelPower = r * Math.sin(robotAngle) * Math.sqrt(2) + rightX;
        double rightRearWheelPower = r * Math.cos(robotAngle) * Math.sqrt(2) - rightX;

        frontLeft.setPower(leftFrontWheelPower * driveSpeed);
        frontRight.setPower(rightFrontWheelPower * driveSpeed);
        rearLeft.setPower(leftRearWheelPower * driveSpeed);
        rearRight.setPower(rightRearWheelPower * driveSpeed);

        telemetry.addData("leftFront",  "%.2f", leftFrontWheelPower);
        telemetry.addData("rightFront",  "%.2f", rightFrontWheelPower);
        telemetry.addData("leftRear",  "%.2f", leftRearWheelPower);
        telemetry.addData("rightRear", "%.2f", rightRearWheelPower);
    }

    public void suspensionLoop() {
        if (!isSuspended) {
            if (gamepad1.y) {
                // Turn On RUN_TO_POSITION
                isSuspended = true;
                while (true) {
                    suspensionMotor.setPower(-1);
                }
            /*
            long setTime = System.currentTimeMillis();
            while (true) {
                if (System.currentTimeMillis() - setTime > 500)  {
                    setTime = System.currentTimeMillis();

                }
            }
            */
            } else if (gamepad1.dpad_up) {
                suspensionMotor.setPower(0.8);
            } else if (gamepad1.dpad_down) {
                suspensionMotor.setPower(-0.8);
            } else {
                suspensionMotor.setPower(0);
            }
        }

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        wheelMovementLoop(); // Control the movement of the mecanum wheels using gamepad1
        servoMovementLoop();
        suspensionLoop();
        sprintInput();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
