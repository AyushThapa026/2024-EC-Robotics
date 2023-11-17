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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    public DcMotor leftLinearSlide = null;
    public DcMotor rightLinearSlide = null;

    public Servo clawRotation = null;

    public Servo claw = null;
    public DcMotor upperArmJoint = null;
    public DcMotor lowerArmJoint = null;
    public DcMotor rootArmJoint = null;

    //public Servo launcherServo = null;

    private ElapsedTime runtime = new ElapsedTime();


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

        //arm
        leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        upperArmJoint = hardwareMap.dcMotor.get("arm_upper_joint");
        lowerArmJoint = hardwareMap.dcMotor.get("arm_lower_joint");
        clawRotation = hardwareMap.servo.get("claw_rotation");
        claw = hardwareMap.servo.get("claw_controller");
        //rootArmJoint = hardwareMap.get(DcMotor.class, "root_arm_joint");
        //launcherServo = hardwareMap.get(Servo.class, "launcher_servo");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        //this needs to be corrected with testing, this is just and example
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {}

    public void wheelMovementLoop() {
        // Using trig to set the motor speeds so that the bot can move in all directions
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-    gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        double leftFrontWheelPower = r * Math.cos(robotAngle) * Math.sqrt(2) + rightX;
        double rightFrontWheelPower = r * Math.sin(robotAngle) * Math.sqrt(2) - rightX;
        double leftRearWheelPower = r * Math.sin(robotAngle) * Math.sqrt(2) + rightX;
        double rightRearWheelPower = r * Math.cos(robotAngle) * Math.sqrt(2) - rightX;

        frontLeft.setPower(leftFrontWheelPower);
        frontRight.setPower(rightFrontWheelPower);
        rearLeft.setPower(leftRearWheelPower);
        rearRight.setPower(rightRearWheelPower);

        telemetry.addData("leftFront",  "%.2f", leftFrontWheelPower);
        telemetry.addData("rightFront",  "%.2f", rightFrontWheelPower);
        telemetry.addData("leftRear",  "%.2f", leftRearWheelPower);
        telemetry.addData("rightRear", "%.2f", rightRearWheelPower);
    }

    public void armMovementLoop() {
        double upperArmJointPower = gamepad2.left_stick_y;
        double lowerArmJointPower = gamepad2.right_stick_y;

        double jointSpeedDamp = 0.3;
        double linearSpeedPower = 0.3;

        upperArmJoint.setPower(upperArmJointPower * jointSpeedDamp);
        lowerArmJoint.setPower(lowerArmJointPower * jointSpeedDamp);

        if(gamepad1.dpad_up) {
            leftLinearSlide.setPower(linearSpeedPower);
            rightLinearSlide.setPower(linearSpeedPower);
        } else if(gamepad1.dpad_down){
            leftLinearSlide.setPower(-linearSpeedPower);
            rightLinearSlide.setPower(-linearSpeedPower);
        }



        if(gamepad2.dpad_up)
            clawRotation.setPosition(clawRotation.getPosition()+0.02);
        else if(gamepad2.dpad_down)
            clawRotation.setPosition(clawRotation.getPosition()-0.02);


        if(gamepad2.a)
            claw.setPosition(1);
        else if (gamepad1.b)
            claw.setPosition(1/6.0);
        else if (gamepad1.x)
            claw.setPosition(5/6.0);
        else if (gamepad1.y)
            claw.setPosition(0);
    }

    /*
    public void launcherLoop(){
        if(gamepad.button){
             launcherServo.setPosition(Some number);
        }
     }
     */

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        wheelMovementLoop(); // Control the movement of the mecanum wheels using gamepad1
        armMovementLoop(); // Control the movement of the arm claw using gamepad2
        //launcherLoop();

        // Using the run time to display the amount of time remaining in the game mode
        if(runtime.seconds() < 120)
            telemetry.addData("Time Left in Normal Mode", "%4.1f S", (120 - runtime.seconds()));
        else
            telemetry.addData("Time Left in Endgame", "%4.1f S", (150 - runtime.seconds()));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
