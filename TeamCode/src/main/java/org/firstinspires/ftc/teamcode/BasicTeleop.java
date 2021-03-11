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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Teleop", group="Linear Opmode")
public class BasicTeleop extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final HardwareMecanumDrive drive = new HardwareMecanumDrive();
    private final HardwareManipulators manipulators = new HardwareManipulators();
    private boolean intakeOn = false;
    private boolean intakeBoost = false;
    private boolean armOpen = true;

    @Override
    public void runOpMode() {
        // Initialize drive and manipulators
        telemetry.addData("Status", "Initializing");
        drive.initialize(hardwareMap);
        manipulators.initialize(hardwareMap);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for the power of each motor and servo
            double frontLeftPower;
            double frontRightPower;
            double backLeftPower;
            double backRightPower;
            double intakePower;
            double intakeServoPower;
            double rampPower;
            double shooterPower;
            double armPower;
            double armPosition;

            // Calculate each mathematical component of drive power
            double hypotenuse = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rotation = gamepad1.right_stick_x;

            // Calculate power for each wheel
            frontLeftPower = Range.clip(hypotenuse * Math.sin(angle) - rotation, -1.0, 1.0);
            frontRightPower = Range.clip(hypotenuse * Math.cos(angle) + rotation, -1.0, 1.0);
            backLeftPower = Range.clip(hypotenuse * Math.cos(angle) - rotation, -1.0, 1.0);
            backRightPower = Range.clip(hypotenuse * Math.sin(angle) + rotation, -1.0, 1.0);

            // Set intake motor state based on the input
            if (gamepad2.a)
            {
                intakeOn = true;
                intakeBoost = false;
            }
            else if (gamepad2.left_trigger > 0)
            {
                intakeOn = true;
                intakeBoost = true;
            }
            else if (gamepad2.b)
            {
                intakeOn = false;
                intakeBoost = false;
            }

            // Set intake roller power to 50% when A is pressed
            // Set power to 100% when right trigger is pressed
            // Set power to 0% when B is pressed
            if (intakeOn && !intakeBoost)
                intakePower = .5;
            else if (intakeOn)
                intakePower = 1;
            else
                intakePower = 0;

            // Set intake to run forward while X is held
            // Set intake to run backward while Y is held
            if (gamepad2.x)
            {
                manipulators.intakeServo.setDirection(DcMotor.Direction.FORWARD);
                intakeServoPower = 1;
            }
            else if (gamepad2.y)
            {
                manipulators.intakeServo.setDirection(DcMotor.Direction.REVERSE);
                intakeServoPower = 1;
            }
            else
                intakeServoPower = 0;

            // Raise ramp at 50% power while DPAD UP is held
            // Lower ramp at 75% power while DPAD DOWN is held
            if (gamepad2.dpad_up)
            {
                manipulators.rampMotor.setDirection(DcMotor.Direction.REVERSE);
                rampPower = .50;
            }
            else if (gamepad2.dpad_down)
            {
                manipulators.rampMotor.setDirection(DcMotor.Direction.FORWARD);
                rampPower = .75;
            }
            else
                rampPower = 0;

            // Set shooter power based on how far in the trigger is pressed
            if (gamepad2.right_trigger > 0)
                shooterPower = gamepad2.right_trigger;
            else
                shooterPower = 0;

            // Set arm state based on the input
            if (gamepad2.dpad_right)
                armOpen = true;
            else if (gamepad2.dpad_left)
                armOpen = false;

            // Open arm when DPAD RIGHT is pressed
            // Close arm when DPAD LEFT is pressed
            if (armOpen)
                armPosition = .85;
            else
                armPosition = .5;

            // Raise arm while left joystick up is held
            // Lower arm while left joystick down is held
            if (gamepad2.left_stick_y < 0)
            {
                manipulators.armMotor.setDirection(DcMotor.Direction.REVERSE);
                armPower = 1;
            }
            else if (gamepad2.left_stick_y > 0)
            {
                manipulators.armMotor.setDirection(DcMotor.Direction.FORWARD);
                armPower = 1;
            }
            else
                armPower = 0;

            // Send power to motors and servos
            drive.setPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            manipulators.setIntakeServo(intakeServoPower);
            manipulators.setIntakeMotor(intakePower);
            manipulators.setRampMotor(rampPower);
            manipulators.setShooterMotor(shooterPower);
            manipulators.setArmMotor(armPower);
            manipulators.setArmServo(armPosition);

            // Show the elapsed game time, servo power, and motor power
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)",
                                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Manipulators", "Intake Servo (%.2f), Intake Motor (%.2f), Ramp Motor (%.2f), Shooter Motor (%.2f), Arm Motor (%.2f), Arm Servo (%.2f)",
                                intakeServoPower, intakePower, rampPower, shooterPower, armPower, armPosition);
            telemetry.update();
        }
    }
}
