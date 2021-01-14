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

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Intake", group="Linear Opmode")
public class BasicIntake extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake;
    private CRServo intakeServo;
    private DcMotor rampMotor;
    private boolean motorOn = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        rampMotor = hardwareMap.get(DcMotor.class, "ramp");

        // Set direction of motor
        intake.setDirection(DcMotor.Direction.REVERSE);
        rampMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double intakePower;
            double intakeServoPower;
            double rampPower;

            // Set intake roller power to 50% when A is pressed and set power to 0% when B is pressed
            if (gamepad1.a)
                motorOn = true;
            if (gamepad1.b)
                motorOn = false;
            if (motorOn)
                intakePower = .5;
            else
                intakePower = 0;

            // Set intake to run forwards when right trigger is held or in reverse when left trigger is held
            if (gamepad1.right_trigger > 0)
            {
                intakeServo.setDirection(DcMotor.Direction.FORWARD);
                intakeServoPower = 1;
            }
            else if (gamepad1.left_trigger > 0)
            {
                intakeServo.setDirection(DcMotor.Direction.REVERSE);
                intakeServoPower = 1;
            }
            else
                intakeServoPower = 0;

            // Raise the ramp at 10% power if X is held and lower ramp at 10% power if Y is held
            if (gamepad1.x)
            {
                rampMotor.setDirection(DcMotor.Direction.FORWARD);
                rampPower = .1;
            }
            else if (gamepad1.y)
            {
                rampMotor.setDirection(DcMotor.Direction.REVERSE);
                rampPower = .1;
            }
            else
                rampPower = 0;

            // Send power to motors
            intake.setPower(intakePower);
            intakeServo.setPower(intakeServoPower);
            rampMotor.setPower(rampPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Intake Motor", "%5.2f", intake.getPower());
            telemetry.addData("Intake Servo", "%5.2f", intakeServo.getPower());
            telemetry.addData("Ramp Motor", "%5.2f", rampMotor.getPower());
            telemetry.update();
        }
    }
}
