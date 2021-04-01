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

@Autonomous(name="Basic: Autonomous Far Red", group="Basic")
public class BasicAutonomousFarRed extends LinearOpMode
{
    // Declare OpMode members
    private final HardwareMecanumDrive drive = new HardwareMecanumDrive();
    private final HardwareManipulators manipulators = new HardwareManipulators();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");
        drive.initialize(hardwareMap);
        manipulators.initialize(hardwareMap);
        telemetry.addData("Status", "Initialized");

        waitForStart();

        if (opModeIsActive())
        {
            // Close claw around wobble goal
            manipulators.setArmServo(.9);
            sleep(1000);
            manipulators.setArmMotor(1);
            sleep(500);
            manipulators.setArmMotor(0);

            // Note: negative is forward
            // Drive backward at 50% power for 5 seconds, then stop and open claw
            drive.setPower(0.51, 0.51, 0.5, 0.5);
            sleep(5000);
            drive.setPower(0, 0, 0, 0);
            manipulators.setArmServo(.5);
            sleep(1000);

            // Drive forward for 2.5 seconds, then stop
            drive.setPower(-0.5, -0.52, -0.50, -0.52);
            sleep(2500);
            drive.setPower(0, 0, 0, 0);

            // Shoot rings until more than 28 seconds have passed in the autonomous period
            while (getRuntime() <= 28)
            {
                manipulators.setShooterMotor(1);
                sleep(1000);
                manipulators.setIntakeServos(1, 1, 1);
            }

            // Drive backward for .25 seconds, then stop
            drive.setPower(0.5, 0.5, 0.5, 0.5);
            sleep(250);
            drive.setPower(0, 0, 0, 0);
        }
    }
}
