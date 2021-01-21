package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareManipulators
{
    public CRServo intakeServo = null;
    public DcMotor intakeMotor = null;
    public DcMotor rampMotor = null;
    public DcMotor shooterMotor = null;
    private HardwareMap map = null;

    public void initialize(HardwareMap hwMap)
    {
        // Save reference to map
        map = hwMap;

        // Initialize servo and motors
        intakeServo = map.get(CRServo.class, "intakeServo");
        intakeMotor = map.get(DcMotor.class, "intakeMotor");
        rampMotor = map.get(DcMotor.class, "rampMotor");
        shooterMotor = map.get(DcMotor.class, "shooterMotor");

        // Set servo and motor direction
        intakeServo.setDirection(CRServo.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        rampMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void setPower(double intakeServoPower, double intakeMotorPower, double rampMotorPower, double shooterMotorPower)
    {
        // Set servo and motor power
        intakeServo.setPower(intakeServoPower);
        intakeMotor.setPower(intakeMotorPower);
        rampMotor.setPower(rampMotorPower);
        shooterMotor.setPower(shooterMotorPower);
    }
}
