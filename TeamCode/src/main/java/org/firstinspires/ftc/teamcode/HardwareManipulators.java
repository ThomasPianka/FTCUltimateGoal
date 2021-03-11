package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareManipulators
{
    public CRServo intakeServo = null;
    public DcMotor intakeMotor = null;
    public DcMotor rampMotor = null;
    public DcMotor shooterMotor = null;
    public DcMotor armMotor = null;
    public Servo armServo = null;
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
        armMotor = map.get(DcMotor.class, "armMotor");
        armServo = map.get(Servo.class, "armServo");

        // Set servo and motor direction
        intakeServo.setDirection(CRServo.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        rampMotor.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setIntakeServo(double intakeServoPower)
    {
        intakeServo.setPower(intakeServoPower);
    }

    public void setIntakeMotor(double intakeMotorPower)
    {
        intakeMotor.setPower(intakeMotorPower);
    }

    public void setRampMotor(double rampMotorPower)
    {
        rampMotor.setPower(rampMotorPower);
    }

    public void setShooterMotor(double shooterMotorPower)
    {
        shooterMotor.setPower(shooterMotorPower);
    }

    public void setArmMotor(double armMotorPower)
    {
        armMotor.setPower(armMotorPower);
    }

    public void setArmServo(double armServoPosition)
    {
        armServo.setPosition(armServoPosition);
    }
}
