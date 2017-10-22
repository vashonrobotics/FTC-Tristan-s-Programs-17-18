package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Tristan on 9/26/2016.
 */
public class MecanumDriveAuto
{
    DcMotor motorRF;            //Gamepad 1: right stick and left stick
    DcMotor motorLF;            //Gamepad 1: right stick and left stick
    DcMotor motorRB;            //Gamepad 1: right stick and left stick
    DcMotor motorLB;            //Gamepad 1: right stick and left stick
    double off = 1;

    public MecanumDriveAuto(HardwareMap hardwareMap)
    {
        motorRF = hardwareMap.dcMotor.get("motorRF"); //sets DcMotors to type of motor
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
    }

    public void mecaMovement(double degrees, double power)
    {
        double radians = Math.toRadians(degrees+45);
        motorLF.setPower(power * Math.cos(radians));
        motorLB.setPower(power * Math.sin(radians));
        motorRF.setPower(power * Math.sin(radians));
        motorRB.setPower(power * Math.cos(radians));
    }

    public void movement(double powerL, double powerR)
    {
        motorLF.setPower(powerL);
        motorLB.setPower(powerL);
        motorRF.setPower(powerR);
        motorRB.setPower(powerR);
    }
}
