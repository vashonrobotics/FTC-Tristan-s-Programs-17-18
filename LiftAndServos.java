package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Tristan on 9/26/2016.
 */
public class LiftAndServos//I noah guy
{
    Servo servoLeft;
    Servo servoRight;
    DcMotor liftMotor;

    double servoPosistion = 0;

    double off = 1;
    public LiftAndServos(HardwareMap hardwareMap)
    {
        liftMotor = hardwareMap.dcMotor.get("liftMotor"); //sets DcMotors to type of motor
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoLeft = hardwareMap.servo.get("servoL");
        servoRight = hardwareMap.servo.get("servoR");
    }

    public void lift(double power)
    {
        if(off + power > 0) {
            liftMotor.setPower(power);
        }
        off += power;
    }

    public void moveServos()
    {

        servoLeft.setPosition(servoPosistion);
        servoRight.setPosition(1-servoPosistion);
    }

    public void servoPosistion(double change) {
        servoPosistion += change;
        if(servoPosistion > 1)
        {
            servoPosistion = 1;
        }
        if(servoPosistion < 0)
        {
            servoPosistion = 0;
        }
        //servoLeft.setPosition(servoPosistion);
        //servoRight.setPosition(-servoPosistion);
    }

}
