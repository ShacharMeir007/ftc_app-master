package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Disabled
public class DcToServo
{
    private String name;
    private  DcMotor  motor = null;
    private double position = 0;
    private double power = 0;
    private double target;
    private double error = 0;
    private  Gamepad gamepad;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public DcToServo(String m_name, DcMotor m_motor, double m_target, Gamepad m_gamepad, HardwareMap m_hardwareMap, Telemetry m_telementry)
    {
        name = m_name;
        DcMotor motor = m_motor;
        position = motor.getCurrentPosition();
        target = m_target;
        gamepad = m_gamepad;
        hardwareMap = m_hardwareMap;
        telemetry = m_telementry;

    }

    double getPosition () { return position;}
    double getTarget () {return  target;}
    String getName() {return name;}
    DcMotor getMotor() {return motor;}
    double getError() {return error;}
    void setTarget (double Newtarget) {target = Newtarget;}
    void setError (double NewError) {error = NewError;}
    void setNewPower (double p) {power = p;}
    void setName (String newName) {name = newName;}
    void setPosition () {position = motor.getCurrentPosition();}

    public void DrivePosition()
    {
        while (true)
        {
            if(gamepad.a)
            {
                target = 250;
            }
            else if(gamepad.b)
            {
                target = 0;
            }
            else if(gamepad.x)
            {
                target =  -250;
            }
            double kp = 1.0 / 200;
            position = motor.getCurrentPosition();
            setError(target  - position);
            setNewPower(error *  kp);
            double max_power = 0.15;
            if(power > max_power) power = max_power;
            if(power < -max_power) power = -max_power;
            motor.setPower(-power);
            telemetry.addData("power",power);
            //telemetry.addData("pos",position);
            telemetry.addData("input",gamepad.left_stick_x);
            telemetry.update();

        }
    }


}
