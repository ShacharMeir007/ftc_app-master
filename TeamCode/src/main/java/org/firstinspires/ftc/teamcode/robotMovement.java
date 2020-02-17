/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "robotMovement")
public class robotMovement extends LinearOpMode {
{

    @Override
    public void runOpMode()
    {

    }

*/
/*    class DcToServo
    {
        private String name;
        private  DcMotor  motor = null;
        private double position = 0;
        private double power = 0;
        private double target;
        private double error = 0;
        public DcToServo(String m_name)
        {
            name = m_name;
            DcMotor motor = hardwareMap.dcMotor.get(name);
            position = motor.getCurrentPosition();
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
            while ()
            {
                if(gamepad1.a)
                {
                    target = 250;
                }
                else if(gamepad1.b)
                {
                    target = 0;
                }
                else if(gamepad1.x)
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
                telemetry.addData("input",gamepad1.left_stick_x);
                telemetry.update();

            }
        }*//*



}*/
