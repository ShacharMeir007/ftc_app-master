package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="check")

public class check_test extends LinearOpMode
{


    @Override
    public void runOpMode ()
    {
        DcMotor FL = hardwareMap.dcMotor.get("FL");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor left_pump = hardwareMap.dcMotor.get("left_pump");
        DcMotor height = hardwareMap.dcMotor.get("height");

        waitForStart();
        while (opModeIsActive())
        {
            while(gamepad1.x)
            {
                FL.setPower(0.2);
            }
            while(gamepad1.b)
            {
                BL.setPower(0.2);
            }
            while(gamepad1.a)
            {
                left_pump.setPower(0.2);
            }
            while(gamepad1.y)
            {
                height.setPower(0.2);
            }

            FL.setPower(0);
            BL.setPower(0);
            left_pump.setPower(0);
            height.setPower(0);
        }

    }
}

