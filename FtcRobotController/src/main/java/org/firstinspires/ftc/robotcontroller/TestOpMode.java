package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class TestOpMode extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode()
    {
        boolean var = gamepad1.a;
        telemetry.addData("var",var);
    }
}

