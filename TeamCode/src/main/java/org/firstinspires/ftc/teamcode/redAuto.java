package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="red auto")

public class redAuto extends LinearOpMode
{
    /*values for dcServo motor*/private static final double  DC_SERVO_MAX_POWER  = 0.15;

    //values for object detection
    private static final String VUFORIA_KEY = "AUdml4//////AAABmeZVu2rww0Djk9HZI25Yl2tK9IRIqatToj5noMuDOFM8oAOwJ9sHqM2u1mEMS/7A4erXo2FAtiOKfHEPmkjKjVY0HFwcSihMHyAlJLPY+3BLatpNd87YkM1ONfjF9RqYKh4eZ5tQQZcvIZWddUWvdmogf2DAIzW518d5oxRtf88tdoBGpRKsn4Zi8D3WBdBYK68vLmkh6jDqSvQbp9yjvnj8u0RWGy6ZU1F9aVqvpa7nXJDVLxYdyrxcvW/dei3VJ8uqhTAKpEv2FHD39kJNiG0ftpghdmd5JiHprU/sgbc18XUuQQ983aW9zYl10ZYM0xMkcyl0BLcxLWxc8vZC5tpSFl138+NGFP8TEYyTITt4";
    private static final String SKYSTONE = "skystone";
    private static final String STONE = "stone";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final int SIDES = 2;
    private static final double THRESHOLD_LEFT = 270;
    private static final double THRESHOLD_RIGHT = 400;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    private static final double  drive_KP = 1.0/200;
    private static final double Drive_max_speed = 0.5;

    //value for DcServo mototrs
    private static final double KP = -1.0 / 200;

    private static final double CM_PER_1000 = 50;

    private static final double POSITION_360 = 6200;

    private static final double DRIVE_MIN_SPEED = 0.2;

    private static final double MAX_POWER = 1;

    class objectDetection extends Thread
    {

        boolean flag;

        objectDetection(boolean Flag)
        {
            flag = Flag;
        }


        private void setFlag(boolean newFlag)
        {
            flag = newFlag;
        }

        private void initVuforia()
        {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }

        /**
         * Initialize the TensorFlow Object Detection engine.
         */
        private void initTfod()
        {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, STONE, SKYSTONE);
        }

        @Override
        public void run()
        {
            // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
            // first.
            initVuforia();
            double power = 0;
            if (ClassFactory.getInstance().canCreateTFObjectDetector())
            {
                initTfod();
            } else
            {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }


            if (flag)
            {
                /** Activate TensorFlow Object Detection. */

                if (tfod != null)
                {
                    tfod.activate();
                }

                while (flag)
                {
                    if (tfod != null)
                    {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null)
                        {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            telemetry.addData("power",power);
                            power += 1;
                            for(Recognition recognition : updatedRecognitions)
                            {
                                double recognitionSideAvg = (recognition.getRight()+recognition.getLeft())/SIDES;
                                if(recognition.getLabel().equals(SKYSTONE) && recognitionSideAvg >= THRESHOLD_LEFT && recognitionSideAvg >= THRESHOLD_RIGHT)
                                {
                                    power = 0;
                                    telemetry.addData("skystone","found");
                                    telemetry.addData("recognitionSideAvg=",recognitionSideAvg);
                                }
                            }
                            telemetry.update();
                        }
                    }
                }
            }
        }
    }





    private class autoDcServo extends Thread
    {
        String name;
        private DcMotor encoded_motor;
        private double target = 0;
        private double offset;
        boolean flag = true;
        private double position = 0;
        double privateKP;
        autoDcServo(String tmpName, double tmpPrivateKp)
        {
            name = tmpName;
            encoded_motor = hardwareMap.dcMotor.get(name);
            offset = encoded_motor.getCurrentPosition();
            privateKP = tmpPrivateKp;
        }

        private void setTarget(double newVal)
        {
            target = newVal;
        }

        void setFlag(boolean newState)
        {
            flag = false;
        }

        @Override
        public void run()
        {
            while (flag)
            {
                try
                {
                    TimeUnit.MILLISECONDS.sleep(10);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
                position = encoded_motor.getCurrentPosition() - offset;
                double error = target - position;
                double power = error * KP * privateKP;
                if (power > DC_SERVO_MAX_POWER) power = DC_SERVO_MAX_POWER;
                if (power < -DC_SERVO_MAX_POWER) power = -DC_SERVO_MAX_POWER;
                encoded_motor.setPower(-power);
            }
        }
    }

    class autoPump
    {
        String left_pumpName;
        String right_pumpName;
        DcMotor leftPump;
        DcMotor rightPump;

        autoPump(String tmpLeft_pumpName,String tmpRight_pumpName)
        {
            left_pumpName = tmpLeft_pumpName;
            right_pumpName = tmpRight_pumpName;
            leftPump = hardwareMap.dcMotor.get(left_pumpName);
            rightPump = hardwareMap.dcMotor.get(right_pumpName);
        }

        void pumping()
        {
            rightPump.setPower(MAX_POWER);
            leftPump.setPower(-MAX_POWER * 0.7);
        }

        void stopPumping()
        {
            rightPump.setPower(0);
            leftPump.setPower(0);
        }

        void reversePumping()
        {
            rightPump.setPower(-MAX_POWER);
            leftPump.setPower(MAX_POWER);
        }
    }

    class AutodriveThread
    {
        String motor1Name;
        String motor2Name;
        String motor3Name;
        String motor4Name;
        double motor1Val;
        double motor2Val;
        double motor3Val;
        double motor4Val;
        DcMotor motor1;
        DcMotor motor2;
        DcMotor motor3;
        DcMotor motor4;

        AutodriveThread(String tmpMotor1, String tmpMotor2, String tmpMotor3, String tmpMotor4)
        {
            motor1Name = tmpMotor1;
            motor2Name = tmpMotor2;
            motor3Name = tmpMotor3;
            motor4Name = tmpMotor4;
            motor1 = hardwareMap.dcMotor.get(motor1Name);
            motor2 = hardwareMap.dcMotor.get(motor2Name);
            motor3 = hardwareMap.dcMotor.get(motor3Name);
            motor4 = hardwareMap.dcMotor.get(motor4Name);
        }

        void stopAll()
        {
            motor1.setPower(0);
            motor2.setPower(0);
            motor3.setPower(0);
            motor4.setPower(0);
        }



        Double normelize(Double value)
        {
            double maxSpeed = 1;
            if (value > maxSpeed) value = maxSpeed;

            else if (value < -maxSpeed) value = -maxSpeed;
            return value;
        }


        void cmForwardDrive(double cm)
        {
            double direction = cm > 0 ? -1 : 1;
            double initialPos = motor1.getCurrentPosition();
            double distance = cm / CM_PER_1000 * 1000;
            double final_speed = 1 * direction;
            double initial_speed = 0.2 * direction;
            double section_length = distance / 3;
            double speed;
            while (Math.abs(section_length - (motor1.getCurrentPosition() - initialPos)) > 25)
            {
                speed = acceleration(initial_speed,final_speed,section_length,motor1.getCurrentPosition() - initialPos);
                motor1.setPower(-speed);
                motor2.setPower(speed);
                motor3.setPower(-speed);
                motor4.setPower(speed);
            }
            while (Math.abs(section_length*2 - (motor1.getCurrentPosition() - initialPos)) > 25);
            while (Math.abs(section_length*3 - (motor1.getCurrentPosition() - initialPos)) > 25)
            {
                speed = acceleration(final_speed, initial_speed, section_length,motor1.getCurrentPosition() - initialPos - section_length*2);
                motor1.setPower(-speed);
                motor2.setPower(speed);
                motor3.setPower(-speed);
                motor4.setPower(speed);
            }
            stopAll();
        }


        void turnByAngle(double angle)
        {
            double distance = (angle * POSITION_360) / 360;
            double target = distance + motor1.getCurrentPosition();
            double error = target - motor1.getCurrentPosition();
            for (int i = 0; i < 3; i++)
            {
                while (Math.abs(error) > 100)
                {
                    double speed = error * drive_KP;
                    if (speed > 0 && speed > Drive_max_speed) speed = Drive_max_speed;
                    if (speed < 0 && speed < -Drive_max_speed) speed = -Drive_max_speed;
                    if ((speed > 0 && speed < -DRIVE_MIN_SPEED)) speed = DRIVE_MIN_SPEED;
                    if ((speed < 0 && speed > -DRIVE_MIN_SPEED)) speed = DRIVE_MIN_SPEED;
                    motor1.setPower(speed * 0.7);
                    motor2.setPower(speed * 0.7);
                    motor3.setPower(speed * 0.7);
                    motor4.setPower(speed * 0.7);
                    error = target - motor1.getCurrentPosition();
                    telemetry.addData("postion", motor1.getCurrentPosition());
                    telemetry.update();
                }
                stopAll();
                sleep(100);
            }
        }

        void moveSidecm(double cm)
        {
            double direction = cm > 0 ? 1 : -1;
            double initialPos = motor1.getCurrentPosition();
            double distance = cm / CM_PER_1000 * 1000;
            double final_speed = 0.6 * direction;
            double initial_speed = 0.4 * direction;
            double section_length = distance / 3;
            double speed;
            while (Math.abs(section_length - (motor1.getCurrentPosition() - initialPos)) > 50)
            {
                speed = acceleration(initial_speed,final_speed,section_length,motor1.getCurrentPosition() - initialPos);
                motor1.setPower(speed);
                motor2.setPower(speed);
                motor3.setPower(-speed);
                motor4.setPower(-speed);
            }
            while (Math.abs(section_length*2 - (motor1.getCurrentPosition() - initialPos)) > 50);
            while (Math.abs(section_length*3 - (motor1.getCurrentPosition() - initialPos)) > 50)
            {
                speed = acceleration(final_speed, initial_speed, section_length,motor1.getCurrentPosition() - initialPos - section_length*2);
                motor1.setPower(speed);
                motor2.setPower(speed);
                motor3.setPower(-speed);
                motor4.setPower(-speed);
            }
            stopAll();
        }


        double acceleration(double initial_speed, double final_speed, double accelerationDistance, double currentposition)
        {
            return initial_speed + ((currentposition*(final_speed - initial_speed)) / accelerationDistance );
        }


        }


    private void goToFoundation (Servo move,AutodriveThread drive,Servo block,autoPump pump)
    {
        pump.pumping();
        drive.cmForwardDrive(-35);
        /* close */ block.setPosition(0);
        drive.moveSidecm(85);
        drive.cmForwardDrive(300);
        drive.turnByAngle(115);
        drive.cmForwardDrive(55);
    }

    private void goBack(Servo move,AutodriveThread drive,Servo block,autoPump pump)
    {
        drive.cmForwardDrive(-45);
        drive.turnByAngle(-120);
        drive.cmForwardDrive(-255);
        drive.moveSidecm(-45);
    }


    @Override
    public void runOpMode () throws InterruptedException
    {
      Servo move = hardwareMap.get(Servo.class,"move");
      AutodriveThread drive = new AutodriveThread("FL","FR","BL","BR");
      Servo block = hardwareMap.servo.get("block");
      autoPump pump = new autoPump("left_pump","right_pump");
      objectDetection oD = new objectDetection(true);
      waitForStart();
      //drive.moveSidecm(-210);

        //goToFoundation(move, drive, block, pump);
        //goBack(move, drive, block, pump);

      //move.setPosition(0.5); /* close move fund */
    }
}



