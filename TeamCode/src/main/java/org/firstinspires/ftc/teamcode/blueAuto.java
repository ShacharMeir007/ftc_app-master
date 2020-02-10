package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="blue auto")

public class blueAuto extends LinearOpMode
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




    private class AutoDcServo extends Thread
    {
        String name;
        private DcMotor encoded_motor;
        private double target = 0;
        private double offset;
        boolean flag = true;
        double position;

        String mode = "Position";

        private double power = 0.5;

        double getPosition()
        {
            return position;
        }

        void setPosition(double newPosition)
        {
            position = newPosition;
        }

        AutoDcServo(String tmpName)
        {
            name = tmpName;
            encoded_motor = hardwareMap.dcMotor.get(name);
            offset = encoded_motor.getCurrentPosition();
        }

        private void setTarget(double newVal)
        {
            target = newVal;
        }

        void setFlag(boolean newState)
        {
            flag = false;
        }

        public void setPower(double newPower)
        {
            power = newPower;
        }

        //swtich between move by position mode and move by power
        void changeMode(String newMode)
        {
            switch (newMode)
            {
                case "Position":
                    mode = "Position";
                    break;
                case "Power":
                    mode = "Power";
                    break;

                //in case of invalid input
                default:
                    mode = "Position";
                    break;
            }
        }

        //function that moves the motor by a target position
        private void movePosition()
        {
            position = encoded_motor.getCurrentPosition() - offset;
            try
            {
                TimeUnit.MILLISECONDS.sleep(10);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            double error = target - position;
            double power = error * KP;
            if (power > DC_SERVO_MAX_POWER)
                power = DC_SERVO_MAX_POWER;
            if (power < -DC_SERVO_MAX_POWER) power = -DC_SERVO_MAX_POWER;
            encoded_motor.setPower(-power);
        }
        //function that moves the motor prefixed power
        private void movePower()
        {
            //TODO: add more things to this function else it is useless. Might remove it later
            encoded_motor.setPower(power);
        }

        @Override
        public void run()
        {
            while (flag)
            {
                switch (mode)
                {
                    case "Position":
                        movePosition();
                        break;
                    case "Power":
                        movePower();
                        break;
                }
            }
        }
    }

    class AutodriveThread extends Thread
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
        boolean flag = true;

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

        void setFlag(boolean newState)
        {
            flag = newState;
        }

        Double normelize(Double value)
        {
            double maxSpeed = 1;
            if (value > maxSpeed) value = maxSpeed;

            else if (value < -maxSpeed) value = -maxSpeed;
                return value;
        }


        void moveForward(double distance)
        {
           double target = distance + motor1.getCurrentPosition();
           double error = target - motor1.getCurrentPosition();
           while(Math.abs(error) > 100)
           {
               double speed = error * -drive_KP;
               if(speed > 0 && speed > Drive_max_speed) speed = Drive_max_speed;
               if(speed < 0 && speed < -Drive_max_speed) speed = -Drive_max_speed;
               motor1.setPower(-speed);
               motor2.setPower(speed);
               motor3.setPower(-speed);
               motor4.setPower(speed);
               error = target - motor1.getCurrentPosition();
           }
        }

        void moveSide(double distance)
        {
            double target = distance + motor1.getCurrentPosition();
            double error = target - motor1.getCurrentPosition();
            while(Math.abs(error) > 100)
            {
                double speed = error * drive_KP;
                if(speed > 0 && speed > Drive_max_speed) speed = Drive_max_speed;
                if(speed < 0 && speed < -Drive_max_speed) speed = -Drive_max_speed;
                motor1.setPower(speed);
                motor2.setPower(speed);
                motor3.setPower(-speed);
                motor4.setPower(-speed);
                error = target - motor1.getCurrentPosition();
            }
        }

        void turn(double distance)
        {
            double target = distance + motor1.getCurrentPosition();
            double error = target - motor1.getCurrentPosition();
            while(Math.abs(error) > 100)
            {
                double speed = error * drive_KP;
                if(speed > 0 && speed > Drive_max_speed) speed = Drive_max_speed;
                if(speed < 0 && speed < -Drive_max_speed) speed = -Drive_max_speed;
                motor1.setPower(speed);
                motor2.setPower(speed);
                motor3.setPower(speed);
                motor4.setPower(speed);
                error = target - motor1.getCurrentPosition();
            }
        }

    }


    @Override
    public void runOpMode () throws InterruptedException
    {
        AutodriveThread drive = new AutodriveThread("FL","FR","BL","BR");
        waitForStart();
        hardwareMap.servo.get("block").setPosition(0);
        hardwareMap.dcMotor.get("left_pump").setPower(-0.7);
        hardwareMap.dcMotor.get("right_pump").setPower(1);
        drive.moveSide(4000);
        drive.moveForward(-1000);
        hardwareMap.servo.get("block").setPosition(0.7);
        drive.moveForward(500);
        drive.moveSide(-2000);
        drive.moveForward(3000);
        drive.turn(3000);
        hardwareMap.dcMotor.get("left_pump").setPower(1);
        hardwareMap.dcMotor.get("right_pump").setPower(-1);
        drive.moveForward(1000);
        hardwareMap.dcMotor.get("left_pump").setPower(0);
        hardwareMap.dcMotor.get("right_pump").setPower(0);
    }
}



