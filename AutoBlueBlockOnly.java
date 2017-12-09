/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LiftAndServos;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="AutoBlueJustBlocks", group ="Test")
public class AutoBlueBlockOnly extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    Servo colourServoY;
    Servo colourServoX;
    ColorSensor colourSensor;
    DistanceSensor sensorDistance;

    MecanumDrive drive;

    LiftAndServos LAS;

    OpenGLMatrix lastLocation = null;

    String pictograph = "UNKNOWN";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        //colourServoX = hardwareMap.servo.get("colourServoX");
        //colourServoY = hardwareMap.servo.get("colourServoY");
        //colourSensor = hardwareMap.get(ColorSensor.class, "colorDistanceSensor");
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "colorDistanceSensor");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};
        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;
        drive = new MecanumDrive(hardwareMap);
        LAS = new LiftAndServos(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcfullL/////AAAAGSOSZA30iEJ6lhWTCbftAasmcUshL/HUebUI6EDhrrnupgA15NCxOkPJDBMD46rE8MlgnGyDIEy3MAYNsykv0eP8Yf/tjV080ZMEAiNBpr2APCddbWLQfBtbN7N1gvCg7ytNJ59sDca1P8g8nsByYb7SXzuTq11DMQfDih3Fz+BR3qW+HBM/4vpa7F4PGkXmbdCF8qFFeD9tkZwjvzgPOiVM0psczS/BMPKwVbsdr3bmVsDYf/0lCfqE1Rzupdtwx9MvtVWxyPvl9EmNExdyLC+NRWW+zTg2bYGVtA/KnWvEXe6m/dLcsUlpdcavc40vAssFruJ+qv2TidEjtLGnKNwkSmUF22GH2Ngk1RiUY1wd";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        if (opModeIsActive()) {
/*
            Color.RGBToHSV((int) (colourSensor.red() * 255),
                    (int) (colourSensor.green() * 255),
                    (int) (colourSensor.blue() * 255),
                    hsvValues);
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", colourSensor.alpha());
            telemetry.addData("Red  ", colourSensor.red());
            telemetry.addData("Green", colourSensor.green());
            telemetry.addData("Blue ", colourSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

            //Gem Knock-off

            colourServoY.setPosition(1);
            sleep(2200);
            double avgColor = 0.0;
            for(int i = 0; i < 5; i++) {
                Color.RGBToHSV((int) (colourSensor.red() * 255),
                        (int) (colourSensor.green() * 255),
                        (int) (colourSensor.blue() * 255),
                        hsvValues);
                telemetry.addData("Hue", hsvValues[0]);
                telemetry.addData("Avg Hue", avgColor);
                if(hsvValues[0] >= 330) {
                    avgColor += (hsvValues[0] - 330) / 5;
                }
                else {
                    avgColor += hsvValues[0] / 5;
                }
                sleep(100);
                telemetry.update();
            }

            if((250 > avgColor && avgColor > 120))
            {
                colourServoX.setPosition(1);
                sleep(500);
                colourServoX.setPosition(0.5);
                sleep(500);
            }
            else if((0 >= avgColor && avgColor <=  30))
            {
                colourServoX.setPosition(0);
                sleep(500);
                colourServoX.setPosition(0.5);
                sleep(500);
            }
            drive.tankDrive(0,0);
            sleep(1000);

            colourServoY.setPosition(0);
            sleep(1000);


            //For lower left thing
            requestOpModeStop();
*/
            LAS.setServos(0.2);
            sleep(2000);
            LAS.lift(0.5);
            sleep(300);
            LAS.lift(0);
            drive.mecMove(0,0.5,0);
            sleep(1500);

            drive.mecMove(270,0.4,0);
            sleep(500);
            drive.mecMove(0,0,0);
            sleep(300);
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            int timePassed = 0;
            telemetry.addData("Relic ", vuMark.toString());
            telemetry.update();
            while(vuMark == RelicRecoveryVuMark.UNKNOWN)
            {
                if(timePassed >= 5000) {
                    break;
                }
                drive.mecMove(270,0.35,0);
                sleep(200);
                timePassed += 200;
                drive.mecMove(0,0,0);
                sleep(150);
                timePassed += 300;
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("Relic ", vuMark.toString());
                telemetry.update();
                sleep(150);
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("Relic ", vuMark.toString());
                telemetry.update();
            }
            drive.mecMove(0,0,0);
            telemetry.addData("Relic ", vuMark.toString());
            telemetry.update();
            sleep(1000);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                pictograph = vuMark.toString();
                telemetry.addData("VuMark", "%s visible", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);//Angle the robot is at to the pic
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                    /*
                while(trans.get(1) > 1 && rot.thirdAngle < -1) {
                    trans = pose.getTranslation();
                    rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    tX = trans.get(0);
                    tY = trans.get(1);//Angle the robot is at to the pic
                    tZ = trans.get(2);//Distance
                    // Extract the rotational components of the target relative to the robot
                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;//Distance from
                    if (rY > 1) {
                        drive.tankDrive(-.35, .35);
                    }
                    if (rY < -1) {
                        drive.tankDrive(.35, -.35);
                    }
                }*/

                    drive.mecMove(90,0.5,0);
                    sleep(2300);

                    drive.mecMove(0,0,0.85);
                    sleep(470);

                    drive.mecMove(0,0,0);
                    sleep(500);

                    if(pictograph.equals("LEFT"))
                    {
                        drive.mecMove(90, 0.5, 0);
                        sleep(1200);
                    }
                    else if(pictograph.equals("CENTER"))
                    {
                        drive.mecMove(90, 0.5, 0);
                        sleep(700);
                    }
                    else if(pictograph.equals("RIGHT"))
                    {
                        drive.mecMove(270, 0.5, 0);
                        sleep(100);
                    }
                    drive.tankDrive(0,0);
                    sleep(500);
                    drive.mecMove(0, 0.5, 0);
                    sleep(1000);
                    drive.tankDrive(0,0);
                    LAS.setServos(0.5);
                    sleep(500);
                    drive.mecMove(180,0.4,0);
                    sleep(500);
                    drive.mecMove(0, 0.4, 0);
                    sleep(600);
                    drive.mecMove(180, 0.5, 0);
                    sleep(500);
                    drive.mecMove(0,0,0);
                    requestOpModeStop();
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
                drive.tankDrive(0,0);
                requestOpModeStop();
            }

            telemetry.update();

        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}