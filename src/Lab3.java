import java.util.LinkedList;
import java.util.Queue;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class Lab3 {

	private static int heading;
	private static int backupHeading;
	private static double[] coord; // coord[0] is x, coord[1] is y
	private static double[] backupCoord;
	private static EV3LargeRegulatedMotor rightWheel;
	private static EV3LargeRegulatedMotor leftWheel;
	private static boolean isWrong;

	public static void main(String[] args) {
// initialize robot
		rightWheel = new EV3LargeRegulatedMotor(MotorPort.A);
		leftWheel = new EV3LargeRegulatedMotor(MotorPort.B);
		rightWheel.setSpeed(200);
		leftWheel.setSpeed(200);
		heading = 0;
		backupHeading = 0;
		coord = new double[2];
		backupCoord = new double[2];
		isWrong = false;
		EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(MotorPort.D);

// start ultrasonic sensor
		EV3UltrasonicSensor ultrasensor = new EV3UltrasonicSensor(SensorPort.S2);
		SensorMode sonic = (SensorMode) ultrasensor.getDistanceMode();
		float[] sample_sonic = new float[sonic.sampleSize()];
		sensorMotor.rotate(-90);

// set up bump sensor
		EV3TouchSensor touchsensor = new EV3TouchSensor(SensorPort.S1);
		SensorMode touch = touchsensor.getTouchMode();
		float[] sample_touch = new float[touch.sampleSize()];
		EV3TouchSensor touchsensor2 = new EV3TouchSensor(SensorPort.S3);
		SensorMode touch2 = touchsensor2.getTouchMode();
		float[] sample_touch2 = new float[touch2.sampleSize()];

// let's go!
		System.out.println("Press any key to start");
		Button.waitForAnyPress();
		long startTime = System.currentTimeMillis();

// move forward until bump
		leftWheel.setSpeed(400);
		rightWheel.setSpeed(400);
		rightWheel.forward();
		leftWheel.forward();
		while (sample_touch[0] == 0.0 && sample_touch2[0] == 0.0) {
			touch.fetchSample(sample_touch, 0);
			touch2.fetchSample(sample_touch2, 0);
		}
		long sprintTime = System.currentTimeMillis() - startTime;
		leftWheel.stop(true);
		rightWheel.stop(true);
		Sound.beepSequence();
		moveBackward(0.2);

		System.out.println("heading: " + heading);
		System.out.println((int) coord[0] + " " + (int) coord[1]);

// rotate right and start following the obstacle
		leftWheel.setSpeed(200);
		rightWheel.setSpeed(200);
		leftWheel.rotate(360);
		rightWheel.startSynchronization();
		rightWheel.forward();
		leftWheel.forward();
		rightWheel.endSynchronization();

// take moving average of the sensor
		Queue<Float> lastThreeSamples = new LinkedList<>();
		float lastThreeSum = 0;
		for (int i = 0; i < 3; i++) {
			sonic.fetchSample(sample_sonic, 0);
			lastThreeSamples.add(sample_sonic[0] > 0.6f ? 0.5f : sample_sonic[0]);
			lastThreeSum += sample_sonic[0] > 0.6f ? 0.5f : sample_sonic[0];
		}
		float movingAverage = (float) lastThreeSum / 3;
// System.out.println(movingAverage);

// update initial coordinate
		Delay.msDelay(1500);
		updateCoord(1.5);

		long timer = System.currentTimeMillis();
		long debugTimer = System.currentTimeMillis();

		while (!((coord[0] > -10) && (coord[0] < 10) && (coord[1] > -10) && (coord[1] < 10))) {

			if (System.currentTimeMillis() - startTime > (120000 - 5000)) {
				System.out.println("miehahahaha");
				break;
			}

			touch.fetchSample(sample_touch, 0);
			touch2.fetchSample(sample_touch2, 0);

// keep fetching data from the bump sensor
			if (sample_touch[0] != 0.0 || sample_touch2[0] != 0.0) {
// stop motors with brakes on if bumped into something
				leftWheel.stop(true);
				rightWheel.stop(true);

				resetCoord();

				leftWheel.startSynchronization();

				leftWheel.backward();
				rightWheel.backward();

				leftWheel.endSynchronization();

				Delay.msDelay(800);

				leftWheel.rotate(120);
				leftWheel.forward();
				rightWheel.forward();
				Delay.msDelay(1000);
				updateCoord(1);
			}

			if (System.currentTimeMillis() - debugTimer >= 1000) {
				System.out.println("heading: " + heading);
				System.out.println((int) coord[0] + " " + (int) coord[1]);
				debugTimer = System.currentTimeMillis();
			}

			if (System.currentTimeMillis() - timer >= 500) {
				if (movingAverage < 0.05f) { // turn right a lot
					isWrong = false;
					leftWheel.stop(true);
					rightWheel.stop(true);
					turn(20);
				} else if (movingAverage < 0.1f) { // turn right a little
					isWrong = false;
					leftWheel.stop(true);
					rightWheel.stop(true);
					turn(10);
				} else if (movingAverage < 0.2f) { // go straight
					isWrong = false;
				} else if (movingAverage < 0.25f) { // turn left a little
					isWrong = false;
					leftWheel.stop(true);
					rightWheel.stop(true);
					turn(-10);
				} else if (movingAverage < 0.8f) { // turn left a lot
					isWrong = false;
					leftWheel.stop(true);
					rightWheel.stop(true);
					turn(-15);
				} else { // something goes way too wrong
					if (!isWrong) { // first time that it goes wrong
						backupCoord[0] = coord[0];
						backupCoord[1] = coord[1];
						backupHeading = heading;
						isWrong = true;
					}
					leftWheel.stop(true);
					rightWheel.stop(true);
					turn(-20);
					leftWheel.forward();
					rightWheel.forward();
					Delay.msDelay(1500);
					updateCoord(1.5);
				}
				timer = System.currentTimeMillis();
			}

			leftWheel.forward();
			rightWheel.forward();
			updateCoord(0.5);

// take new measurement
			lastThreeSum -= lastThreeSamples.poll();
			sonic.fetchSample(sample_sonic, 0);
			if (sample_sonic[0] > 0.6f) {
				lastThreeSamples.add(0.5f);
			} else {
				lastThreeSamples.add(sample_sonic[0]);
			}
			lastThreeSum += sample_sonic[0] > 0.6f ? 0.5f : sample_sonic[0];
			movingAverage = lastThreeSum / 3;
		}

		Sound.beepSequence();
		rightWheel.stop(true);
		leftWheel.stop(true);

		System.out.println("heading: " + heading);
		System.out.println((int) coord[0] + " " + (int) coord[1]);

		turn(90);
		leftWheel.setSpeed(400);
		rightWheel.setSpeed(400);
		rightWheel.forward();
		leftWheel.forward();
		Delay.msDelay(sprintTime);
		rightWheel.stop(true);
		leftWheel.stop(true);
		Delay.msDelay(8000);
	}

	private static void turn(int degree) {
		leftWheel.rotate(degree * 4);
		heading -= degree;
	}

	private static boolean backToRange() {
		if (-10 < coord[0] && coord[0] < 10 && -10 < coord[1] && coord[1] < 10)
			return true;
		return false;
	}

// @param: time is in seconds
	private static void updateCoord(double time) {
		double leftSpeed = (leftWheel.getSpeed() * Math.PI / 180) * 2.7;
		double rightSpeed = (rightWheel.getSpeed() * Math.PI / 180) * 2.7;
		double groundSpeed = (leftSpeed + rightSpeed) / 2;
		coord[0] = coord[0] + groundSpeed * time * Math.cos(heading * Math.PI / 180);
		coord[1] = coord[1] + groundSpeed * time * Math.sin(heading * Math.PI / 180);
	}

// @param: distance is in meters
	private static void moveForward(double distance) {
		rightWheel.startSynchronization();
		rightWheel.forward();
		leftWheel.forward();
		rightWheel.endSynchronization();

		long time = (long) ((distance / 0.1885) * 1000);
		Delay.msDelay(time);

		rightWheel.stop(true);
		leftWheel.stop(true);
	}

// @param: distance is in meters
	private static void moveBackward(double distance) {
		rightWheel.startSynchronization();
		rightWheel.backward();
		leftWheel.backward();
		rightWheel.endSynchronization();

		long time = (long) ((distance / 0.1885) * 1000);
		Delay.msDelay(time);

		rightWheel.stop(true);
		leftWheel.stop(true);
	}

	private static void resetCoord() {
		coord[0] = backupCoord[0];
		coord[1] = backupCoord[1];
		heading = backupHeading;

		backupCoord[0] = 0;
		backupCoord[1] = 0;
		backupHeading = 0;
		isWrong = false;
	}
}