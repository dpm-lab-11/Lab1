package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 20;
  public static final double PROPCONST = 1.0; // Proportionality constant
  public static final int MAXCORRECTION = 50; // Bound on correction to prevent stalling

  private final int bandCenter;
  private final int bandWidth;
  private final int maxFilter = 9;
  private final int minDistance = 8;
  private final int maxDistance = 80;
  
  private int distance;
  private int filterControl;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;


  public PController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int bandCenter, int bandwidth) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
	this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    rightMotor.setSpeed(MOTOR_SPEED);
    leftMotor.forward();
    rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= maxDistance && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= maxDistance) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }

  }


  @Override
  public int readUSDistance() {
    return this.distance;
  }
  
  public int calcProp(int diff) {

	    int correction;

	    // PROPORTIONAL: Correction is proportional to magnitude of error

	    diff = Math.abs(diff);

	    correction = (int) (PROPCONST * (double) diff);
	    if (correction >= MOTOR_SPEED) {
	      correction = MAXCORRECTION;
	    }

	    return correction;
	  }

	}

}
