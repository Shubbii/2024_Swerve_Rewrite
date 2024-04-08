package org.team2168;

import org.team2168.utils.F310;
import org.team2168.utils.LinearInterpolator;
import org.team2168.Constants;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class OI {

    private static OI instance = null;
    
    public F310 driverJoystick = new F310(Constants.Controllers.DRIVER_JOYSTICK);
	public F310 operatorJoystick = new F310(Constants.Controllers.OPERATOR_JOYSTICK);
    public F310 testJoystick = new F310(Constants.Controllers.TEST_JOYSTICK); // test joystick removed for packet loss improvement

    public static final SendableChooser<String> joystickChooser = new SendableChooser<>();

    private LinearInterpolator driverJoystickYInterpolator;
	private LinearInterpolator driverJoystickXInterpolator;
	private LinearInterpolator driverJoystickZInterpolator;
	private LinearInterpolator driverFlightStickZInterpolator;

	private SlewRateLimiter driverJoystickYRateLimiter = new SlewRateLimiter(1.5);
	private SlewRateLimiter driverJoystickXRateLimiter = new SlewRateLimiter(1.5);
	private SlewRateLimiter driverJoystickZRateLimiter = new SlewRateLimiter(0.5);

	private double[][] driverJoystickYArray = {
		{-1.0, -1.0}, //don't scale turning max
		{-0.6, -0.5},
		{-0.15, 0.00}, //set neutral deadband to 15%
		{+0.15, 0.00}, // reduced driving motor speed
		{+0.6, 0.5},
		{+1.00,+1.00}
	};
	private double[][] driverJoystickXArray = {
		{-1.0, -1.0},  //don't scale turning max
		{-0.6, -0.5},
		{-0.15, 0.00}, //set neutral deadband to 15%
		{+0.15, 0.00}, // reduced driving motor speed
		{+0.6, 0.5},
		{+1.00,+1.00}
	};
	private double[][] driverJoystickZArray = {
		{-1.0, -0.50},  //scale down turning to max 50%
		{-0.09, 0.00},  //set neutral deadband to 21%
		{+0.09, 0.00},
		{+1.00,+0.50}
	};
	private double[][] driverFlightStickZArray = {
		{-1.0, 0.00},  // use 0.0 for different turning process //scale down turning to max 50%
		{-0.40, 0.00},  //set neutral deadband to 5%
		{+0.15, 0.00},  // FIX THIS WHEN NON DRIFTING FLIGHT STICK!!!
		{+1.00, 0.00}
	};

    private OI() {


		driverJoystickYInterpolator = new LinearInterpolator(driverJoystickYArray);
		driverJoystickXInterpolator = new LinearInterpolator(driverJoystickXArray);
		driverJoystickZInterpolator = new LinearInterpolator(driverJoystickZArray);
		driverFlightStickZInterpolator = new LinearInterpolator(driverFlightStickZArray);

		SmartDashboard.putData("Driver Joystick Chooser", joystickChooser);
		joystickChooser.setDefaultOption("Flight Joystick", "flight");
		joystickChooser.addOption("F310 Joystick", "F310");
  
    }

    /*************************************************************************
	 * Drivetrain *
	 *************************************************************************/

	/**
	 * Get the value of the left stick's x-axis after being put through the interpolator
	 * @return a value from -0.1 to 0.1
	 */
	public double getDriverJoystickXValue() {
		return driverJoystickXInterpolator.interpolate(driverJoystick.getLeftStickRaw_X());
	}

	/**
	 * Get the value of the left stick's y-axis after being put through the interpolator
	 * @return a value from -0.1 to 0.1
	 */
	public double getDriverJoystickYValue() {
		return driverJoystickYInterpolator.interpolate(driverJoystick.getLeftStickRaw_Y());
	}

	/**
	 * Get the value of the right stick's x-axis after being put through the interpolator
	 * @return a value from -0.5 to 0.5
	 */
	public double getDriverJoystickZValue() {
		if (joystickChooser.getSelected().equals("flight"))
			return driverFlightStickZInterpolator.interpolate(driverJoystick.getRawAxis(2));
		else
			return driverJoystickZInterpolator.interpolate(driverJoystick.getRightStickRaw_X());
	}

	/**
	 * Gets value of driver joystick x-imput after filtering through a SlewRateLimiter object
	 * @return rate-limited driverJoystick X value
	 */
	public double getLimitedDriverJoystickXValue() {
		return driverJoystickXRateLimiter.calculate(getDriverJoystickXValue());
	}

	/**
	 * Gets value of driverJoystick y-imput after filtering through a SlewRateLimiter object
	 * @return rate-limited driverJoystick Y value
	 */
	public double getLimitedDriverJoystickYValue() {
		return driverJoystickYRateLimiter.calculate(getDriverJoystickYValue());
	}

	/**
	 * Gets value of driverJoystick Z-input after filtering through a SlewRateLimiter object
	 * @return rate-limited driverJoystick Z value
	 */
	public double getLimitedDriverJoystickZValue() {
		return driverJoystickZRateLimiter.calculate(getDriverJoystickZValue());
	}


    public static OI getInstance() {
		if (instance == null)
			instance = new OI();

		return instance;
	}
}
