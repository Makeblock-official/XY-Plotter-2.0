
//init our variables
long max_delta;
long x_counter;
long y_counter;
long z_counter;
bool x_can_step;
bool y_can_step;
bool z_can_step;
int milli_delay;

void init_steppers()
{
	//turn them off to start.
	disable_steppers();
	
	//init our points.
	current_units.x = 0.0;
	current_units.y = 0.0;
	current_units.z = 0.0;
	target_units.x = 0.0;
	target_units.y = 0.0;
	target_units.z = 0.0;
	
	pinMode(X_STEP_PIN, OUTPUT);
	pinMode(X_DIR_PIN, OUTPUT);
	pinMode(X_ENABLE_PIN, OUTPUT);
	pinMode(X_MIN_PIN, INPUT_PULLUP);
	pinMode(X_MAX_PIN, INPUT_PULLUP);
	
	pinMode(Y_STEP_PIN, OUTPUT);
	pinMode(Y_DIR_PIN, OUTPUT);
	pinMode(Y_ENABLE_PIN, OUTPUT);
	pinMode(Y_MIN_PIN, INPUT_PULLUP);
	pinMode(Y_MAX_PIN, INPUT_PULLUP);
	
	pinMode(Z_STEP_PIN, OUTPUT);
	pinMode(Z_DIR_PIN, OUTPUT);
	pinMode(Z_ENABLE_PIN, OUTPUT);
	pinMode(Z_MIN_PIN, INPUT_PULLUP);
	pinMode(Z_MAX_PIN, INPUT_PULLUP);
	
	//figure our stuff.
	calculate_deltas();
}

void dda_move(long micro_delay)
{
	//enable our steppers
	digitalWrite(X_ENABLE_PIN, HIGH);
	digitalWrite(Y_ENABLE_PIN, HIGH);
	digitalWrite(Z_ENABLE_PIN, HIGH);
	
	//figure out our deltas
	max_delta = max(delta_steps.x, delta_steps.y);
	max_delta = max(delta_steps.z, max_delta);

	//init stuff.
	long x_counter = -max_delta/2;
	long y_counter = -max_delta/2;
	long z_counter = -max_delta/2;
	
	//our step flags
	bool x_can_step = 0;
	bool y_can_step = 0;
	bool z_can_step = 0;
	
	if (micro_delay >= 16383)
		milli_delay = micro_delay / 1000;
	else
		milli_delay = 0;

	//do our DDA line!
	do
	{
		x_can_step = can_step(X_MIN_PIN, X_MAX_PIN, current_steps.x, target_steps.x, x_direction);
		y_can_step = can_step(Y_MIN_PIN, Y_MAX_PIN, current_steps.y, target_steps.y, y_direction);
		z_can_step = can_step(Z_MIN_PIN, Z_MAX_PIN, current_steps.z, target_steps.z, z_direction);

		if (x_can_step)
		{
			x_counter += delta_steps.x;
			
			if (x_counter > 0)
			{
				do_step(X_STEP_PIN, X_DIR_PIN, x_direction);
				x_counter -= max_delta;
				
				if (x_direction)
					current_steps.x++;
				else
					current_steps.x--;
			}
		}

		if (y_can_step)
		{
			y_counter += delta_steps.y;
			
			if (y_counter > 0)
			{
				do_step(Y_STEP_PIN, Y_DIR_PIN, y_direction);
				y_counter -= max_delta;

				if (y_direction)
					current_steps.y++;
				else
					current_steps.y--;
			}
		}
		
		if (z_can_step)
		{
			z_counter += delta_steps.z;
			
			if (z_counter > 0)
			{
                                if(Z_ENABLE_SERVO==0){
				  do_step(Z_STEP_PIN, Z_DIR_PIN, z_direction);
                                }
				z_counter -= max_delta;
				
				if (z_direction)
					current_steps.z++;
				else
					current_steps.z--;
			}
		}
		
		
				
		//wait for next step.
		if (milli_delay > 0)
			delay(milli_delay);			
		else
			delayMicroseconds(micro_delay);
	}
	while (x_can_step || y_can_step || z_can_step);
	
	//set our points to be the same
	current_units.x = target_units.x;
	current_units.y = target_units.y;
	current_units.z = target_units.z;
	calculate_deltas();
}

bool can_step(byte min_pin, byte max_pin, long current, long target, byte direction)
{
	//stop us if we're on target
	if (target == current)
		return false;
	//stop us if we're at home and still going 
	else if (read_switch(min_pin) && !direction)
		return false;
	//stop us if we're at max and still going
	else if (read_switch(max_pin) && direction)
		return false;

	//default to being able to step
	return true;
}

void do_step(byte pinA, byte pinB, byte dir)
{
        switch (dir << 2 | digitalRead(pinA) << 1 | digitalRead(pinB)) {
            case 0: /* 0 00 -> 10 */
            case 5: /* 1 01 -> 11 */
                digitalWrite(pinA, HIGH);
                break;
            case 1: /* 0 01 -> 00 */
            case 7: /* 1 11 -> 10 */
                digitalWrite(pinB, LOW);
                break;
            case 2: /* 0 10 -> 11 */
            case 4: /* 1 00 -> 01 */   
                digitalWrite(pinB, HIGH);
                break;
            case 3: /* 0 11 -> 01 */
            case 6: /* 1 10 -> 00 */
                digitalWrite(pinA, LOW);
                break;
        }
	delayMicroseconds(5);
}


bool read_switch(byte pin)
{
	//dual read as crude debounce
	
	if ( SENSORS_INVERTING )
		return !digitalRead(pin) && !digitalRead(pin);
	else
		return digitalRead(pin) && digitalRead(pin);
}

long to_steps(float steps_per_unit, float units)
{
	return steps_per_unit * units;
}

void set_target(float x, float y, float z)
{
	target_units.x = x;
	target_units.y = y;
	target_units.z = z;
	
	calculate_deltas();
}

void set_position(float x, float y, float z)
{
	current_units.x = x;
	current_units.y = y;
	current_units.z = z;
	
	calculate_deltas();
}

void calculate_deltas()
{
	//figure our deltas.
	delta_units.x = abs(target_units.x - current_units.x);
	delta_units.y = abs(target_units.y - current_units.y);
	delta_units.z = abs(target_units.z - current_units.z);
				
	//set our steps current, target, and delta
	current_steps.x = to_steps(x_units, current_units.x);
	current_steps.y = to_steps(y_units, current_units.y);
	current_steps.z = to_steps(z_units, current_units.z);

	target_steps.x = to_steps(x_units, target_units.x);
	target_steps.y = to_steps(y_units, target_units.y);
	target_steps.z = to_steps(z_units, target_units.z);

	delta_steps.x = abs(target_steps.x - current_steps.x);
	delta_steps.y = abs(target_steps.y - current_steps.y);
	delta_steps.z = abs(target_steps.z - current_steps.z);
	
	//what is our direction
	x_direction = (target_units.x >= current_units.x);
	y_direction = (target_units.y >= current_units.y);
	z_direction = (target_units.z >= current_units.z);

	//set our direction pins as well
	digitalWrite(X_DIR_PIN, x_direction);
	digitalWrite(Y_DIR_PIN, y_direction);
	digitalWrite(Z_DIR_PIN, z_direction);
}


long calculate_feedrate_delay(float feedrate)
{
	//how long is our line length?
	float distance = sqrt(delta_units.x*delta_units.x + delta_units.y*delta_units.y + delta_units.z*delta_units.z);
	long master_steps = 0;
	
	//find the dominant axis.
	if (delta_steps.x > delta_steps.y)
	{
		if (delta_steps.z > delta_steps.x)
			master_steps = delta_steps.z;
		else
			master_steps = delta_steps.x;
	}
	else
	{
		if (delta_steps.z > delta_steps.y)
			master_steps = delta_steps.z;
		else
			master_steps = delta_steps.y;
	}

	//calculate delay between steps in microseconds.  this is sort of tricky, but not too bad.
	//the formula has been condensed to save space.  here it is in english:
	// distance / feedrate * 60000000.0 = move duration in microseconds
	// move duration / master_steps = time between steps for master axis.

	return ((distance * 60000000.0) / feedrate) / master_steps;	
}

long getMaxSpeed()
{
	if (delta_steps.z > 0)
		return calculate_feedrate_delay(FAST_Z_FEEDRATE);
	else
		return calculate_feedrate_delay(FAST_XY_FEEDRATE);
}

void disable_steppers()
{
	//enable our steppers
	digitalWrite(X_ENABLE_PIN, LOW);
	digitalWrite(Y_ENABLE_PIN, LOW);
	digitalWrite(Z_ENABLE_PIN, LOW);
}
