'''
This file contains the PSEUDOCODE for the entire robot operation.
Use the following notations in the code:
- All distances are in inches
- The distance sensors are called front_left_sensor, front_right_sensor,
  left_sensor, right_sensor, and back_sensor.
    - front_left_sensor() will read the sensor distance, etc.
- The drive train motors are called front_left_motor, front_right_motor,
  back_left_motor, and back_right_motor.
	- front_left_motor(spd) will spin the motor at speed spd, etc.
	- motors(spd1, spd2, spd3, spd4) runs the 4 motors together
- The lift mechanisms are called front_left and back_lift
    - front_lift(state) moves the front lift as far UP as it can go
      or as far DOWN as it can go, etc.
    - lifts(front_state, back_state) runs the 2 lift mechanisms SIMULTANEOUSLY
- Both vacuums are identified as vacuum
	- vacuum(state) can be ON or OFF
- signal EOP with buzz()
'''

# Move robot in that direction for delay seconds (speed is positive)
def move(direction, spd, delay):
	if (direction == 'FORWARD'):
		motors(spd, spd, spd, spd)
	elif (direction == 'BACKWARD'):
		motors(-spd, -spd, -spd, -spd)
	elif (direction == 'LEFT'):
		motors(-spd, spd, spd, -spd)
	elif (direction == 'RIGHT'):
		motors(spd, -spd, -spd, spd)
	time.sleep(delay)
	motors(0, 0, 0, 0)

# Rotate robot for delay seconds (positive speed is CW)
def rotate(spd, delay):
	motors(spd, -spd, spd, -spd)
	time.sleep(delay)
	motors(0, 0, 0, 0)

# Align robot with front-facing wall. ASSUMES robot is facing a stair step
def align():
	while(True):
		alignment = front_left_sensor() - front_right_sensor()
		if(abs(alignment) < 1): # parallel enough
			return
		
		# It's up to you whether to do full PID on alignment or not
		# This code will just use P (proportional controller)
		rotate(alignment * some_factor, 0.1)

# Before traverse_1 can begin, robot needs to be as close to the next
# step as possible (if ASCENDING) or far back enough for the back wheels
# to come down (if DESCENDING)
def traverse_0(POSITION):
	if (POSITION == 'BOTTOM'):
		while(front_left_sensor() > 5 and front_right_sensor() > 5):
			move('FORWARD', 1, 0.1)
			align()
	elif (POSITION == 'TOP'):
		while(back_sensor() < 3):
			move('BACKWARD', 1, 0.1)
		move('BACKWARD', 1, 0.5) # move further back for room for back wheels

# Move half of the robot to the next step, enough for the robot to
# vacuum the first half of the step
def traverse_1(POSITION):
	if (POSITION == 'BOTTOM'): # only front half of robot ascends
		lifts('DOWN', 'DOWN')
		move('FORWARD', 1, 2)
	elif (POSITION == 'TOP'): # only back half of robot descends
		lifts('DOWN', 'DOWN')

# After vacuuming the first half, move rest of the robot to the next ste
def traverse_2(POSITION):
	if (POSITION == 'BOTTOM'): # back half of robot also ascends
		lifts('UP', 'UP')
		move('FORWARD', 1, 2)
	elif (POSITION == 'TOP'): # front half of robot also descends
		move('BACKWARD', 1, 2) # this needs to be precise! Castor must still be on the step while front wheels aren't 
		lifts('UP', 'UP')

# Clean in that direction until hits a wall 
def vacuum(direction):
	vacuum('ON')
	if (direction == 'LEFT'):
		while(left_sensor() > 3):
			move('LEFT', 1, 1)
	elif (direction == 'RIGHT'):
		while(right_sensor() > 3):
			move('LEFT', 1, 1)
	vacuum('OFF')

# Move away from the stairs and signal EOP
def escape_operation(POSITION):
	if (POSITION == 'BOTTOM'):
		move('FORWARD', 1, 3)
	elif (POSITION == 'TOP'):
		move('BACKWARD', 1, 3)
	buzz()

if __name__ == "__main__":
	'''
	Step 1: Setup. Detect whether at top or bottom of stairs.
	Assume that user faces the robot upstairs (orientation error allowed).
	'''
	POSITION = None # whether robot started at top or bottom
	STEPS = 8 # stairwell has 8 steps
	PASSES = 0 # number of steps cleaned
	WALL_THRESHOLD = 3 # when sensor is <WALL_THRESHOLD, robot has "touched" the wall
	
	vacuum('OFF')
	lifts('UP')
	
	if (front_left_sensor() < 30 and front_right_sensor() < 30):
		POSITION = 'BOTTOM'
	else:
		POSITION = 'TOP'
	
	
	while(STEPS != PASSES):
		'''
		Step 2: Get to first half of next step.
		'''
		traverse_0(POSITION)
		traverse_1(POSITION)
		
		'''
		Step 3: Clean first half
		'''
		vacuum('LEFT')
		
		'''
		Step 4: Get to second half of next step.
		'''
		traverse_2(POSITION)
		
		'''
		Step 5: Clean second half
		'''
		vacuum('RIGHT')
		
		PASSES += 1
	
	'''
	Step 6: Exit stairs at the very end
	'''
	escape_operation(POSITION)
