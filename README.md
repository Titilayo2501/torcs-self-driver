# TORCS Autonomous Driver - snakeoil3

A Python autonomous racing client for TORCS (The Open Race Car Simulator).

## My Results
- Finished in **1st place**
- **0 damage** recorded
- Best lap: **02:45:69**

## Files
- `snakeoil3_gym_v1.py` - Best and final version (most consistent performance)
- `snakeoil3_gym_v2.py` - Experimental version (tuned further but caused instability)

## Development Process
I started with v1 which achieved stable lap times with 0 damage and 1st place.
I then experimented in v2 by adjusting the steering sensitivity and speed further,
but the car started floating and behaving inconsistently on the track.
I reverted back to v1 as it was the most reliable and consistent version.

## How it works (v1 - best version)
- Steers using angle and track position sensors
- Target speed: 80 km/h
- Traction Control System to prevent wheel spin
- Automatic 6-speed gear shifting

## How to run
1. Open TORCS and start a race (SCR server on port 3001)
2. Run:
   py snakeoil3_gym_v1.py

## Author
Titilayo Olusegun
