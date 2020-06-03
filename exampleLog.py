# generate example data lot, to unit test plotting tools
# this file will be in ASCII/CSV format
#
# altitude is parabolic.  Starts at (t,h) = (0,0), and goes up to
#   (timeMax/2,hMax).  Lands back down at (timeMax,0).
#
# pressure depends on sea level pressure, using 101325 pascals in formula.
#   formula is pressurePa = 101325 * (1 - 0.0000225577 * altitudeM) ** 5.25588,
#   from https://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html

### parameters

timeStartMs = 0
timeEndMs   = 10000
timeStepMs  = 100

maxHeightM = 300

filename = "example100.txt"

### run

# open the file
f = open(filename,"wt")

# write header line
f.write("#time_ms,pressure_pa,altitude_m\n")

# write lines
timeMs = timeStartMs
altitudeM = 0.0
pressurePa = 0.0

while ( timeMs <= timeEndMs ):
    altitudeM = 4 * maxHeightM / (timeEndMs*timeEndMs) * timeMs * (timeEndMs - timeMs)
    pressurePa = 101325 * (1 - 0.0000225577 * altitudeM) ** 5.25588

    # write line
    f.write( "{:d},{:.1f},{:.2f}\n".format(timeMs,pressurePa,altitudeM) )

    # advance time
    timeMs += timeStepMs

# close the file
f.close()
