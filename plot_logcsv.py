# plot telemtery
# captured from serial monitor, saved as lognn.csv
# note some early logs (e.g. LOG12.CSV, LOG13.CSV) have timestamp rollovers

# want to add some analysis features:
#   timestamp analysis (average diff, worst diff, std. dev. diffs)
#   annotate flight segments, mark with truth data, get average sensor readings
#   re-estimate altitude with "cal" offsets/new estimatation technique

### imports
import csv
import matplotlib.pyplot as plt

### parameters

DEBUG = True

#filename = "arduino/feather-express/fe_test3/LOG12.CSV"
filename = "arduino/feather-express/fe_test3/LOG13.CSV"

altitudePlotUnits = "ft" # either "m" or "ft"

bUnrollTimestampSupport = True

### init

timeMs = []
timeSec = []
pressurePa = []
altitudeM = []
altitudeFt = []

altitudeStartM = 0.0
altitudeEndM = 0.0
altitudeMaxM = 0.0

convertMtoFt = 3.28084

timestampUnrolls = 0

### read data
print("opening {:s} ... \n".format(filename))

with open(filename, "rt", newline='') as csvfile:
    # use csv library to read
    logcsv = csv.reader(csvfile)
    #logcsv = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC) # broken
    #note: QUOTE_NONNUMERIC doesn't work unless strings are ALWAYS quoted

    nRows = 0
    timemsRaw = 0
    prevTimemsRaw = 0
    
    for row in logcsv:

        # skip line if the first value starts with "#"
        if row[0].startswith('#'):
            continue

        #print(row) # debug

        # read data into data structures
        # TODO: preallocate data structures to speed this code up
        prevTimemsRaw = timemsRaw;
        timemsRaw = int(row[0]);
        timeMs.append(timemsRaw)
        pressurePa.append(float(row[1]))
        altitudeM.append(float(row[2]))

        if ( altitudePlotUnits == "ft" ):
            altitudeFt.append( altitudeM[-1] * convertMtoFt )

        # fix timestamp rollovers
        if ( bUnrollTimestampSupport ):
            if ( timemsRaw - prevTimemsRaw < 0 ):
                timestampUnrolls += 1

            timeMs[-1] = timeMs[-1] + (2**16)*timestampUnrolls

        # calculate timestamp in seconds for convenience later
        timeSec.append(float(timeMs[-1])*1e-3) 

        # gather altitude stats
        if float(row[2]) > altitudeMaxM :
            altitudeMaxM = float(row[2])

        nRows += 1

    csvfile.close()

print("read in {:d} rows\n".format(nRows))

# DEBUG
if (DEBUG):
    print("Some time values (ms): ")
    print(timeMs[0:2])
    print("...")
    print(timeMs[-3:-1])
    print("Some time values (sec): ")
    print(timeSec[0:2])
    print("...")
    print(timeSec[-3:-1])
    print()

# report altitude stats
print("altitude (m) start: {:7.1f}".format(altitudeM[0]))
print("altitude (m) end  : {:7.1f}".format(altitudeM[-1]))
print("altitude (m) max  : {:7.1f}\n".format(altitudeMaxM))

if ( altitudePlotUnits == "ft" ):
    print("altitude (ft) start: {:7.1f}".format(altitudeM[0]*convertMtoFt))
    print("altitude (ft) end  : {:7.1f}".format(altitudeM[-1]*convertMtoFt))
    print("altitude (ft) max  : {:7.1f}\n".format(altitudeMaxM*convertMtoFt))

### do plots

# altitude and pressure vs. time
# altutide vs. time
plt.figure(1)
plt.subplot(211)
if ( altitudePlotUnits == "m" ):
    plt.plot(timeSec,altitudeM)
    plt.ylabel('altitude (m)')
elif ( altitudePlotUnits == "ft" ):
    plt.plot(timeSec,altitudeFt)
    plt.ylabel('altitude (ft)')
else:
    raise ValueError("Invalid altitudePlotUnits")
plt.grid(True)
plt.title("data vs. time")
# pressure vs. time
plt.subplot(212)
plt.plot(timeSec,pressurePa)
plt.ylabel("pressure (Pa)")
plt.xlabel("time (s)")
plt.grid(True)


# altitude vs. pressure
plt.figure(2)
if ( altitudePlotUnits == "m" ):
    plt.scatter(pressurePa,altitudeM)
    plt.ylabel("altitude (m)")
elif ( altitudePlotUnits == "ft" ):
    plt.scatter(pressurePa,altitudeFt)
    plt.ylabel("altitude (ft)")
else:
    raise ValueError("Invalid altitudePlotUnits")
plt.xlabel("pressure (Pa)")
plt.title("altitude vs. pressure")
plt.grid(True)

# call plt.show() once at the end of all plots
plt.show()



