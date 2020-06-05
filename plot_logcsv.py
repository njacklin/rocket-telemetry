# plot telemtery
# captured from serial monitor, saved as lognn.csv
# note some early logs (e.g. LOG12.CSV, LOG13.CSV) have timestamp rollovers

# want to add some analysis features:
#   annotate flight segments, mark with truth data, get average sensor readings
#   re-estimate altitude with "cal" offsets/new estimatation technique

### imports
import csv
import numpy as np
import matplotlib.pyplot as plt

### parameters

DEBUG = True

#filename = "arduino/feather-express/fe_test3/LOG12.CSV"
filename = "arduino/feather-express/fe_test3/LOG13.CSV"

# log annotations/"flight phase" metadata

bUseFlightPhaseTruth = True

##flightPhaseTruth = { # LOG12.CSV
##    "groundBeforeTakeoff" : { "startSec": 40.0, "endSec": 80.0, "trueAGLft":   0.0 },
##    "flightLevel1"        : { "startSec":132.5, "endSec":155.0, "trueAGLft": 100.0 },
##    "flightLevel2"        : { "startSec":180.0, "endSec":200.0, "trueAGLft": 200.0 },
##    "flightLevel3"        : { "startSec":225.0, "endSec":255.0, "trueAGLft": 300.0 },
##    "flightLevel4"        : { "startSec":296.0, "endSec":306.0, "trueAGLft": 393.0 },
##    "groundAfterLanding"  : { "startSec":380.0, "endSec":425.0, "trueAGLft":   0.0 },
##} # end flightPhase LOG12.CSV
##trueGroundAltitudeM = 52.0
flightPhaseTruth = { # LOG13.CSV
    "groundBeforeTakeoff" : { "startSec": 50.0, "endSec": 80.0, "trueAGLft":   0.0 },
    "flightLevel1"        : { "startSec":110.0, "endSec":130.0, "trueAGLft": 100.0 },
    "flightLevel2"        : { "startSec":155.0, "endSec":180.0, "trueAGLft": 200.0 },
    "flightLevel3"        : { "startSec":210.0, "endSec":250.0, "trueAGLft": 300.0 },
    "flightLevel4"        : { "startSec":290.0, "endSec":305.0, "trueAGLft": 500.0 },
    "groundAfterLanding"  : { "startSec":415.0, "endSec":440.0, "trueAGLft":   0.0 },
} # end flightPhase LOG13.CSV
trueGroundAltitudeM = 52.0

altitudePlotUnits = "ft" # either "m" or "ft"

bUnrollTimestampSupport = True

### init

convertMtoFt = 3.28084

timestampUnrolls = 0

### helper functions

# count lines in log file.  ignore lines that start with "#" or that are empty.
# returns -1 if the file cannot be opened
def logFileCountDataRecords(fn):

    count = 0

    try: 
        with open(fn,"rt") as f:
            for line in f:
                if (len(line) == 0 or line[0] == "#"):
                    pass
                else:
                    count += 1
    except IOError as e:
        print("ERROR: I/O error({0}): {1}".format(e.errno, e.strerror))
        return -1
        
    return count

# convertAltitudeM.  converts pressure in Pa to altitude in m.
# uses formula from Adafruit BMP sensor example code
# this formula was used to create the logged altitude data with offset = 101325 Pa
# note this formula is the same as on
# https://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html (inversed)
def convertAltitudeM(pressurePa,offsetPa=101325.0):
    return ( 44330.0 * (1.0 - pow(pressurePa / offsetPa, 0.1903)) )

### read data
print("opening {:s} ... \n".format(filename))

nRecord = logFileCountDataRecords(filename)
if nRecord < 0:
    raise Exception("Could not determine log file data record count.")

timeMs = np.zeros(nRecord)
timeSec = np.zeros(nRecord)
pressurePa = np.zeros(nRecord)
altitudeM = np.zeros(nRecord)
altitudeFt = np.zeros(nRecord)

try:
    with open(filename, "rt", newline='') as csvfile:
        # use csv library to read
        logcsv = csv.reader(csvfile)
        #logcsv = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC) # broken
        #note: QUOTE_NONNUMERIC doesn't work unless strings are ALWAYS quoted

        nRows = 0
        timemsRaw = 0
        prevTimemsRaw = 0

        iRecord = 0
        
        for row in logcsv:

            # skip line if the first value starts with "#"
            if row[0].startswith('#'):
                continue

            #print(row) # debug

            # read data into data structures
            prevTimemsRaw = timemsRaw;
            timemsRaw = int(row[0]);
            
            timeMs[iRecord] = timemsRaw
            pressurePa[iRecord] = float(row[1])
            altitudeM[iRecord] = float(row[2])

            # fix timestamp rollovers
            if ( bUnrollTimestampSupport ):
                if ( timemsRaw - prevTimemsRaw < 0 ):
                    timestampUnrolls += 1

                timeMs[iRecord] = timeMs[iRecord] + (2**16)*timestampUnrolls

            iRecord += 1

        
        # calculate timestamp in seconds for convenience later
        timeSec = timeMs * 1e-3 

        # calculate altitude in ft
        if ( altitudePlotUnits == "ft" ):
            altitudeFt = altitudeM * convertMtoFt

            
except IOError as e:
    print("ERROR: I/O error({0}): {1}".format(e.errno, e.strerror))
        
if (DEBUG):
    print("Some time values (ms): ")
    print(timeMs[0:3])
    print("...")
    print(timeMs[-4:-1])
    print("Some time values (sec): ")
    print(timeSec[0:3])
    print("...")
    print(timeSec[-4:-1])
    print()

### analysis

# timestamp analysis
timeDiffSec = timeSec[1:] - timeSec[0:-1]
print("timestamp analysis:")
print("       average diff between timestamps = {:0.6f} s".format(timeDiffSec.mean()))
print("    worst case diff between timestamps = {:0.6f} s".format(timeDiffSec.max()))
print("  std. dev. of diff between timestamps = {:0.6f} s".format(timeDiffSec.std()))
print()
print("  timestamps rolled over {:d} times".format(timestampUnrolls))
print()

# report altitude stats
print("altitude (m) start: {:7.1f}".format(altitudeM[0]))
print("altitude (m) end  : {:7.1f}".format(altitudeM[-1]))
print("altitude (m) max  : {:7.1f}\n".format(altitudeM.max()))

if ( altitudePlotUnits == "ft" ):
    print("altitude (ft) start: {:7.1f}".format(altitudeM[0]*convertMtoFt))
    print("altitude (ft) end  : {:7.1f}".format(altitudeM[-1]*convertMtoFt))
    print("altitude (ft) max  : {:7.1f}\n".format(altitudeM.max()*convertMtoFt))

# look at flightPhaseTruth data if provided
if bUseFlightPhaseTruth:

    avgGroundAltitudeM = 0
    avgGroundPressurePa = 0
    nGroundSegment = 0
    
    for fpk,fpv in flightPhaseTruth.items() :
        print("Analyzing flightPhaseTruth '{:s}'...".format(fpk))

        print("  phase starts at {:.1f} s and ends at {:.1f} s".\
              format(fpv['startSec'],fpv['endSec']))

        # loop over data, find relevant indices
        bFoundStart = False
        bFoundEnd = False
        for i in range(len(timeSec)):
            # look for start
            if not bFoundStart:
                if (timeSec[i] >= fpv['startSec']):
                    fpv['startIndex'] = i
                    bFoundStart = True
            # look for end
            if bFoundStart and not bFoundEnd:
                if (timeSec[i] >= fpv['endSec']):
                    fpv['endIndex'] = max(0,i-1)
                    bFoundEnd = True
                    break

        if not bFoundStart or not bFoundEnd:
            raise ValueError("start and/or end times not found in data record")

        # compute average pressure and altitude
        # weighted by time between samples 
        fpv['avgPressurePa'] = (   pressurePa[fpv['startIndex']:fpv['endIndex']+1] \
                                 * timeDiffSec[fpv['startIndex']+1:fpv['endIndex']+2] ).sum() \
                               / timeDiffSec[fpv['startIndex']+1:fpv['endIndex']+2].sum()
        
        fpv['avgAltitudeM'] = ( altitudeM[fpv['startIndex']:fpv['endIndex']+1] \
                                * timeDiffSec[fpv['startIndex']+1:fpv['endIndex']+2] ).sum() \
                              / timeDiffSec[fpv['startIndex']+1:fpv['endIndex']+2].sum()

        print("  average pressure = {:.1f} Pa".format(fpv['avgPressurePa']))
        print("  average altitude = {:.1f} m".format(fpv['avgAltitudeM']))
        if ( altitudePlotUnits == "ft" ):
            print("  average altitude = {:.1f} ft".format(fpv['avgAltitudeM']*convertMtoFt))
        print()

        # accumulate for average ground statistics
        if fpk.startswith("ground"):
            nGroundSegment += 1
            avgGroundAltitudeM += fpv['avgAltitudeM']
            avgGroundPressurePa += fpv['avgPressurePa']

    # compute average ground statistics
    avgGroundAltitudeM /= nGroundSegment
    avgGroundPressurePa /= nGroundSegment
    print("Average 'ground' pressure = {:.1f} Pa".format(avgGroundPressurePa))
    print("Average 'ground' altitude = {:.1f} m".format(avgGroundAltitudeM))
    if ( altitudePlotUnits == "ft" ):
        print("Average 'ground' altitude = {:.1f} ft".format(avgGroundAltitudeM*convertMtoFt))
    print()

    # compute altitude using subtraction of ground altitude reading
    print("Altitude Above Ground Level (AGL) using straight subtraction:")
    # compute AGL
    for fpk,fpv in flightPhaseTruth.items():
        fpv['altitudeAGLM'] = fpv['avgAltitudeM'] - flightPhaseTruth['groundBeforeTakeoff']['avgAltitudeM']
        print("  {:s} average AGL = {:.1f} ft, true AGL = {:.1f} ft"\
              .format(fpk,fpv['altitudeAGLM']*convertMtoFt,fpv['trueAGLft']))
    print()

    # lets try to get better AGL values
    # try 1: use ground pressure as offset in altitude calc (does this give us AGL...?)
    print("Better AGL estimate try 1: use avg ground pressure ({:.1f} Pa) as offset"\
          .format(avgGroundPressurePa))
    
    for fpk,fpv in flightPhaseTruth.items():
        fpv['estimatedAGL1M'] = convertAltitudeM(fpv['avgPressurePa'],avgGroundPressurePa)
        print("  {:s} esimated AGL = {:.1f} ft".\
              format(fpk,fpv['estimatedAGL1M']*convertMtoFt),end="")
        print(", true AGL = {:.1f} ft".format(fpv['trueAGLft']))
    print()
    
    # try 2
    # just use averages before takeoff, since that is all we will know in practice
    # first, use ground pressure reading and true ground altitude to find altimeter setting
    # then use true ground altitude to subtract off to get AGL
    # use https://www.engineeringtoolbox.com/air-altitude-pressure-d_462.html
    # and solve for pressure at sea level
    print("Better AGL estimate try 2: use sensor data to set sea level pressure offset")
    pressureSeaLevelCalcPa = flightPhaseTruth['groundBeforeTakeoff']['avgPressurePa'] \
                             / ( 1 - 2.25577e-5 * trueGroundAltitudeM ) ** 5.25588
    print("  sea level pressure offset calculated = {:.1f} Pa".format(pressureSeaLevelCalcPa))
    for fpk,fpv in flightPhaseTruth.items():
        fpv['estimatedAGL2M'] = convertAltitudeM(fpv['avgPressurePa'],pressureSeaLevelCalcPa) \
                                - trueGroundAltitudeM
        print("  estimated AGL for '{:s}': {:.1f} ft".format(fpk,fpv['estimatedAGL2M']*convertMtoFt),end="")
        print(", true AGL = {:.1f} ft".format(fpv['trueAGLft']))
    print()
              
### do plots
        
if ( altitudePlotUnits == "m" ):
    altitudePlot = altitudeM
elif ( altitudePlotUnits == "ft" ):
    altitudePlot = altitudeFt
else:
    raise ValueError("Invalid altitudePlotUnits")

# altitude and pressure vs. time
# altutide vs. time
hFig1 = plt.figure(1)
hFig1s1 = plt.subplot(211)
plt.plot(timeSec,altitudePlot)
if ( altitudePlotUnits == "m" ):
    plt.ylabel('altitude (m)')
elif ( altitudePlotUnits == "ft" ):
    plt.ylabel('altitude (ft)')
plt.grid(True)
plt.title("data vs. time")
# pressure vs. time
hFig1s2 = plt.subplot(212)
plt.plot(timeSec,pressurePa)
plt.ylabel("pressure (Pa)")
plt.xlabel("time (s)")
plt.grid(True)

# add annotations
if bUseFlightPhaseTruth:
    # add annotations to altitude plot
    plt.subplot(hFig1s1)
    for fpk,fpv in flightPhaseTruth.items() :
        # add start annotation
        plt.scatter(fpv["startSec"],altitudePlot[fpv['startIndex']],c="g",marker="x")
        # add end annotation
        plt.scatter(fpv["endSec"],altitudePlot[fpv['endIndex']],c="r",marker="x")
    
    # add annotations to pressure plot
    plt.subplot(hFig1s2)
    for fpk,fpv in flightPhaseTruth.items() :
        # add start annotation
        plt.scatter(fpv["startSec"],pressurePa[fpv['startIndex']],c="g",marker="x")
        # add end annotation
        plt.scatter(fpv["endSec"],pressurePa[fpv['endIndex']],c="r",marker="x")
    

# altitude vs. pressure
plt.figure(2)
plt.scatter(pressurePa,altitudePlot)
if ( altitudePlotUnits == "m" ):
    plt.ylabel("altitude (m)")
elif ( altitudePlotUnits == "ft" ):
    plt.ylabel("altitude (ft)")
plt.xlabel("pressure (Pa)")
plt.title("altitude vs. pressure")
plt.grid(True)

# call plt.show() once at the end of all plots
plt.show()



