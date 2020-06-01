# plot telemtery, simple

### imports
import csv
import matplotlib.pyplot as matplot

### parameters

filename = "example100.txt"

### init

timeMs = []
pressurePa = []
altitudeM = []

altitudeStartM = 0.0
altitudeEndM = 0.0
altitudeMaxM = 0.0

### read data
print("opening {:s} ... \n".format(filename))

with open(filename, "rt", newline='') as csvfile:
    # use csv library to read
    logcsv = csv.reader(csvfile)
    #logcsv = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC) # broken
    #note: QUOTE_NONNUMERIC doesn't work unless strings are ALWAYS quoted

    nRows = 0
    
    for row in logcsv:

        # skip line if the first value starts with "#"
        if row[0].startswith('#'):
            continue

        #print(row)
        
        timeMs.append(float(row[0]))
        pressurePa.append(float(row[1]))
        altitudeM.append(float(row[2]))

        # gather altitude stats
        if float(row[2]) > altitudeMaxM :
            altitudeMaxM = float(row[2])

        nRows += 1

    csvfile.close()

print("read in {:d} rows\n".format(nRows))

# report altitude stats
print("altitude (m) start: {:7.1f}".format(altitudeM[0]))
print("altitude (m) end  : {:7.1f}".format(altitudeM[-1]))
print("altitude (m) max  : {:7.1f}\n".format(altitudeMaxM))


### do plots

# altitude and pressure vs. time
# altutide vs. time
matplot.figure(1)
matplot.subplot(211)
matplot.plot(timeMs,altitudeM)
matplot.ylabel('altitude (m)')
#matplot.xlabel("time (ms)")
matplot.grid(True)
#matplot.title("altiude (m) vs. time (ms)")
matplot.title("data vs. time")
# pressure vs. time
matplot.subplot(212)
matplot.plot(timeMs,pressurePa)
matplot.ylabel("pressure (Pa)")
matplot.xlabel("time (ms)")
matplot.grid(True)
#matplot.title("pressure (Pa) vs. time (ms)")


# altitude vs. pressure
matplot.figure(2)
matplot.scatter(pressurePa,altitudeM)
matplot.ylabel("altitude (m)")
matplot.xlabel("pressure (Pa)")
matplot.title("altitude (m) vs. pressure (Pa)")
matplot.grid(True)

# call matplot.show() once at the end of all plots
matplot.show()



