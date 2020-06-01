import csv
import sys

f = open(sys.argv[1], 'rt')
try:
    csvlog = csv.reader(f)
    for row in csvlog:
        print(row)
finally:
    f.close()
