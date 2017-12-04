#!/usr/bin/python

import numpy as np
import math
import matplotlib.pyplot as plt
from numpy import genfromtxt
from pandas import read_csv
from matplotlib.backends.backend_pdf import PdfPages
import getopt,sys


def main(argv):

    o_file = ''
    i_file = ''
    
    try:
        opts,args = getopt.getopt(argv,"hi:o:",["ifile=","help","ofile="])
    except getopt.GetoptError:
        print "USAGE:"
        print 'plot_att.py -i <estimatoroutputfile> -o <plotoutput>'
        sys.exit(2)
    for opt,arg in opts:
        if opt in ("-h","--help"):
            print "USAGE:"
            print 'plot_att.py -i <estimatoroutputfile> -o <plotoutput>'
            print "-i , --ifile : .KVH file input to attitude estimator."
            print "-o , --ofile : .csv file output of attitude estimator."
            sys.exit()
        elif opt in ("-i","--ifile"):
            i_file = arg
        elif opt in ("-o","--ofile"):
            o_file = arg


    print "LOADED FILE: " + i_file
    print "GENERATING PLOTS"
    
    params = read_csv(i_file,nrows=1,header=None)
    data = read_csv(i_file,skiprows=1,header=None)

    exp = "KVH"

    

    plt.figure(1)
    plt.suptitle('Estimated Attitude',y=0.99)
    plt.subplot(311)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([2])*180.0/math.pi,label=exp)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([5])*180.0/math.pi,label="PHINS")
    plt.ylabel('Roll (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,data.as_matrix([1])[-1]-data.as_matrix([1])[0], -200, 200])
    plt.grid(True)
    plt.legend(bbox_to_anchor=(0., 1., 1., 1.), loc=3,ncol=2, mode="expand", borderaxespad=0.25, fontsize=12)
    plt.subplot(312)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([3])*180.0/math.pi,data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([6])*180.0/math.pi)
    plt.ylabel('Pitch (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,data.as_matrix([1])[-1]-data.as_matrix([1])[0], -200, 200])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([4])*180.0/math.pi,data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([7])*180.0/math.pi)
    plt.ylabel('Heading (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,data.as_matrix([1])[-1]-data.as_matrix([1])[0], -200, 200])
    plt.grid(True)

    plt.figure(2)
    plt.suptitle('Estimated Attitude Error')
    plt.subplot(311)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([2])*180.0/math.pi-data.as_matrix([5])*180.0/math.pi)
    plt.ylabel('Roll (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,data.as_matrix([1])[-1]-data.as_matrix([1])[0], -190, -170])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([3])*180.0/math.pi-data.as_matrix([6])*180.0/math.pi)
    plt.ylabel('Pitch (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,data.as_matrix([1])[-1]-data.as_matrix([1])[0], -10, 10])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([4])*180.0/math.pi-data.as_matrix([7])*180.0/math.pi)
    plt.ylabel('Heading (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,data.as_matrix([1])[-1]-data.as_matrix([1])[0], 35, 55])
    plt.grid(True)

    plt.figure(3)
    plt.suptitle('Angular Velocity Sensor Bias')
    plt.subplot(311)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([8]))
    plt.ylabel('x (radians/s)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([9]))
    plt.ylabel('y (radians/s)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([10]))
    plt.ylabel('z (radians/s)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)

    plt.figure(4)
    plt.suptitle('Linear Accelerometer Sensor Bias')
    plt.subplot(311)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([14]))
    plt.ylabel('x (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([15]))
    plt.ylabel('y (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([16]))
    plt.ylabel('z (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)

    plt.figure(5)
    plt.suptitle('q_tilde')
    plt.subplot(311)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([17])*180.0/math.pi)
    plt.ylabel('x (radians)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([18])*180.0/math.pi)
    plt.ylabel('y (radians)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([19])*180.0/math.pi)
    plt.ylabel('z (radians)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)

    plt.figure(6)
    plt.suptitle('Gravity Estimate')
    plt.subplot(311)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([20]))
    plt.ylabel('x (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([21]))
    plt.ylabel('y (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(data.as_matrix([1])-data.as_matrix([1])[0],data.as_matrix([22]))
    plt.ylabel('z (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)

    print "SAVING TO: " + o_file
    pp = PdfPages(o_file)
    pp.savefig(plt.figure(1))
    pp.savefig(plt.figure(2))
    pp.savefig(plt.figure(3))
    pp.savefig(plt.figure(4))
    pp.savefig(plt.figure(5))
    pp.savefig(plt.figure(6))
    pp.close()

        
    
if __name__ == "__main__":
    main(sys.argv[1:])
