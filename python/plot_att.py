#!/usr/bin/python

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
from numpy import genfromtxt
from pandas import read_csv
from matplotlib.backends.backend_pdf import PdfPages
import getopt,sys


def main(argv):

    o_file = ''
    i_file = ''
    exp = 'KVH'

    
    try:
        opts,args = getopt.getopt(argv,"he:i:o:",["ifile=","help","ofile=","exp="])
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
        elif opt in ("-e","--exp"):
            exp = arg

    print "LOADING FILE: " + i_file
    
    params = read_csv(i_file,nrows=1,header=None)
    data = read_csv(i_file,skiprows=1,header=None)
    print "LOADED FILE: " + i_file

    #print "SUBSAMPLING"
    data = data.as_matrix()



    print "GENERATING PLOTS"

    print "SAVING TO: " + o_file
    pp = PdfPages(o_file)

    if params.as_matrix()[0][0]=='PARAMS':
        plt.figure(0)
        plt.axis('off')
        plt.text(0.5,0.7,"CONFIG PARAMS USED:\n",ha='center',va='center')
        plt.text(0.05,0.2,"  Date Modified: " + str(params.as_matrix()[0][1]) +
                 "\n                    Hz: " + str(params.as_matrix()[0][2]) +
                 "\n                  LAT: " + str(params.as_matrix()[0][3]) +
                 "\n Processed File: " + str(params.as_matrix()[0][4]) +
                 "\n           KVH File: " + str(params.as_matrix()[0][5]) +
                 "\nAlignment(rpy): " + str(params.as_matrix()[0][6:9]) +
                 "\n            Ro(rpy): " + str(params.as_matrix()[0][9:12]) +
                 "\n                      k: " + str(params.as_matrix()[0][12:18]) + "\n")
        pp.savefig(plt.figure(0))
        plt.close("all")

    t = data[:,1]-data[0,1]

    
    plt.figure(1)
    plt.suptitle('Estimated Attitude',y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,2]*180.0/math.pi,label=exp)
    plt.plot(t,data[:,5]*180.0/math.pi,label="PHINS")
    plt.ylabel('Roll (degrees)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -25, 25])
    plt.grid(True)
    plt.legend(bbox_to_anchor=(0., 1., 1., 1.), loc=3,ncol=2, mode="expand", borderaxespad=0.25, fontsize=12)
    plt.subplot(312)
    plt.plot(t,data[:,3]*180.0/math.pi,t,data[:,6]*180.0/math.pi)
    plt.ylabel('Pitch (degrees)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -25, 25])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,4]*180.0/math.pi,t,data[:,7]*180.0/math.pi)
    plt.ylabel('Heading (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,t[-1], -190, 190])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.savefig('att.eps', format='eps', dpi=1000)
    plt.close("all")

    delay = 15;
    #delay = 1;
    plt.figure(2)
    plt.suptitle('Estimated Attitude Error')
    plt.subplot(311)
    plt.plot(t[0:-delay],data[0:-delay,2]*180.0/math.pi-data[(delay-1):-1,5]*180.0/math.pi)
    plt.ylabel('Roll (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,t[-1], -2, 2])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t[0:-delay],data[0:-delay,3]*180.0/math.pi-data[(delay-1):-1,6]*180.0/math.pi)
    plt.ylabel('Pitch (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,t[-1], -2, 2])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t[0:-delay],data[0:-delay,4]*180.0/math.pi-data[(delay-1):-1,7]*180.0/math.pi)
    plt.ylabel('Heading (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,t[-1], -20, 20])
    plt.grid(True)
    plt.savefig('error.eps', format='eps', dpi=1000)
    pp.savefig(plt.figure(2))
    plt.close("all")

    plt.figure(3)
    plt.suptitle('Angular Velocity Sensor Bias')
    plt.subplot(311)
    plt.plot(t,data[:,8])
    plt.ylabel('x (radians/s)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -1./100000., 1./100000.])
    plt.ticklabel_format(axis='y', style='sci', scilimits=(-2, 2))
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,9])
    plt.ylabel('y (radians/s)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -1./100000., 1./100000.])
    plt.ticklabel_format(axis='y', style='sci', scilimits=(-2, 2))
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,10])
    plt.ylabel('z (radians/s)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -1./100000., 1./100000.])
    plt.ticklabel_format(axis='y', style='sci', scilimits=(-2, 2))
    plt.grid(True)
    plt.savefig('ang_bias.eps', format='eps', dpi=1000)
    pp.savefig(plt.figure(3))
    plt.close("all")

    plt.figure(4)
    plt.suptitle('Linear Accelerometer Sensor Bias')
    plt.subplot(311)
    plt.plot(t,data[:,14])
    plt.ylabel('x (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -1./100., 1./100.])
    plt.ticklabel_format(axis='y', style='sci', scilimits=(-2, 2))
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,15])
    plt.ylabel('y (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -1./100., 1./100.])
    plt.ticklabel_format(axis='y', style='sci', scilimits=(-2, 2))
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,16])
    plt.ylabel('z (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -1./100., 1./100.])
    plt.ticklabel_format(axis='y', style='sci', scilimits=(-2, 2))
    plt.grid(True)
    plt.savefig('acc_bias.eps', format='eps', dpi=1000)
    pp.savefig(plt.figure(4))
    plt.close("all")

    plt.figure(5)
    plt.suptitle('q_tilde')
    plt.subplot(311)
    plt.plot(t,data[:,17]*180.0/math.pi)
    plt.ylabel('x (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,t[-1], -2., 2.])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,18]*180.0/math.pi)
    plt.ylabel('y (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([0,t[-1], -2., 2.])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,19]*180.0/math.pi)
    plt.ylabel('z (degrees)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -15., 15.])
    plt.grid(True)
    pp.savefig(plt.figure(5))
    plt.close("all")

    plt.figure(6)
    plt.suptitle('Gravity Estimate')
    plt.subplot(311)
    plt.plot(t,data[:,20])
    plt.ylabel('x (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -0.2, 0.2])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,21])
    plt.ylabel('y (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -1.2, 1.2])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,22])
    plt.ylabel('z (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -0.2, 0.2])
    plt.grid(True)
    #pp.savefig(plt.figure(6))
    plt.close("all")

    plt.figure(12)
    acc = data[:,20:23]-data[:,14:17]
    plt.suptitle('Bias Corrected Gravity Estimate')
    plt.subplot(411)
    plt.plot(t,data[:,20]-data[:,14])
    plt.ylabel('x (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -0.2, 0.2])
    plt.grid(True)
    plt.subplot(412)
    plt.plot(t,data[:,21]-data[:,15])
    plt.ylabel('y (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -1.2, 1.2])
    plt.grid(True)
    plt.subplot(413)
    plt.plot(t,data[:,22]-data[:,16])
    plt.ylabel('z (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -0.2, 0.2])
    plt.grid(True)
    plt.subplot(414)
    plt.plot(t,np.sum(np.abs(acc)**2,axis=-1)**(1./2))
    plt.xlabel('Seconds (s)')
    plt.ylabel('MAG')
    #plt.axis([0,t[-1], -0.2, 0.2])
    plt.grid(True)
    #pp.savefig(plt.figure(12))
    plt.close("all")

    plt.figure(7)
    plt.suptitle('ACC Measurement')
    plt.subplot(311)
    plt.plot(t,data[:,23])
    plt.ylabel('x (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -0.2, 0.2])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,24])
    plt.ylabel('y (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -1.2, 1.2])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,25])
    plt.ylabel('z (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -0.2, 0.2])
    plt.grid(True)
    #pp.savefig(plt.figure(7))
    plt.close("all")

    plt.figure(8)
    plt.suptitle('ANG Measurement')
    plt.subplot(311)
    plt.plot(t,data[:,26])
    plt.ylabel('x (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,27])
    plt.ylabel('y (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,28])
    plt.ylabel('z (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    #pp.savefig(plt.figure(8))
    plt.close("all")

    
    plt.figure(9)
    plt.suptitle('MAG Measurement')
    plt.subplot(311)
    plt.plot(t,data[:,29])
    plt.ylabel('x')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,30])
    plt.ylabel('y')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,31])
    plt.ylabel('z')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    #pp.savefig(plt.figure(9))
    plt.close("all")

    plt.figure(10)
    plt.suptitle('MAG Measurement')
    plt.plot(data[:,29],data[:,30])
    plt.ylabel('x')
    plt.xlabel('y')
    plt.axis([-0.3,0.3, -0.3, 0.3])
    plt.grid(True)
    #pp.savefig(plt.figure(10))
    plt.close("all")


    plt.figure(11)
    plt.suptitle('w_E_n')
    plt.subplot(311)
    plt.plot(t,data[:,11])
    plt.xlabel('Seconds (s)')
    plt.ticklabel_format(axis='y', style='sci', scilimits=(-2, 2))
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,12])
    plt.xlabel('Seconds (s)')
    plt.ticklabel_format(axis='y', style='sci', scilimits=(-2, 2))
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,13])
    plt.xlabel('Seconds (s)')
    plt.ticklabel_format(axis='y', style='sci', scilimits=(-2, 2))
    plt.grid(True)
    pp.savefig(plt.figure(11))
    plt.close("all")


    st = 1
    nd = 1000
    
    plt.figure(12)
    plt.suptitle('Delta a')
    plt.subplot(311)
    plt.plot(t[st:nd],data[st:nd,20]-9.81*data[st:nd,23])
    plt.ylabel('x (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -0.2, 0.2])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t[st:nd],data[st:nd,21]-9.81*data[st:nd,24])
    plt.ylabel('y (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -1.2, 1.2])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t[st:nd],data[st:nd,22]-9.81*data[st:nd,25])
    plt.ylabel('z (g)')
    plt.xlabel('Seconds (s)')
    #plt.axis([0,t[-1], -0.2, 0.2])
    plt.grid(True)
    #pp.savefig(plt.figure(12))
    plt.close("all")

    plt.figure(13)
    plt.suptitle('ANG Measurement')
    plt.subplot(311)
    plt.plot(t[st:nd],data[st:nd,26])
    plt.ylabel('x (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t[st:nd],data[st:nd,27])
    plt.ylabel('y (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t[st:nd],data[st:nd,28])
    plt.ylabel('z (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.grid(True)
    #pp.savefig(plt.figure(13))
    plt.close("all")



    pp.close()

        
    
if __name__ == "__main__":
    main(sys.argv[1:])
