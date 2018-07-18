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
    phins_file = ''
    phins_exists = 0
    phins_data = [[]]
    
    try:
        opts,args = getopt.getopt(argv,"he:i:o:p:",["ifile=","help","ofile=","exp=","pfile="])
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
        elif opt in ("-p","--pfile"):
            phins_file = arg
            phins_exists = 1

    if phins_exists:
        print "LOADING FILE: " + phins_file
        phins_data = read_csv(phins_file,header=None,sep='\s+|,',engine='python')
        phins_data = np.matrix(phins_data)

    phins_t = phins_data[:,5]
        
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

    t = data[:,7]#-data[0,7]

    
    plt.figure(1)
    plt.suptitle('Estimated Attitude',y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,8]*180.0/math.pi,label="KVH")
    if phins_exists:
        plt.plot(phins_t,phins_data[:,12],label="PHINS")
    plt.ylabel('Roll (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -180, 180])
    plt.grid(True)
    if phins_exists:
        plt.legend(bbox_to_anchor=(0., 1., 1., 1.), loc=3,ncol=2, mode="expand", borderaxespad=0.25, fontsize=12)
    plt.subplot(312)
    plt.plot(t,data[:,9]*180.0/math.pi)
    if phins_exists:
        plt.plot(phins_t,phins_data[:,13])
    plt.ylabel('Pitch (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -180, 180])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,10]*180.0/math.pi)
    if phins_exists:
        plt.plot(phins_t,phins_data[:,14])
    plt.ylabel('Heading (degrees)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -180, 180])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    y_height = max([abs(data[:,11:13].min()),abs(data[:,11:13].max())])
    plt.suptitle('Angular Rate Bias',y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,11])
    plt.ylabel('X (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,12])
    plt.ylabel('Y (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,13])
    plt.ylabel('Z (rad/s)')
    plt.xlabel('Seconds (s)')
    #plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    y_height = max([abs(data[:,14:16].min()),abs(data[:,14:16].max())])
    plt.suptitle('Earth Rate North',y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,14])
    plt.ylabel('X (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,15])
    plt.ylabel('Y (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,16])
    plt.ylabel('Z (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")


    plt.figure(1)
    y_height = max([abs(data[:,17:19].min()),abs(data[:,17:19].max())])
    plt.suptitle('Acceleration Bias',y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,17])
    plt.ylabel('X (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,18])
    plt.ylabel('Y (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,19])
    plt.ylabel('Z (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")


    plt.figure(1)
    y_height = max([abs(data[:,20:22].min()),abs(data[:,20:22].max())])
    plt.suptitle('Estimated Accleration',y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,20])
    plt.ylabel('x (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,21])
    plt.ylabel('Y (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,22])
    plt.ylabel('Z (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")


    plt.figure(1)
    y_height = max([abs(data[:,23:25].min()),abs(data[:,23:25].max())])
    plt.suptitle('Measured Acceleration',y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,23])
    plt.ylabel('x (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,24])
    plt.ylabel('Y (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,25])
    plt.ylabel('Z (m/s^2)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    y_height = max([abs(data[:,26:28].min()),abs(data[:,26:28].max())])
    plt.suptitle('Measured Angular Rate',y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,26])
    plt.ylabel('x (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,27])
    plt.ylabel('Y (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,28])
    plt.ylabel('Z (rad/s)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")


    plt.figure(1)
    y_height = max([abs(data[:,29:31].min()),abs(data[:,29:31].max())])
    plt.suptitle('Measured Mag',y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,29])
    plt.ylabel('x (Gauss)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,30])
    plt.ylabel('Y (Gauss)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,31])
    plt.ylabel('Z (Gauss)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")


    plt.figure(1)
    y_height = max([abs(data[:,32].min()),abs(data[:,32].max())])
    plt.suptitle('Temp',y=0.99)
    plt.plot(t,data[:,32])
    plt.ylabel('Temp (C)')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], 0,y_height+y_height/10.0])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")


    status = data[:,34:40].min()
    if status!=1:
        print "!!!!!STATUS ERROR!!!!!"
        print data[:,34:40]


    if phins_exists:

            plt.figure(1)
            plt.suptitle('PHINS Angular Rate',y=0.99)
            plt.subplot(311)
            plt.plot(phins_t,phins_data[:,6])
            plt.ylabel('X (rad/s)')
            plt.xlabel('Seconds (s)')
            plt.grid(True)
            plt.subplot(312)
            plt.plot(phins_t,phins_data[:,7])
            plt.ylabel('Y (rad/s)')
            plt.xlabel('Seconds (s)')
            plt.grid(True)
            plt.subplot(313)
            plt.plot(phins_t,phins_data[:,8])
            plt.ylabel('Z (rad/s)')
            plt.xlabel('Seconds (s)')
            plt.grid(True)
            pp.savefig(plt.figure(1))
            plt.close("all")

            plt.figure(1)
            plt.suptitle('PHINS Acceleration',y=0.99)
            plt.subplot(311)
            plt.plot(phins_t,phins_data[:,9])
            plt.ylabel('X (m/s^2)')
            plt.xlabel('Seconds (s)')
            plt.grid(True)
            plt.subplot(312)
            plt.plot(phins_t,phins_data[:,10])
            plt.ylabel('Y (m/s^2)')
            plt.xlabel('Seconds (s)')
            plt.grid(True)
            plt.subplot(313)
            plt.plot(phins_t,phins_data[:,11])
            plt.ylabel('Z (m/s^2)')
            plt.xlabel('Seconds (s)')
            plt.grid(True)
            pp.savefig(plt.figure(1))
            plt.close("all")

            plt.figure(1)
            plt.suptitle('Heave',y=0.99)
            plt.plot(phins_t,phins_data[:,15])
            plt.ylabel('Heave')
            plt.xlabel('Seconds (s)')
            plt.grid(True)
            pp.savefig(plt.figure(1))
            plt.close("all")

    pp.close()

        
    
if __name__ == "__main__":
    main(sys.argv[1:])
