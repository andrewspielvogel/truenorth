#!/usr/bin/python

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
from numpy import genfromtxt
from pandas import read_csv
from matplotlib.backends.backend_pdf import PdfPages
import getopt,sys


def calc_bound(data):
    return max([(abs(data[:,0])).max(),(abs(data[:,1])).max(),(abs(data[:,2])).max()])

def plot_comp(plt,t,data,title,units):

    y_height = calc_bound(data[:,0:3])
    
    if y_height==0.0:
        y_height = 1
        
    plt.suptitle(title,y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,0])
    plt.ylabel('X (' + units + ')')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,1])
    plt.ylabel('Y (' + units + ')')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,2])
    plt.ylabel('Z (' + units + ')')
    plt.xlabel('Seconds (s)')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)

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

        phins_t = np.array(phins_data[:,5]).flatten()
        
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
    plot_comp(plt,t,data[:,11:14],'Angular Rate Bias','rad/s')
    pp.savefig(plt.figure(1))
    plt.close("all")
    
    plt.figure(1)
    plot_comp(plt,t,data[:,14:17],'Earth Rate North','rad/s')
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    plot_comp(plt,t,data[:,17:20],'Acceleration Bias','m/s^2')
    pp.savefig(plt.figure(1))
    plt.close("all")


    plt.figure(1)
    plot_comp(plt,t,data[:,20:23],'Estimated Acceleration','m/s^2')
    pp.savefig(plt.figure(1))
    plt.close("all")
    
    plt.figure(1)
    plot_comp(plt,t,data[:,23:26],'Measure Acceleration','m/s^2')
    pp.savefig(plt.figure(1))
    plt.close("all")
    
    plt.figure(1)
    plot_comp(plt,t,data[:,26:29],'Measured Angular Rate','rad/s')
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    plot_comp(plt,t,data[:,29:32],'Measured Mag','')
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
        plot_comp(plt,phins_t,phins_data[:,6:9],'PHINS Angular Rate','rad/s')
        pp.savefig(plt.figure(1))
        plt.close("all")

        plt.figure(1)
        plot_comp(plt,t,data[:,29:32],'PHINS Acceleration','m/s^2')
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
