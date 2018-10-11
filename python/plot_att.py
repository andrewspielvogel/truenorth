#!/usr/bin/python

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
from numpy import genfromtxt
from pandas import read_csv
from matplotlib.backends.backend_pdf import PdfPages
import getopt,sys
from taxis import taxis
from taxis import tlabel
import copy

def calc_bound(data):
    return max([(abs(data[:,0])).max(),(abs(data[:,1])).max(),(abs(data[:,2])).max()])

def plot_comp(plt,time,data,title,units,step):

    y_height = calc_bound(data[:,0:3])
    
    if y_height==0.0:
        y_height = 1

    t = copy.copy(time[0:-1:step])
    label = tlabel(t);
    t = taxis(t)
    data = data[0:-1:step]
    
    plt.suptitle(title,y=0.99)
    plt.subplot(311)
    plt.plot(t,data[:,0])
    plt.ylabel('X (' + units + ')')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(312)
    plt.plot(t,data[:,1])
    plt.ylabel('Y (' + units + ')')
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(t,data[:,2])
    plt.ylabel('Z (' + units + ')')
    plt.xlabel(label)
    plt.axis([t[0],t[-1], -y_height-y_height/10.0,y_height+y_height/10.0])
    plt.grid(True)

def main(argv):

    plot_samp_skip = 1

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

    
        
    print "LOADING FILE: " + i_file
    
    params = read_csv(i_file,nrows=1,header=None)
    data = read_csv(i_file,skiprows=1,header=None)
    print "LOADED FILE: " + i_file

    #print "SUBSAMPLING"
    data = data.as_matrix()
    t = data[:,7]#-data[0,7]


    if phins_exists:
        print "LOADING FILE: " + phins_file
        phins_data = read_csv(phins_file,header=None,sep='\s+|,',engine='python')
        phins_data = np.matrix(phins_data)

        phins_t = np.array(phins_data[:,5]).flatten()

        resampled_roll    = np.array([np.interp(phins_t.astype(float),t.astype(float),data[:,8].astype(float))])
        resampled_pitch   = np.array([np.interp(phins_t.astype(float),t.astype(float),data[:,9].astype(float))])
        resampled_heading = np.array([np.interp(phins_t.astype(float),t.astype(float),data[:,10].astype(float))])
        resampled_data = np.transpose(np.concatenate(([resampled_roll,resampled_pitch,resampled_heading])))


    print "GENERATING PLOTS"

    print "SAVING TO: " + o_file
    
    pp = PdfPages(o_file)
    if params.as_matrix()[0][0]=='PARAMS':
        plt.figure(0)
        plt.axis('off')
        plt.text(0.25,0.8,"CONFIG PARAMS USED:\n",ha='left',va='center',fontsize=8)
        plt.text(0.05,0.2,"PDF File: " + o_file +
                 "\n  Date Modified: " + str(params.as_matrix()[0][1]) +
                 "\n                    Hz: " + str(params.as_matrix()[0][2]) +
                 "\n                  LAT: " + str(params.as_matrix()[0][3]) +
                 "\n Processed File: " + str(params.as_matrix()[0][4]) +
                 "\n           KVH File: " + str(params.as_matrix()[0][5]) +
                 "\nAlignment(rpy): " + str(params.as_matrix()[0][6:9]) +
                 "\n            Ro(rpy): " + str(params.as_matrix()[0][9:12]) +
                 "\nk_acc: " + str(params.as_matrix()[0][12:15]) +
                 "\nk_ang_bias: " + str(params.as_matrix()[0][15:18]) +
                 "\nk_acc_bias: " + str(params.as_matrix()[0][18:21]) +
                 "\nk_E_n: " + str(params.as_matrix()[0][21:24]) +
                 "\nk_g: " + str(params.as_matrix()[0][24:27]) +
                 "\nk_north: " + str(params.as_matrix()[0][27:30]) + "\n",ha='left',fontsize=7)
        pp.savefig(plt.figure(0))
        plt.close("all")

    
    plt.figure(1)
    plt.suptitle('Estimated Attitude',y=0.99)
    plt.subplot(311)
    plt.plot(taxis(t),data[:,8]*180.0/math.pi,label="KVH")
    if phins_exists:
        plt.plot(taxis(phins_t),phins_data[:,12],label="PHINS")
    plt.ylabel('Roll (degrees)')
    plt.axis([taxis(t)[0],taxis(t)[-1], -10, 10])
    plt.grid(True)
    if phins_exists:
        plt.legend(bbox_to_anchor=(0., 1., 1., 1.), loc=3,ncol=2, mode="expand", borderaxespad=0.25, fontsize=12)
    plt.subplot(312)
    plt.plot(taxis(t),data[:,9]*180.0/math.pi)
    if phins_exists:
        plt.plot(taxis(phins_t),phins_data[:,13])
    plt.ylabel('Pitch (degrees)')
    plt.axis([taxis(t)[0],taxis(t)[-1], -10, 10])
    plt.grid(True)
    plt.subplot(313)
    plt.plot(taxis(t),data[:,10]*180.0/math.pi)
    if phins_exists:
        plt.plot(taxis(phins_t),phins_data[:,14])
    plt.ylabel('Heading (degrees)')
    plt.xlabel(tlabel(phins_t))
    plt.axis([taxis(t)[0],taxis(t)[-1], -180, 180])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")

    if phins_exists:
        plt.figure(1)

        error = np.unwrap(np.unwrap(resampled_data)-np.unwrap(phins_data[:,12:15]*math.pi/180.0))

        rms = math.sqrt(np.dot(error[:,2],error[:,2])/len(error[:,2]))*180.0/math.pi
        plot_comp(plt,phins_t,error*180.0/math.pi,'Attitude Error -- Heading RMS Error: %f degrees' % (rms),'degrees',1)
        plt.subplot(311)
        plt.axis([taxis(t)[0],taxis(t)[-1], -1, 1])
        plt.ylabel('roll (degrees)')
        plt.subplot(312)
        plt.axis([taxis(t)[0],taxis(t)[-1], -1, 1])
        plt.ylabel('pitch (degrees)')
        plt.subplot(313)
        plt.axis([taxis(t)[0],taxis(t)[-1], -5,5])
        plt.ylabel('heading (degrees)')
        pp.savefig(plt.figure(1))
        plt.close("all")

    plt.figure(1)
    plot_comp(plt,t,data[:,14:17],'Earth Rate North','rad/s',plot_samp_skip)
    pp.savefig(plt.figure(1))
    plt.close("all")

    lat = params.as_matrix()[0][3];
    earth_rate_norm_true = (2.0* math.pi)/(24.0 * 3600.0) * math.cos(lat);
    ones = (t*0) + 1;
    
    plt.figure(1)
    plt.suptitle('Earth Rate North Norm (Actual=%.10f rad/sec)' % (earth_rate_norm_true), y=0.99)
    plt.plot(taxis(t),np.sum(np.abs(data[:,14:17])**2,axis=1)**(1./2))
    plt.plot(taxis(t), ones * earth_rate_norm_true)
    
    plt.ylabel('Norm')
    plt.xlabel(tlabel(t))
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")
    
    plt.figure(1)
    plot_comp(plt,t,data[:,11:14],'Angular Rate Bias','rad/s',plot_samp_skip)
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    plot_comp(plt,t,data[:,17:20],'Acceleration Bias','m/s^2',plot_samp_skip)
    pp.savefig(plt.figure(1))
    plt.close("all")


    plt.figure(1)
    plot_comp(plt,t,data[:,20:23],'Estimated Acceleration','m/s^2',plot_samp_skip)
    plt.subplot(311)
    plt.axis([taxis(t)[0],taxis(t)[-1], -2,2])
    plt.subplot(312)
    plt.axis([taxis(t)[0],taxis(t)[-1], 9,11])
    plt.subplot(313)
    plt.axis([taxis(t)[0],taxis(t)[-1], -2,2])
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    plot_comp(plt,t,data[:,20:23]-data[:,23:26],'da','m/s^2',plot_samp_skip)
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    plot_comp(plt,t,np.cross(data[:,23:26],data[:,20:23]-data[:,23:26]),'J(a)da','m/s^2',plot_samp_skip)
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    plot_comp(plt,t,np.cross(data[:,26:29],data[:,20:23]-data[:,23:26]),'J(w)da','m/s^2',plot_samp_skip)
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    plot_comp(plt,t,np.cross(data[:,26:29],data[:,40:43]),'J(w)vel','m/s',plot_samp_skip)
    #plot_comp(plt,t,data[:,40:43],'J(w)vel','m/s',plot_samp_skip)
    pp.savefig(plt.figure(1))
    plt.close("all")
    
    plt.figure(1)
    plot_comp(plt,t,data[:,23:26],'Measure Acceleration','m/s^2',plot_samp_skip)
    pp.savefig(plt.figure(1))
    plt.close("all")
     
    plt.figure(1)
    plot_comp(plt,t,data[:,26:29],'Measured Angular Rate','rad/s',plot_samp_skip)
    pp.savefig(plt.figure(1))
    plt.close("all")

    plt.figure(1)
    plot_comp(plt,t,data[:,29:32],'Measured Mag','gaus',plot_samp_skip)
    pp.savefig(plt.figure(1))
    plt.close("all")
    
    plt.figure(1)
    y_height = max([abs(data[:,32].min()),abs(data[:,32].max())])
    plt.suptitle('Temp',y=0.99)
    plt.plot(taxis(t),data[:,32])
    plt.ylabel('Temp (C)')
    plt.xlabel(tlabel(t))
    plt.axis([taxis(t)[0],taxis(t)[-1], 0,y_height+y_height/10.0])
    plt.grid(True)
    pp.savefig(plt.figure(1))
    plt.close("all")


    status = data[:,34:40].min()
    if status!=1:
        print "!!!!!STATUS ERROR!!!!!"
        print data[:,34:40]


    if phins_exists:

        plt.figure(1)
        plot_comp(plt,phins_t,phins_data[:,6:9],'PHINS Angular Rate','degrees/s',1)
        pp.savefig(plt.figure(1))
        plt.close("all")

        plt.figure(1)
        plot_comp(plt,phins_t,phins_data[:,9:12]*9.81,'PHINS Acceleration','m/s^2',1)
        pp.savefig(plt.figure(1))
        plt.close("all")
        
        plt.figure(1)
        plt.suptitle('Heave',y=0.99)
        plt.plot(taxis(phins_t),phins_data[:,15])
        plt.ylabel('Heave (m)')
        plt.xlabel(tlabel(phins_t))
        plt.grid(True)
        pp.savefig(plt.figure(1))
        plt.close("all")

    pp.close()

        
    
if __name__ == "__main__":
    main(sys.argv[1:])
