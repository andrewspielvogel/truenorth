#!/usr/bin/python

import sys, getopt, datetime

def main(argv):
    hz = '5000'
    lat = '39.32'
    o_file = ''
    i_file = ''
    c_file = ''
    rpy_align = '[0,0,0]'
    rpy_r0 = '[0,0,0]'
    k_acc = '[0.1,0.1,0.1]'
    k_acc_bias = '[0.01,0.01,0.1]'
    k_ang_bias = '[0.0001,0.0001,0.01]'
    k_g = '[1,1,1]'
    k_north = '[1,1,1]'
    k_E_n = '[0.005,0.005,0.005]'
    
    try:
        opts,args = getopt.getopt(argv,"hi:o:c:l:a:R:k:z:",["ifile=","help","ofile=","hz=","lat=","cfile=","rpy_align=","rpy_Ro=","k_acc=","k_acc_bias","k_ang_bias","k_g","k_north","k_E_n"])
    except getopt.GetoptError:
        print "USAGE:"
        print 'gen_config_file.py -i <KVHfile> -o <estimatoroutputfile> -c <configfile_generated>'
        sys.exit(2)
    for opt,arg in opts:
        if opt in ("-h","--help"):
            print "USAGE:"
            print 'gen_config_file.py -i <KVHfile> -o <estimatoroutputfile> -c <outputtedconfigfile>'
            print "-i , --ifile : .KVH file input to attitude estimator."
            print "-o , --ofile : .csv file output of attitude estimator."
            print "-c , --cfile : .m file to save attitude estimator config file."
            print "-z , --hz    : Sampling frequency (s^-1)."
            print "-l , --lat   : Latitude (degrees)."
            print "-a , --rpy_align : [roll, pitch, yaw] (radians)."
            print "-R , --rpy_Ro    : [roll, pitch, yaw] (radians)."
            print "--k_acc (diag)."
            print "--k_acc_bias (diag)."
            print "--k_ang_bias (diag)."
            print "--k_E_n (diag)."
            sys.exit()
        elif opt in ("-i","--ifile"):
            i_file = "\"" + arg + "\""
        elif opt in ("-o","--ofile"):
            o_file = "\"" + arg + "\""
        elif opt in ("-c","--cfile"):
            c_file = arg
        elif opt in ("-z","--hz"):
            hz = arg
        elif opt in ("-l","--lat"):
            lat = arg
        elif opt in ("-a","--rpy_align"):
            rpy_align = arg
        elif opt in ("-R","--rpy_Ro"):
            rpy_Ro = arg
        elif opt in ("-k"):
            k = arg
        elif opt in ("--k_acc"):
            k_acc = arg
        elif opt in ("--k_acc_bias"):
            k_acc_bias = arg
        elif opt in ("--k_ang_bias"):
            k_ang_bias = arg
        elif opt in ("--k_E_n"):
            k_E_n = arg

    now = datetime.datetime.now()

    print "GENERATING CONFIG FILE: " + c_file
    
    file = open(c_file,"w")
    file.write("last_mod = " + "\"" + now.isoformat() + "\"" + "\n")
    file.write("hz = " + hz + "\n")
    file.write("lat = " + lat + "\n")
    file.write("o_file = " + o_file + "\n")
    file.write("i_file = " + i_file + "\n")
    file.write("rpy_align = " + rpy_align + "\n")
    file.write("rpy_Ro = " + rpy_Ro + "\n")
    file.write("k_acc = " + k_acc + "\n")
    file.write("k_acc_bias = " + k_acc_bias + "\n")
    file.write("k_ang_bias = " + k_ang_bias + "\n")
    file.write("k_E_n = " + k_E_n + "\n")
    file.close()


if __name__ == "__main__":
    main(sys.argv[1:])
