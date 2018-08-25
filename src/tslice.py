#!/usr/bin/env python

import sys
import argparse
import datetime as dt
import math

time_fmt_str = "%d %b %Y %H:%M:%S"
line_sep = '--------------------------------'

def main():

  # parse args
  parser = argparse.ArgumentParser(
    description='Takes a dsl formatted ascii input file, a begin time, and either a end time'
    + ' or a duration in seconds and produces a dsl formatted ascii output file that contains'
    + ' only the lines from the input file that fall within the computed interval.')

  # files (required positional args)
  parser.add_argument('input', help='input file path')
  parser.add_argument('output', help='output file path')

  # time slicing (optional args)
  parser.add_argument('--begin_time', nargs=6, type=int,
    metavar=('YEAR','MONTH','DAY','HOURS', 'MINUTES', 'SECONDS'),
    help='formatted as: year(int) month(int) day(int) hour(int) minute(int) seconds(float)')

  parser.add_argument('--end_time', nargs=6, type=int,
    metavar=('YEAR', 'MONTH', 'DAY', 'HOURS', 'MINUTES', 'SECONDS'),
    help='formatted as: year(int) month(int) day(int) hour(int) minute(int) seconds(float)')

  parser.add_argument('--duration', type=int,
    help='an amount in seconds')

  args = parser.parse_args();

  # get the file handles
  input_file = open(args.input, 'r')
  output_file = open(args.output, 'w')

  # determine start time
  if args.begin_time != None:
    begin_time = dt.datetime(args.begin_time[0], args.begin_time[1], args.begin_time[2],
      args.begin_time[3], args.begin_time[4], args.begin_time[5])
  else:
    begin_time = dt.datetime(1900, 1, 1, 0, 0, 0, 0)

  # determine end time
  if args.end_time != None and args.duration != None:
    print "error: can't specify both end time and duration for a slice"
    return
  elif args.end_time != None:
    end_time = dt.datetime(args.end_time[0], args.end_time[1], args.end_time[2],
      args.end_time[3], args.end_time[4], args.end_time[5])
  elif args.duration != None:
    duration = dt.timedelta(seconds=args.duration)
    end_time = begin_time + duration
  else:
    end_time = dt.datetime(dt.MAXYEAR, 12, 31, 23, 59, 59, 999999)

  print line_sep
  print 'TSLICE'
  print line_sep
  print 'slicing ', args.input
  print 'from', begin_time.strftime(time_fmt_str)
  print 'to', end_time.strftime(time_fmt_str)

  # some variables
  num_input_lines = 0
  num_good_lines = 0
  num_bad_lines = 0


  for line in input_file:

    num_input_lines = num_input_lines + 1;

    # print num_input_lines, ':', line

    # parse the stamp
    toks = line.split(' ')
    ymd = toks[1].split('/')
    hms = toks[2].split(':')

    yr = int(ymd[0])
    mh = int(ymd[1])
    dy = int(ymd[2])

    hr = int(hms[0])
    mn = int(hms[1])
    sc = int(math.floor(float(hms[2])))
    us = int((float(hms[2]) - sc) * 1000000)

    stamp = dt.datetime(yr, mh, dy, hr, mn, sc, us)
    # print stamp.strftime(time_fmt_str)

    if stamp > begin_time and stamp < end_time:
      num_good_lines = num_good_lines + 1
      output_file.write(line)
    else:
      num_bad_lines = num_bad_lines + 1

    # sys.stdin.readline()

  print line_sep
  print 'Summary:'
  print '# input lines =', num_input_lines
  print '# good lines =', num_good_lines
  print '# bad lines =', num_bad_lines
  print line_sep


if __name__=='__main__':
  main()

