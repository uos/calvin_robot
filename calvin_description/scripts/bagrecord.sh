#!/bin/sh
rosbag record $(for i in $(cat $(dirname $0)/bagtopics2); do echo -n "$i " ; done;)
