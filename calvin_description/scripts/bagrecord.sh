#!/bin/sh
rosbag record $(for i in $(cat bagtopics2); do echo -n "$i " ; done;)
