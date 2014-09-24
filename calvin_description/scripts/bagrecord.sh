#!/bin/sh
rosbag record "$@" $(cat $(dirname $0)/bagtopics)
