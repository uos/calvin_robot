#!/bin/sh
rosbag record "$@" $(cat bagtopics)
