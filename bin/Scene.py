#!/usr/bin/env python
# _*_ coding: utf-8 _*_
# @Time : 2022/1/18 下午4:55
# @Author : Garen_Lee
# @Version：V 0.1
# @File : TxtReader.py
# @desc : 读取txt文档并可视化对比

from cmath import cos
import sys
from xml.etree.ElementTree import PI
import numpy as np
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import math

def TxtReaderCompare(dir1, dir2, dir3, dir4):
	pi = 3.1415926
	vis_box_x, vis_box_y = [], []
	if 1:
		f1 = open(dir1, "r")
		line1 = f1.readline()
		test_x, test_y, test_h = [], [], []
		while line1:
			linedate1 = line1.split(",")
			if line1 == "\n":
				break
			test_x.append(float(linedate1[0]))
			test_y.append(float(linedate1[1]))
			test_h.append(float(linedate1[2]) * pi / 180)
			line1 = f1.readline()
		for i in range(0, len(test_x)):
			vis_box_x.clear()
			vis_box_y.clear()
			front_x = test_x[i] + math.cos(test_h[i]) * 2.5
			front_y = test_y[i] + math.sin(test_h[i]) * 2.5
			vis_box_x.append(front_x - math.sin(test_h[i]) * 1.5)
			vis_box_y.append(front_y + math.cos(test_h[i]) * 1.5)
			rear_x = test_x[i] - math.cos(test_h[i]) * 2.5
			rear_y = test_y[i] - math.sin(test_h[i]) * 2.5
			vis_box_x.append(rear_x - math.sin(test_h[i]) * 1.5)
			vis_box_y.append(rear_y + math.cos(test_h[i]) * 1.5)
			vis_box_x.append(rear_x + math.sin(test_h[i]) * 1.5)
			vis_box_y.append(rear_y - math.cos(test_h[i]) * 1.5)
			vis_box_x.append(front_x + math.sin(test_h[i]) * 1.5)
			vis_box_y.append(front_y - math.cos(test_h[i]) * 1.5)
			plt.plot(vis_box_x, vis_box_y, "k")
			vis_box_x.clear()
			vis_box_y.clear()
			vis_box_x.append(front_x + math.sin(test_h[i]) * 1.5)
			vis_box_y.append(front_y - math.cos(test_h[i]) * 1.5)
			vis_box_x.append(front_x - math.sin(test_h[i]) * 1.5)
			vis_box_y.append(front_y + math.cos(test_h[i]) * 1.5)
			plt.plot(vis_box_x, vis_box_y, "r")
		plt.plot(test_x, test_y, "go-")

	if 1:
		f2 = open(dir2, "r")
		line2 = f2.readline()
		opt_x, opt_y, opt_h = [], [], []
		while line2:
			linedate2 = line2.split(",")
			if line2 == "\n":
				break
			opt_x.append(float(linedate2[0]))
			opt_y.append(float(linedate2[1]))
			opt_h.append(float(linedate2[2]))
			line2 = f2.readline()
		for i in range(0, len(opt_x)):
			vis_box_x.clear()
			vis_box_y.clear()
			front_x = opt_x[i] + math.cos(opt_h[i]) * 2.5
			front_y = opt_y[i] + math.sin(opt_h[i]) * 2.5
			vis_box_x.append(front_x - math.sin(opt_h[i]) * 2.5)
			vis_box_y.append(front_y + math.cos(opt_h[i]) * 2.5)
			rear_x = opt_x[i] - math.cos(opt_h[i]) * 2.5
			rear_y = opt_y[i] - math.sin(opt_h[i]) * 2.5
			vis_box_x.append(rear_x - math.sin(opt_h[i]) * 2.5)
			vis_box_y.append(rear_y + math.cos(opt_h[i]) * 2.5)
			vis_box_x.append(rear_x + math.sin(opt_h[i]) * 2.5)
			vis_box_y.append(rear_y - math.cos(opt_h[i]) * 2.5)
			vis_box_x.append(front_x + math.sin(opt_h[i]) * 2.5)
			vis_box_y.append(front_y - math.cos(opt_h[i]) * 2.5)
			plt.plot(vis_box_x, vis_box_y, "k")
			vis_box_x.clear()
			vis_box_y.clear()
			vis_box_x.append(front_x + math.sin(opt_h[i]) * 2.5)
			vis_box_y.append(front_y - math.cos(opt_h[i]) * 2.5)
			vis_box_x.append(front_x - math.sin(opt_h[i]) * 2.5)
			vis_box_y.append(front_y + math.cos(opt_h[i]) * 2.5)
			plt.plot(vis_box_x, vis_box_y, "r")
		plt.plot(opt_x, opt_y, "bo-")

	if 0:
		f3 = open(dir3, "r")
		line3 = f3.readline()
		x_min_box, y_min_box, x_max_box, y_max_box = [], [], [], []
		while line3:
			linedate3 = line3.split(",")
			if line3 == "\n":
				break
			x_min_box.append(float(linedate3[0]))
			y_min_box.append(float(linedate3[1]))
			x_max_box.append(float(linedate3[2]))
			y_max_box.append(float(linedate3[3]))
			line3 = f3.readline()
		x_box, y_box = [], []
		for i in range(0, len(x_min_box)):
			x_box.clear()
			y_box.clear()
			x_box.append(x_min_box[i])
			y_box.append(y_min_box[i])
			x_box.append(x_max_box[i])
			y_box.append(y_min_box[i])
			x_box.append(x_max_box[i])
			y_box.append(y_max_box[i])
			x_box.append(x_min_box[i])
			y_box.append(y_max_box[i])
			x_box.append(x_min_box[i])
			y_box.append(y_min_box[i])
			plt.plot(x_box, y_box, "r")

	if 1:
		f4 = open(dir4, "r")
		line4 = f4.readline()
		obs_x, obs_y = [], []
		while line4:
			linedate4 = line4.split(",")
			if line4 == "\n":
				break
			obs_x.append(float(linedate4[0]))
			obs_y.append(float(linedate4[1]))
			line4 = f4.readline()
		plt.plot(obs_x, obs_y, "yo")

	plt.axis("equal")
	plt.show()


if __name__=="__main__":
	TxtReaderCompare("./resultData/referPath.txt", "./resultData/optTraj.txt", "./resultData/boxfile.txt", "./resultData/obsfile.txt")
