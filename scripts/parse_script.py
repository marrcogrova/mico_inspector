#!/usr/bin/env python

from LogParser import *
import sys
import matplotlib.pyplot as plt

log_file = str(raw_input("Introduce log file: "))

parser = LogParser(log_file)

print(parser.listTags())

while True:
	tag = str(raw_input("Introduce tag or list of tags separated by commas: "))
	
	if ',' in tag:
		for x in tag.split(','):
			vector = parser.vectorFloat(x)
			if len(vector) > 1:	
				plt.plot(vector)
				plt.ylabel("time")
			else:
				print("Tag not found")
		plt.legend(tag.split(','))		
		plt.show()
	else:
		vector = parser.vectorFloat(tag)
		if len(vector) > 1:	
			plt.plot(vector)
			plt.ylabel(tag)
			plt.show()
		else:
			print("Tag not found")

