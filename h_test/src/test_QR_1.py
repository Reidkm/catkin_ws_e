#! /usr/bin/env python
# coding:utf8
import pyzbar.pyzbar as pyzbar
import cv2
import os
import glob as gb
import shutil
 
#imagePath = "/home/reid/SF_image/111.bmp"
#srcImg = cv2.imread(imagePath)
#cv2.imshow("Image", srcImg)
#barcodes = pyzbar.decode(srcImg)
#for barcode in barcodes:
#	barcodeData = barcode.data.decode("utf-8")
#	print(barcodeData)
#print (os.path.abspath('.'))
#dirs = "/home/reid/SF_image/"

#for i in dirs:                             # 循环读取路径下的文件并筛选输出
	#if os.path.splitext(i)[1] == ".bmp":   # 筛选csv文件
		#print (i)                            # 输出所有的csv文件
#for filename in os.listdir("."):              #listdir的参数是文件夹的路径
#	fname,fename=os.path.splitext(filename)
#	if fename == ".bmp"
#		print ( filename) 

path = gb.glob('./*.bmp')
#print(path[0])
#print(len(path))
for file_path in path:
	print(file_path)
	srcImg = cv2.imread(file_path)
	barcodes = pyzbar.decode(srcImg)
	barcodeData = ''
	for barcode in barcodes:
		barcodeData = barcode.data.decode("utf-8")
	if barcodeData.strip() == '':
		print("write")
		#shutil.copy(file_path,'./unidentified_image/')
		with open('./data.txt', 'a') as file:
			file.write(file_path+"\n")
	else :
		print(barcodeData)
		#shutil.copy(file_path,'./identified_image/')
	
