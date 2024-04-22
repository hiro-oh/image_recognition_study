#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

"""
２画像のヒストグラム比較による類似度の算出
【引用元】
https://qiita.com/jun_higuche/items/752ef756a182261fcc55
"""
import cv2, os
# from opencv_japanese import imread

image_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'images')

print(image_dir)

# 画像ファイルのパスを作成
image_path1 = os.path.join('../images/a.jpg')
image_path2 = os.path.join('../images/b.jpg')
# 画像を読み込む
image1 = cv2.imread(image_path1)
image2 = cv2.imread(image_path2)

print(type(image1))

cv2.imshow('YOLO V8 Detection', image1)  

height = image1.shape[0]
width = image1.shape[1]

img_size = (int(width), int(height))

# 比較するために、同じサイズにリサイズ
image1 = cv2.resize(image1, img_size)
image2 = cv2.resize(image2, img_size)

# 画像をヒストグラム化する
image1_hist = cv2.calcHist([image1], [2], None, [256], [0, 256])
image2_hist = cv2.calcHist([image2], [2], None, [256], [0, 256])

# ヒストグラムした画像を比較
print("「1_1.png」と「1_2.png」の類似度：" + str(cv2.compareHist(image1_hist, image2_hist, 0)))