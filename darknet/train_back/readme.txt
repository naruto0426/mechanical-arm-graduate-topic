生成anchors:
	./darknet detector calc_anchors train/obj.data -num_of_clusters 9 -width 192 -height 192
train:
	./darknet detector train train/obj.data train/yolo-obj.cfg