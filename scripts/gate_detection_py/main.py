import argparse
import cv2
import numpy as np
import os
import signal
import sys
import tensorboard as tb
import tensorflow as tf
import time as tm
from scipy import ndimage

from cv2 import VideoCapture
FLAGS = None

# Default Webcame Colormapping - BGR
cap, writer, edit = None, None, None
graph = tf.Graph()

def open_stream():
	# Uses video stream unless otherwise specified
	if FLAGS.video_stream:
		input_mode = 0
		cap = cv2.VideoCapture(input_mode)
		if not cap.isOpened():
			print('Unable to open video stream ', input_mode)
			return None
	else:
		# Looks for video file inside Flags.data_dir directory
		file_name = os.path.join(FLAGS.data_dir, '*.avi')
		if not os.path.exists(file_name):
			print('Could not find suitable video file in ', FLAGS.data_dir)
			return None
		cap = cv2.VideoCapture(os.path.join(FLAGS.data_dir, '*.avi'))
		if not cap.isOpened():
			print('Video file could not be opened: ', file_name)
			return None
	return cap


def open_src():
	if not os.path.exists('test_img.jpg'):
		print('could not find test_img.jpg')
	else:
		cap = cv2.VideoCapture('test_img.jpg')
		if not cap.isOpened():
			print('Unable to open video stream on test_img.jpg')
			return None
		else:
			return cap


def open_video_writer(cap, record='edited.avi'):
	fourcc = int(cv2.VideoCapture.get(cap, cv2.CAP_PROP_FOURCC))
	fps = int(cv2.VideoCapture.get(cap, cv2.CAP_PROP_FPS))
	width = int(cv2.VideoCapture.get(cap, cv2.CAP_PROP_FRAME_WIDTH))
	height = int(cv2.VideoCapture.get(cap, cv2.CAP_PROP_FRAME_HEIGHT))
	print(fourcc,fps,width,height)
	if not os.path.isdir(FLAGS.data_dir):
		os.makedirs(FLAGS.data_dir)
	if os.path.exists(os.path.join(FLAGS.data_dir, record)):
		os.remove(os.path.join(FLAGS.data_dir, record))
	if not cap.isOpened:
		print('No video stream to record.')
		return None
	writer = cv2.VideoWriter(os.path.join(FLAGS.data_dir, record), cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), fps, (width, height))
	# writer = cv2.VideoWriter(os.path.join(FLAGS.data_dir, record), fourcc, fps, (width, height))
	return writer


def close_stream(cap):
	print('Closing video stream: ...')
	if cap is not None and cap.isOpened():
		cap.release()
		print('video stream closed.')
	else:
		print('already closed.')


def close_video_writer(writer):
	print('Closing recording stream ...')
	if writer is not None and writer.isOpened:
		print('recording stream closed.')
		writer.release()
	else:
		print('already closed.')


def create_graph():
	start = tm.time()
	with graph.as_default():
		input_placeholder = tf.placeholder(tf.uint8,shape=[640,480,3])
		# tf_frame = tf.convert_to_tensor(input_placeholder,np.uint8)
		tf_frame_f32 = tf.image.convert_image_dtype(input_placeholder,tf.float32)
		reshaped = tf.expand_dims(tf_frame_f32,0)
		# print(reshaped)
		sobel = tf.image.sobel_edges(reshaped)
		# fake_sobel = tf.expand_dims(reshaped,4)
		# sobel = tf.tile(fake_sobel,[1,1,1,1,2])
		# print(sobel)
		sob_abs = tf.abs(sobel,'sob_abs')
		sob_squeeze = tf.squeeze(sob_abs)
		sob_img = tf.reduce_max(sob_squeeze,3,False)
		sob_u8 = tf.image.convert_image_dtype(sob_img,dtype=tf.uint8)
		max_val = tf.reduce_max(tf.reduce_max(tf.reduce_max(sob_u8,0),0),0)
		sob_img_u8_norm = tf.scalar_mul(255/max_val,sob_u8)

	# throw_away_test = sess.run(sob_img_u8_norm)

	end = tm.time()
	print(end-start)

def process_image2(frame,sess):
	start = tm.time()
	nd_sobel = cv2.cvtColor(ndimage.sobel(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)),cv2.COLOR_GRAY2RGB)
	print(np.shape(nd_sobel))
	print(nd_sobel)
	return nd_sobel

def kill_signal(signum,frame):
	close_stream(cap)
	close_stream(edit)
	close_stream(writer)
	cv2.destroyAllWindows()
	sys.exit(0)


def conv_array(scale):
	arr = np.ones([scale, scale], dtype=np.float32)


def get_local_maxima(in_tensor,scale):
	in_tensor = tf.expand_dims(in_tensor,3,name='R_expanded')
	max_pooled_in_tensor = tf.nn.pool(in_tensor, window_shape=(scale, scale), pooling_type='MAX', padding='SAME')
	maxima = tf.where(tf.equal(in_tensor, max_pooled_in_tensor), in_tensor, tf.zeros_like(in_tensor))
	return tf.squeeze(maxima,3)


def compute_M(dx, dy, scale, step_size = 1):
	# dx, dy shape: [1, height, width, 1]
	# kernel shape: [filter_height, filter_width, in_channels, out_channels]
	dxdx = tf.multiply(dx, dx, name='dx_sqr')
	dxdy = tf.multiply(dx, dy, name='dx_dy')
	dydy = tf.multiply(dy, dy, name='dy_sqr')
	conv_kernel = tf.constant(1.0/scale, tf.float32, [scale, scale, 1, 1])
	M11 = tf.nn.conv2d(dxdx, conv_kernel, [1, step_size, step_size, 1], 'SAME', name='M11')
	M12 = tf.nn.conv2d(dxdy, conv_kernel, [1, step_size, step_size, 1], 'SAME', name='M12')
	M21 = tf.tile(M12, [1,1,1,1], name='M21')
	M22 = tf.nn.conv2d(dydy, conv_kernel, [1, step_size, step_size, 1], 'SAME', name='M22')
	M = tf.squeeze(tf.stack([tf.stack([M11, M12], 3, name='M_top'), tf.stack([M21, M22], 3, name='M_bot')], 3, name='M'), 5)
	tf.summary.histogram('M',M)
	# output with same scale as input
	return M


def compute_R(sobel_single_channel, scale, k):
	dx = tf.slice(sobel_single_channel, [0, 0, 0, 0], [-1, -1, -1, 1], name = 'dx')
	dy = tf.slice(sobel_single_channel, [0, 0, 0, 1], [-1, -1, -1, 1], name = 'dy')
	tf.summary.histogram('dx',dx)
	tf.summary.histogram('dy',dy)
	M = compute_M(dx, dy, scale)
	M_det = tf.matrix_determinant(M, name = 'M_det')
	M_trace = tf.scalar_mul(k, tf.square(tf.trace(M)))
	R = tf.subtract(M_det, M_trace,name='R')
	# output with same scale as input
	return tf.squeeze(get_local_maxima(R,scale), name='R_max_pooled')


def find_corners_in_channel(channel, scale, k, mask):
	# Compute corner response at given scale
	# apply hue mask
	# find best corner in channel
	R = compute_R(channel, scale, k)
	tf.summary.histogram('R',R)
	if not mask is None:
		masked = tf.where(mask, R, tf.zeros([480, 640]), name='masked')
	else:
		masked = R

	return masked


def find_corners(img, scale_set, mask, k=0.4):
	sobel = tf.image.sobel_edges(img)
	sob_r = tf.squeeze(tf.slice(sobel, [0, 0, 0, 0, 0], [-1, -1, -1, 1, -1]), 3, name = 'sob_r')
	sob_g = tf.squeeze(tf.slice(sobel, [0, 0, 0, 1, 0], [-1, -1, -1, 1, -1]), 3, name = 'sob_g')
	sob_b = tf.squeeze(tf.slice(sobel, [0, 0, 0, 2, 0], [-1, -1, -1, 1, -1]), 3, name = 'sob_b')
	# channels = [sob_r, sob_g, sob_b]
	channels = [sob_r]
	top_n = 2
	corners = []
	for scale in range(scale_set[0], scale_set[2], scale_set[1]):
		corner_scale_subset = []
		for channel in channels:
			corner_scale_subset.append(find_corners_in_channel(channel, scale, k, mask))
		corner_scale_subset_3chan = tf.stack(corner_scale_subset, 2, name = 'scale_subset_corners_3chan')
		best_across_channels = find_top_corners(corner_scale_subset_3chan)
		corners.append(best_across_channels)
	corners_across_scale = find_top_corners(tf.stack(corners, 2, name = 'total_corners_n_scale'))
	print(corners_across_scale)
	row_values, row_indices = tf.nn.top_k(corners_across_scale, top_n, name = 'best_in_row')
	col_values, col_indices = tf.nn.top_k(tf.transpose(row_values), top_n, name = 'best_in_col')
	tf.summary.histogram('corners_across_scale', corners_across_scale)
	top_values = []
	top_indices = []
	for i in range(0, top_n, 1):
		top_val_temp = []
		top_ind_temp = []
		for j in range(0, top_n, 1):
			row = tf.gather_nd(col_indices, [i, j])
			column = tf.gather_nd(row_indices, [row, i])
			value = tf.gather_nd(corners_across_scale, [row, column])
			top_val_temp.append(value)
			top_ind_temp.append([row, column])
		top_values.append(top_val_temp)
		top_indices.append(top_ind_temp)
	tf.summary.histogram('top_values', top_values)
	tf.summary.histogram('top_indices', top_values)
	return top_indices

def find_top_corners(set):
	values, indices = tf.nn.top_k(set, 1, name='top_corners_at_pixel')
	return tf.squeeze(values, name='pixel_topn_squeeze')


def color_raw(input_img, corner_locations, dot_size):
	frame = input_img
	for locs_2 in corner_locations:
		for point in locs_2:
			frame = cv2.circle(frame,(point[0],point[1]),dot_size,(255,0,0),-1)
	return frame


def main(_):
	if tf.gfile.Exists(FLAGS.log_dir):
		tf.gfile.DeleteRecursively(FLAGS.log_dir)
	tf.gfile.MakeDirs(FLAGS.log_dir)
	global cap, writer, edit
	cap = open_stream()
	edit = open_video_writer(cap)
	raw = open_video_writer(cap,'unedited.avi')
	with tf.Session(graph=graph) as sess:
		writer = tf.summary.FileWriter(FLAGS.log_dir, sess.graph)

		with graph.as_default():
			input_placeholder = tf.placeholder(tf.uint8, shape=[480, 640, 3])
			mask_placeholder = tf.placeholder(tf.bool, shape=[480, 640])
			frame_f32 = tf.image.convert_image_dtype(input_placeholder, tf.float32)
			reshaped = tf.expand_dims(frame_f32, 0)
			scale = [4,8,32]
			corners = find_corners(reshaped, scale, mask_placeholder)


		i = 0
		merged = tf.summary.merge_all()
		tf.initialize_all_variables
		# sess.run(tf.initialize_all_variables)
		while True:
			ret, frame = cap.read()

			if ret == True:
				start = tm.time()
				frame_rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
				frame_hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
				frame_hsv_thresh = cv2.inRange(frame_hsv, np.array([0, 180, 120]), np.array([30, 255, 255]))
				frame_rgb_masked = cv2.bitwise_and(frame_rgb, frame_rgb, mask=frame_hsv_thresh)
				# Process image:
				img_tensor = tf.convert_to_tensor(frame_rgb, tf.uint8)
				# img_tensor = tf.convert_to_tensor(frame_hsv, tf.uint8)
				mask_tensor = tf.convert_to_tensor(frame_hsv_thresh.astype(bool), tf.bool)
				summary, coords = sess.run([merged, corners], feed_dict={input_placeholder: img_tensor.eval(), mask_placeholder: mask_tensor.eval()})
				if(i == 0):
					writer.add_summary(summary, i)
					writer.add_graph(sess.graph)
				print(coords)
				marked = color_raw(frame, coords, 10)
				# sobel_intermediate = sess.run(sob_output_vis, feed_dict={input_placeholder:img_tensor.eval()})

				# Display the resulting frame
				if FLAGS.show_stream:
					cv2.imshow('Raw', frame)
					cv2.imshow('masked',cv2.cvtColor(frame_rgb_masked,cv2.COLOR_RGB2BGR))
				if FLAGS.show_processed_stream:
					cv2.imshow('Edited', marked)
					cv2.imshow('Sobel',sobel_intermediate)

				# Press Q on keyboard to stop recording
				if cv2.waitKey(1) & 0xFF == ord('q'):
					print('Manually Stopped (pressed q)')
					break
				end = tm.time()
				print(end - start)

			# Break the loop
			else:
				print('Reached end of video file')
				break
			if(i > 2):
				break
			i += 1
		writer.close()

	close_stream(cap)
	close_video_writer(edit)
	close_video_writer(raw)
	cv2.destroyAllWindows()

if __name__ == '__main__':
	signal.signal(signal.SIGINT, kill_signal)
	signal.signal(signal.SIGTERM, kill_signal)
	parser = argparse.ArgumentParser()
	parser.add_argument('--fake_data', nargs='?', const=True, type=bool, default=False,
	                    help='If true, uses fake data for unit testing.')
	parser.add_argument('--show_stream', nargs='?', const=True, type=bool, default=True,
	                    help='If true, uses imshow to show unproccessed stream realtime.')
	parser.add_argument('--show_processed_stream', nargs='?', const=True, type=bool, default=True,
	                    help='If true, uses imshow to show processed stream realtime.')
	parser.add_argument('--max_steps', type=int, default=1000, help='Number of steps to run trainer.')
	parser.add_argument('--learning_rate', type=float, default=0.001, help='Initial learning rate')
	parser.add_argument('--dropout', type=float, default=0.9, help='Keep probability for training dropout.')
	parser.add_argument('--video_stream', nargs='?', const=True, type=bool, default=True,
	                    help='If false, uses video file in path specified by \
	                    --data_dir, otherwise uses cv2.VideoCapture(0)')
	parser.add_argument('--data_dir', type=str,
	                    default=os.path.join(os.getenv('ADR_DATA_DIR', '/tmp'), 'adr/video_data'),
	                    help='Directory for storing input data')
	parser.add_argument('--log_dir', type=str,
	                    default=os.path.join(os.getenv('ADR_DATA_DIR', '/tmp'), 'adr/tensorflow_data'),
	                    help='TensorFlow summaries log directory')
	FLAGS, unparsed = parser.parse_known_args()
	tf.app.run(main=main, argv=[sys.argv[0]] + unparsed)