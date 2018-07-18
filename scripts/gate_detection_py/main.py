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



def main(_):
	if tf.gfile.Exists(FLAGS.log_dir):
		tf.gfile.DeleteRecursively(FLAGS.log_dir)
	tf.gfile.MakeDirs(FLAGS.log_dir)
	global cap, writer, edit
	cap = open_stream()
	edit = open_video_writer(cap)
	raw = open_video_writer(cap,'unedited.avi')

	with graph.as_default():
		input_placeholder = tf.placeholder(tf.uint8,shape=[480,640,3])
		tf_frame_f32 = tf.image.convert_image_dtype(input_placeholder,tf.float32)
		reshaped = tf.expand_dims(tf_frame_f32,0)
		sobel = tf.image.sobel_edges(reshaped)
		sob_squeeze = tf.squeeze(sobel)
		scale = 6
		dx = tf.expand_dims(tf.squeeze(tf.slice(sobel, [0, 0, 0, 0], [-1, -1, -1, 1])),0)
		dy = tf.expand_dims(tf.squeeze(tf.slice(sobel, [0, 0, 0, 1], [-1, -1, -1, 1])),0)
		conv_kernel = tf.constant(1, tf.float32, [scale, scale])
		Ix = tf.nn.conv2d(dx,conv_kernel,scale,'SAME',True)
		Iy = tf.nn.conv2d(dy,conv_kernel,scale,'SAME',True)
		M1 = tf.multiply(Ix,Ix)
		M2 = tf.multiply(Ix,Iy)
		M3 = tf.multiply(Iy,Ix)
		M4 = tf.multiply(Iy,Iy)
		M = tf.stack(M1,M2,M3,M4,)

		# sob_img = tf.reduce_max(sob_squeeze,3,False)
		# sob_abs = tf.abs(sob_img,'sob_abs')
		# sob_u8 = tf.image.convert_image_dtype(sob_abs,dtype=tf.uint8)
		# max_val = tf.reduce_max(tf.reduce_max(tf.reduce_max(sob_u8,0),0),0)
		# sob_img_u8_norm = tf.scalar_mul(255/max_val,sob_u8)



	with tf.Session(graph=graph) as sess:
		# writer = tf.summary.FileWriter(FLAGS.log_dir,sess.graph())
		tf.initialize_all_variables
		while True:
			ret, frame = cap.read()

			if ret == True:
				start = tm.time()
				# Process image:
				img_tensor = tf.convert_to_tensor(frame, tf.uint8)
				cvted = sess.run(sob_img_u8_norm,feed_dict={input_placeholder:img_tensor.eval()})
				# cvted = process_image(frame, sess)

				# Record processed frame and unprocessed frame:
				raw.write(frame)
				edit.write(cvted)

				# Display the resulting frame
				if FLAGS.show_stream:
					cv2.imshow('Raw', frame)
				if FLAGS.show_processed_stream:
					cv2.imshow('Edited', cvted)

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