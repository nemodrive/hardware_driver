from recording.recorders import VideoWriteBuffer, ThreadedVideoWriteBuffer
import numpy as np
import time

if __name__ == '__main__':

	standard_buffer = VideoWriteBuffer("./test_default.mp4", (2064, 1544))
	standard_buffer2 = VideoWriteBuffer("./test_default2.mp4", (2064, 1544))
	standard_buffer3 = VideoWriteBuffer("./test_default3.mp4", (2064, 1544))

	write_start_time = time.time()

	for frame_num in range(100):

		rand_frame = np.random.randint(0, 255, size=(2064, 1544, 3), dtype=np.uint8)
		#np.random.random((2064, 1544, 3))

		frame_write_time = time.time()

		standard_buffer.write_frame(rand_frame)
		standard_buffer2.write_frame(rand_frame)
		standard_buffer3.write_frame(rand_frame)

		delay = time.time() - frame_write_time

		print(f"frame written in {delay}, fps {1/delay}")

	print(f"standard writer finished in {time.time() - write_start_time}")

	standard_buffer.close()

