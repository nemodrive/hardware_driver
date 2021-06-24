import numpy as np
from recording.recorders import FastSeparateRecorder
from recording.binary_recorders import BinaryPlayer
import sys
from tqdm import tqdm
import cv2


if __name__ == '__main__':

	tasks = [
		# ("/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/test_energetica_v2",
		#  "/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/compressed/_c_test_energetica_v2"),
		# ("/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/test_energetica_v3",
		#  "/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/compressed/_c_test_energetica_v3"),
		# ("/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/test_rectorat1",
		#  "/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/compressed/_c_test_rectorat1"),
		# ("/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/test_rectorat3_reverse",
		#  "/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/compressed/_c_test_rectorat3_reverse"),
		("/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/prime_test_speed",
		  "/media/amiro/9f188395-f35d-4882-b13b-6ad92af9f312/nemodrive/_c_prime_test_speed"),
	]

	for raw_in_path, compressed_out_path in tasks:

		with BinaryPlayer(raw_in_path) as p:
			source_stream = p.stream_generator(loop=False)

			with FastSeparateRecorder(compressed_out_path) as r:

				for data_frame in tqdm(source_stream):

					if "images" in data_frame.keys():
						images = data_frame["images"]

						for pos, img in images.items():
							if img is not None:
								images[pos] = cv2.cvtColor(images[pos], cv2.COLOR_BAYER_RG2RGB)

						data_frame["images"] = images

					r.record_packet(data_frame)
