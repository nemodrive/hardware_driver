import signal  # ensure graceful exit
import yaml
import time

# from streamer import SharedMemStreamer
from streamer_simulator import SimulatedStreamer
from recording.recorders import Player
from network.threaded_server import ThreadedBroadcastServer, NemoStreamManager, NEMO_STREAM_MANAGER_KEY
import multiprocessing
import logging

logger = multiprocessing.log_to_stderr()
logger.setLevel(logging.INFO)
logger.propagate = False

server_running = True


def _handle_signal(signum, frame):
    global server_running
    print("Server now stopping...")
    server_running = False


signal.signal(signal.SIGINT, _handle_signal)


def main():
    # Create synchronized sensor stream service
    # streamer = SharedMemStreamer()
    # streamer = SimulatedStreamer()

    # streamer.stream_generator(time_delayed=True, simulator_delay=None)

    with Player("./saved_datasets/_c_test_automatica_1") as p:
        source_stream = p.stream_generator(loop=True)

        # read config for network streaming
        with open("config.yaml", "r") as f:
            settings = yaml.load(f, Loader=yaml.SafeLoader)["network_streaming"]

        # create collector_running flag and worker_list and register them with the StreamManager class

        # instantiate synchronization Manager for relaying sensor data to different clients
        stream_sync_manager = NemoStreamManager(address=(settings["sync_manager_host"],
                                                         settings["sync_manager_port"]),
                                                authkey=NEMO_STREAM_MANAGER_KEY)

        stream_sync_manager.start()
        stream_workers = stream_sync_manager.list()
        collector_running = stream_sync_manager.Event()
        collector_running.set()

        with ThreadedBroadcastServer(collector_running, stream_workers,
                                     settings["sync_manager_host"],
                                     settings["sync_manager_port"],
                                     settings["host"], settings["port"], settings["header_size"]) as server:

            input("Press Enter to begin ...")

            previous_packet_datetime = None

            for data_packet in source_stream:

                total_elapsed_this_packet = time.time()

                if "images" in data_packet.keys():
                    del data_packet["images"]

                if not server_running:
                    break

                # TODO optionally compress images

                server.send_object(data_packet)
                print(data_packet)

                # simulate delay

                if previous_packet_datetime is None:
                    previous_packet_datetime = data_packet['datetime']
                else:

                    total_elapsed_this_packet = time.time() - total_elapsed_this_packet

                    required_delay = (data_packet['datetime'] - previous_packet_datetime).total_seconds() - total_elapsed_this_packet
                    previous_packet_datetime = data_packet['datetime']

                    if required_delay > 0:
                        time.sleep(required_delay)

    stream_sync_manager.shutdown()

    logger.info("Stopped Stream Server")


if __name__ == '__main__':
    main()
