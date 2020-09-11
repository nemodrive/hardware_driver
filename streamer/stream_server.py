import signal  # ensure graceful exit
import yaml

# from streamer import SharedMemStreamer
from streamer_simulator import SimulatedStreamer
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
    streamer = SimulatedStreamer()

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

        for data_packet in streamer.stream_generator(time_delayed=True, simulator_delay=None):

            if not server_running:
                break

            # TODO optionally compress images

            server.send_object(data_packet)
            print(data_packet)

    stream_sync_manager.shutdown()

    logger.info("Stopped Stream Server")


if __name__ == '__main__':
    main()
