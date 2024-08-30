import asyncio
import argparse
import logging
import uuid
from av.error import ExitError
import time

from aiortc import RTCPeerConnection, VideoStreamTrack, RTCSessionDescription, RTCIceCandidate
from aiortc.contrib.media import MediaRecorder, MediaRelay, MediaBlackhole
from aiortc.contrib.signaling import BYE, create_signaling, add_signaling_arguments

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 

import sys
import threading

logger = logging.getLogger("pc")


# Normalize PTS to sync with the real-time webcam feed
class RemoteVideoStreamTrack(VideoStreamTrack):
    '''
        A video track with normalized PTS
    '''
    def __init__(self, track, node ):
        super().__init__()
        self.track = track
        self.node = node

    async def recv(self):
        start_time = time.time()
        frame = await self.track.recv() # Receive frame
        frame_recv_time = time.time()
        pts, time_base = await self.next_timestamp() # Adjust timestamp
        time_stamp_time = time.time()
        # print(pts)
        frame.pts = pts
        frame.time_base = time_base
        img = frame.to_ndarray(format="bgr24") # Convert frame to img array
        img_conversion_time = time.time()

        processing_time = time.time() - start_time

        self.node.get_logger().info(f"Frame recv time: {frame_recv_time - start_time:.4f} seconds")
        self.node.get_logger().info(f"Timestamp update time: {time_stamp_time - frame_recv_time:.4f} seconds")
        self.node.get_logger().info(f"Image conversion time: {img_conversion_time - time_stamp_time:.4f} seconds")
        self.node.get_logger().info(f"Frame processing time: {processing_time:.4f} seconds")

        try:
            self.node.pubCallback(img) # Publish image
        except Exception as e:
            logger.info(f"Unexpected error running ROS Code {e}")
            return frame

        logger.info("Retruning frame.\n")
        return frame # Return frame for disposing using MediaBlackhole

# Copy-Paste Signaling
async def consume_signaling(pc, signaling ):
    while True:
        obj = await signaling.receive()

        if isinstance(obj, RTCSessionDescription):
            await pc.setRemoteDescription(obj)

            if obj.type == "offer":
                #send answer
                await pc.setLocalDescription(await pc.createAnswer())
                await signaling.send(pc.localDescription)

            elif isinstance(obj, RTCIceCandidate):
                await pc.addIceCandidate(obj)

            elif obj is BYE:
                print("Exiting..")
                break

async def answer(
        pc, 
        signaling,
        recorder,
        node
    ):
    pc_id = "PeerConncetion(%s)" % uuid.uuid4()
    
    def log_info(msg, *args):
        logger.info(pc_id + " " + msg, *args)

    # Runs when the media transfer interface is initialized
    @pc.on("track")
    async def on_track(track):
        log_info("Track %s received", track.kind)
        print("Receiving %s", track.kind)

        # node.get_logger().info("Track added to the media sink")
        recorder.addTrack(RemoteVideoStreamTrack(track, node ))
        await recorder.start()

        # Runs when the interface is ended
        @track.on("ended")
        async def on_ended():
            log_info("Track %s ended", track.kind)

    await signaling.connect()

    await consume_signaling(pc, signaling )

def rtc_eventloop(args, node ):
    signaling = create_signaling(args)
    pc = RTCPeerConnection()
    recorder = MediaBlackhole()
    # recorder = MediaRecorder("/home/gulbasozan/video.mp4")
    coro = answer(
        pc,
        signaling, 
        recorder,
        node,
    )
    
    # Init event loop
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # Run event loop
    try:
        loop.run_until_complete(coro)
    except KeyboardInterrupt:
        pass
    finally:
        loop.run_until_complete(pc.close())
        loop.run_until_complete(signaling.close())
        loop.run_until_complete(recorder.stop())
        loop.close()
        
class RTCWebcamNode(Node):
    def __init__(self ):
        super().__init__("rtc_webcam")

        self.image_raw_pub_ = self.create_publisher(Image, "rto3/img_raw", 10)
        self.image_raw_msg_ = Image()

        self.bridge = CvBridge()

        self.get_logger().info("RTC Webcam Node initialized.")

    def pubCallback(self, img):
        self.get_logger().info("Publishing image to /rto3/img_raw topic..")
        self.image_raw_msg_ = self.bridge.cv2_to_imgmsg(img, "bgr8")
        try:
            self.image_raw_pub_.publish(self.image_raw_msg_)
            self.get_logger().info("Published image.")
        except CvBridgeError as error:
            print(error)
            self.get_logger().info("Couldn't publish the image.")

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    parser = argparse.ArgumentParser(description="Webcam stream")
    parser.add_argument("--verbose", "-v", action="count")
    add_signaling_arguments(parser)

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG) 
    
    rclpy.init(args=None)
    node = RTCWebcamNode()

    # rclpy.spin(node)

    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,))
    spin_thread.start()

    rtc_eventloop(args, node)

    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()


if __name__ == "__main__":
    main()


