#!/usr/bin/env python3
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import SequentialReader, SequentialWriter, StorageOptions, ConverterOptions
from rosbag2_py._storage import TopicMetadata
from sensor_msgs.msg import CompressedImage, Image
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
import av
from cv_bridge import CvBridge
import sys

def decode_bag(input_bag_path, output_bag_path):
    """Decode all compressed and FFMPEG image topics in the MCAP bag"""
    
    # Setup reader to discover image topics
    reader = SequentialReader()
    reader.open(StorageOptions(uri=input_bag_path, storage_id='mcap'), ConverterOptions('', ''))
    
    # Find all image topics that need decoding
    topic_metadata_list = reader.get_all_topics_and_types()
    compressed_topics = []
    ffmpeg_topics = []
    
    for topic_metadata in topic_metadata_list:
        if topic_metadata.type == 'sensor_msgs/msg/CompressedImage':
            compressed_topics.append(topic_metadata.name)
        elif topic_metadata.type == 'ffmpeg_image_transport_msgs/msg/FFMPEGPacket':
            ffmpeg_topics.append(topic_metadata.name)
    
    reader = None  # Close the reader
    
    if not compressed_topics and not ffmpeg_topics:
        print("No compressed or FFMPEG image topics found in the bag.")
        return
    
    print(f"Found compressed topics: {compressed_topics}")
    print(f"Found FFMPEG topics: {ffmpeg_topics}")
    
    # Setup readers and writers for processing
    reader = SequentialReader()
    reader.open(StorageOptions(uri=input_bag_path, storage_id='mcap'), ConverterOptions('', ''))

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=output_bag_path, storage_id='mcap'), ConverterOptions('', ''))

    # Get all topic metadata
    topic_metadata_list = reader.get_all_topics_and_types()

    # Create all original topics in the new bag
    for topic_metadata in topic_metadata_list:
        writer.create_topic(topic_metadata)
    
    # Create decompressed topics
    output_topics = {}
    
    # For compressed image topics
    for compressed_topic in compressed_topics:
        if '/compressed' in compressed_topic:
            output_topic = compressed_topic.replace('/compressed', '')
        else:
            output_topic = compressed_topic + '_decompressed'
        
        output_topics[compressed_topic] = output_topic
        
        output_topic_metadata = TopicMetadata(
            name=output_topic,
            type='sensor_msgs/msg/Image',
            serialization_format='cdr'
        )
        writer.create_topic(output_topic_metadata)
    
    # For FFMPEG topics  
    for ffmpeg_topic in ffmpeg_topics:
        if '/ffmpeg' in ffmpeg_topic:
            output_topic = ffmpeg_topic.replace('/ffmpeg', '')
        else:
            output_topic = ffmpeg_topic + '_decoded'
            
        output_topics[ffmpeg_topic] = output_topic
        
        output_topic_metadata = TopicMetadata(
            name=output_topic,
            type='sensor_msgs/msg/Image',
            serialization_format='cdr'
        )
        writer.create_topic(output_topic_metadata)

    # Initialize H.264 decoder with NVDEC hardware acceleration
    h264_decoder = av.CodecContext.create('h264', 'r')
    try:
        h264_decoder.options = {'hwaccel': 'cuda'}
        print("Using CUDA hardware acceleration for H.264 decoding")
    except:
        print("CUDA acceleration not available, using software decoding")
    
    # Dictionary to store decoders for FFMPEG streams
    ffmpeg_decoders = {}
    
    bridge = CvBridge()
    
    print(f"Starting conversion from {input_bag_path} to {output_bag_path}...")
    frame_counts = {topic: 0 for topic in (compressed_topics + ffmpeg_topics)}
    error_counts = {topic: 0 for topic in (compressed_topics + ffmpeg_topics)}

    while reader.has_next():
        (topic, data, t) = reader.read_next()

        # Always write the original message
        writer.write(topic, data, t)

        # Process compressed image topics (H.264)
        if topic in compressed_topics:
            output_topic = output_topics[topic]
            
            try:
                msg = deserialize_message(data, CompressedImage)
                
                # Decode H.264 compressed data
                packet = av.Packet(msg.data)
                frames = h264_decoder.decode(packet)
                
                for frame in frames:
                    cpu_frame = frame.to_cpu() if hasattr(frame, 'to_cpu') else frame
                    img_np = cpu_frame.to_ndarray(format='bgr24')
                    
                    img_msg = bridge.cv2_to_imgmsg(img_np, encoding='bgr8')
                    img_msg.header = msg.header
                    
                    serialized_msg = serialize_message(img_msg)
                    writer.write(output_topic, serialized_msg, t)
                    frame_counts[topic] += 1

            except Exception as e:
                error_counts[topic] += 1
                if error_counts[topic] <= 5:  # Only print first 5 errors per topic
                    print(f"Error decoding compressed frame on topic {topic} at time {t}: {e}")

        # Process FFMPEG packet topics (H.264)
        elif topic in ffmpeg_topics:
            output_topic = output_topics[topic]
            
            try:
                ffmpeg_msg = deserialize_message(data, FFMPEGPacket)
                
                # Create decoder for this stream if not exists
                if topic not in ffmpeg_decoders:
                    ffmpeg_decoders[topic] = av.CodecContext.create('h264', 'r')
                    try:
                        ffmpeg_decoders[topic].options = {'hwaccel': 'cuda'}
                    except:
                        pass  # Fallback to software decoding
                
                decoder = ffmpeg_decoders[topic]
                
                # Create packet from FFMPEG message data
                packet = av.Packet(ffmpeg_msg.data)
                if hasattr(ffmpeg_msg, 'pts'):
                    packet.pts = ffmpeg_msg.pts
                if hasattr(ffmpeg_msg, 'dts'):
                    packet.dts = ffmpeg_msg.dts
                
                # Decode the packet
                frames = decoder.decode(packet)
                
                for frame in frames:
                    cpu_frame = frame.to_cpu() if hasattr(frame, 'to_cpu') else frame
                    img_np = cpu_frame.to_ndarray(format='bgr24')
                    
                    # Create ROS Image message
                    img_msg = bridge.cv2_to_imgmsg(img_np, encoding='bgr8')
                    
                    # Use timestamp from FFMPEG packet or original message
                    img_msg.header.stamp.sec = int(t // 1000000000)
                    img_msg.header.stamp.nanosec = int(t % 1000000000)
                    img_msg.header.frame_id = ffmpeg_msg.header.frame_id if hasattr(ffmpeg_msg, 'header') else ''
                    
                    serialized_msg = serialize_message(img_msg)
                    writer.write(output_topic, serialized_msg, t)
                    frame_counts[topic] += 1

            except Exception as e:
                error_counts[topic] += 1
                if error_counts[topic] <= 5:  # Only print first 5 errors per topic
                    print(f"Error decoding FFMPEG frame on topic {topic} at time {t}: {e}")

    print("Flushing decoders...")
    for topic, decoder in ffmpeg_decoders.items():
        try:
            frames = decoder.decode(None)
            output_topic = output_topics[topic]
            
            for frame in frames:
                cpu_frame = frame.to_cpu() if hasattr(frame, 'to_cpu') else frame
                img_np = cpu_frame.to_ndarray(format='bgr24')
                
                img_msg = bridge.cv2_to_imgmsg(img_np, encoding='bgr8')
                img_msg.header.stamp.sec = 0
                img_msg.header.stamp.nanosec = 0
                
                serialized_msg = serialize_message(img_msg)
                writer.write(output_topic, serialized_msg, t)
                frame_counts[topic] += 1
        except:
            pass  # Ignore
    
    try:
        frames = h264_decoder.decode(None)
        for frame in frames:
            pass  # Ignore
    except:
        pass

    total_frames = sum(frame_counts.values())
    total_errors = sum(error_counts.values())
    
    print(f"\nConversion complete. Processed {total_frames} total frames with {total_errors} total errors:")
    
    for topic in compressed_topics + ffmpeg_topics:
        frames = frame_counts[topic]
        errors = error_counts[topic]
        success_rate = (frames / (frames + errors)) * 100 if (frames + errors) > 0 else 0
        print(f"  {topic}: {frames} frames, {errors} errors ({success_rate:.1f}% success)")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python3 decode_image_stream.py <input_bag.mcap> <output_bag.mcap>")
        print("\nThis script will automatically detect and decode all compressed and FFMPEG image topics:")
        print("  - sensor_msgs/msg/CompressedImage (H.264)")
        print("  - ffmpeg_image_transport_msgs/msg/FFMPEGPacket (H.264)")
        print("\nExample:")
        print("  python3 decode_image_stream.py test1.mcap test1_decoded")
        sys.exit(1)
    
    rclpy.init()
    
    try:
        decode_bag(sys.argv[1], sys.argv[2])
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()