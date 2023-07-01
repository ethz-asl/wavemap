#!/usr/bin/env python3

# NOTE: This script is an extension of the following script by the
#       Systems, Robotics and Vision Group from the University of the Balearican Islands:
#       https://github.com/srv/srv_tools/blob/melodic/bag_tools/scripts/remove_tf.py

import argparse
import traceback
import rospy
import rosbag


def extract_tf(input_bags, output_bag, frame_ids):
    rospy.loginfo('Processing input bags: %s', ' '.join(input_bags))
    rospy.loginfo('Writing to output bag: %s', output_bag)
    rospy.loginfo('Including frame_ids: %s', ' '.join(frame_ids))

    output_bag = rosbag.Bag(output_bag, 'w')
    for input_bag in input_bags:
        for topic, msg, t in rosbag.Bag(input_bag, 'r').read_messages():
            if topic == "/tf":
                new_transforms = []
                for transform in msg.transforms:
                    if transform.header.frame_id in frame_ids \
                            and transform.child_frame_id in frame_ids:
                        new_transforms.append(transform)
                msg.transforms = new_transforms
                output_bag.write(topic, msg, t)
    rospy.loginfo('Closing output bag and exit...')
    output_bag.close()


if __name__ == "__main__":
    rospy.init_node('extract_tf')
    parser = argparse.ArgumentParser(
        description=
        'Copy the TFs whose parent or child frame ID is included in frame_id(s) '
        'from the input_bag(s) to the output_bag.')
    parser.add_argument('-i',
                        metavar='INPUT_BAG',
                        required=True,
                        help='path(s) input bag file(s)',
                        nargs='+')
    parser.add_argument('-o',
                        metavar='OUTPUT_BAG',
                        required=True,
                        help='path to output bag file')
    parser.add_argument(
        '-f',
        metavar='FRAME_ID',
        required=True,
        help='frame_id(s) of the transforms to include from the /tf topic',
        nargs='+')
    args = parser.parse_args()

    try:
        extract_tf(args.i, args.o, args.f)
    except Exception as e:  # pylint: disable=broad-except
        traceback.print_exc()
