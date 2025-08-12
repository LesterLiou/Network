"""
python3 rosbridge_pub_joint.py --host SERVER_IP --topic /left_follower/joint_states --rate 50
"""
#!/usr/bin/env python3
import argparse, time, math
import roslibpy

def main():
    ap = argparse.ArgumentParser(description='Publish all JointStates via rosbridge')
    ap.add_argument('--host', default='127.0.0.1')     # SERVER_IP
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--topic', default='/left_follower/joint_states')
    ap.add_argument('--rate', type=float, default=50.0)   # Hz
    args = ap.parse_args()

    # 這裡改成你的實際關節名稱
    joint_names = [
        'left_follower_shoulder_pan',
        'left_follower_shoulder_lift',
        'left_follower_elbow_flex',
        'left_follower_wrist_flex',
        'left_follower_wrist_roll',
        'left_follower_gripper'
    ]

    ros = roslibpy.Ros(host=args.host, port=args.port)
    ros.run()
    assert ros.is_connected, '❌ Cannot connect to rosbridge'
    print(f'[INFO] Connected to ws://{args.host}:{args.port}')

    pub = roslibpy.Topic(ros, args.topic, 'sensor_msgs/JointState')
    t0 = time.time()
    period = 1.0 / max(args.rate, 0.001)
    i = 0

    try:
        while ros.is_connected:
            t = time.time() - t0
            # 測試: 每個關節輸出一段不同相位的正弦值
            positions = [math.sin(t + k * 0.3) for k in range(len(joint_names))]
            msg = {
                'name': joint_names,
                'position': positions,
                'velocity': [],
                'effort': []
            }
            pub.publish(roslibpy.Message(msg))
            if i % int(max(args.rate / 2.0, 1)) == 0:
                print(f'[PUB] step={i} positions={positions}')
            i += 1
            time.sleep(period)

    except KeyboardInterrupt:
        pass
    finally:
        pub.unadvertise()
        ros.terminate()

if __name__ == '__main__':
    main()
