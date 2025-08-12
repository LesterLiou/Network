"""
python3 rosbridge_pub_joint.py --host SERVER_IP \
  --topic /left_follower/joint_states_control \
  --names left_follower_shoulder_pan --rate 50
"""
#!/usr/bin/env python3
import argparse, time, math
import roslibpy

def main():
    ap = argparse.ArgumentParser(description='Publish JointState via rosbridge')
    ap.add_argument('--host', default='127.0.0.1')     # SERVER_IP
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--topic', default='/left_follower/joint_states_control')
    ap.add_argument('--names', default='joint1,joint2')   # 依你的機器人關節名
    ap.add_argument('--rate', type=float, default=20.0)   # Hz，命令常見 20~100Hz
    args = ap.parse_args()

    ros = roslibpy.Ros(host=args.host, port=args.port); ros.run()
    assert ros.is_connected, 'cannot connect to rosbridge'
    print(f'[INFO] connected to ws://{args.host}:{args.port}')

    pub = roslibpy.Topic(ros, args.topic, 'sensor_msgs/JointState')
    names = [n.strip() for n in args.names.split(',') if n.strip()]
    t0 = time.time()
    period = 1.0 / max(args.rate, 0.001)
    i = 0
    try:
        while ros.is_connected:
            t = time.time() - t0
            # 測試：給一段正弦命令（實際請改成你的控制輸出）
            pos = [math.sin(t + k*0.3) for k in range(len(names))]
            msg = {'name': names, 'position': pos, 'velocity': [], 'effort': []}
            pub.publish(roslibpy.Message(msg))
            if i % int(max(args.rate/2.0,1)) == 0:
                print('sent:', list(zip(names, pos)))
            i += 1
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        pub.unadvertise(); ros.terminate()

if __name__ == '__main__':
    main()
