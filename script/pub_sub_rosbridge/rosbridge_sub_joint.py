"""
python3 rosbridge_sub_joint.py --host SERVER_IP --topic /left_follower/joint_states

"""
#!/usr/bin/env python3
import argparse, roslibpy

def main():
    ap = argparse.ArgumentParser(description='Subscribe JointState via rosbridge')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--topic', default='/left_follower/joint_states')
    args = ap.parse_args()

    ros = roslibpy.Ros(host=args.host, port=args.port); ros.run()
    assert ros.is_connected, 'cannot connect to rosbridge'
    print(f'[INFO] connected to ws://{args.host}:{args.port}')

    sub = roslibpy.Topic(ros, args.topic, 'sensor_msgs/JointState')
    def cb(m):
        names = m.get('name', []); pos = m.get('position', [])
        print('recv joints:', list(zip(names, pos)))
    sub.subscribe(cb)

    try:
        ros.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        sub.unsubscribe(cb); ros.terminate()

if __name__ == '__main__':
    main()
