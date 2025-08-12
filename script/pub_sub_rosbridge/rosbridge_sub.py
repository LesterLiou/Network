"""
python3 ws_sub.py --host SERVER_IP --topic /chatter --type std_msgs/String

python3 ws_sub.py --host SERVER_IP --topic /joint_states --type sensor_msgs/JointState

"""



#!/usr/bin/env python3
import argparse, base64, os, time
import roslibpy

def parse_args():
    ap = argparse.ArgumentParser(description='Subscribe via rosbridge')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--topic', default='/chatter')
    ap.add_argument('--type', default='std_msgs/String',
                    help='std_msgs/String | sensor_msgs/JointState | sensor_msgs/CompressedImage')
    ap.add_argument('--save-dir', help='if type=CompressedImage, save frames here')
    return ap.parse_args()

def main():
    args = parse_args()
    ros = roslibpy.Ros(host=args.host, port=args.port); ros.run()
    assert ros.is_connected, 'cannot connect to rosbridge'
    print(f'[INFO] connected to ws://{args.host}:{args.port}')

    if args.type == 'sensor_msgs/CompressedImage' and args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    sub = roslibpy.Topic(ros, args.topic, args.type)

    def cb(msg):
        if args.type == 'std_msgs/String':
            print('recv:', msg.get('data'))
        elif args.type == 'sensor_msgs/JointState':
            names = msg.get('name', []); pos = msg.get('position', [])
            print('recv joints:', list(zip(names, pos)))
        elif args.type == 'sensor_msgs/CompressedImage':
            raw = base64.b64decode(msg['data'])
            print('recv frame bytes:', len(raw))
            if args.save_dir:
                out = os.path.join(args.save_dir, f'frame_{int(time.time()*1000)}.jpg')
                with open(out, 'wb') as f: f.write(raw)
        else:
            print('recv:', msg)

    sub.subscribe(cb)
    try:
        ros.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        sub.unsubscribe(cb); ros.terminate()

if __name__ == '__main__':
    main()
