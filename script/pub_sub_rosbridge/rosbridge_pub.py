"""
python3 ws_pub.py --host SERVER_IP --topic /chatter --rate 5 --msg hi

python3 ws_pub.py --host SERVER_IP --joint --topic /joint_states --names j1,j2 --rate 10
"""


#!/usr/bin/env python3
import argparse, time, math
import roslibpy

def main():
    ap = argparse.ArgumentParser(description='Publish via rosbridge (String or JointState)')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--topic', default='/chatter')
    ap.add_argument('--rate', type=float, default=5.0)
    ap.add_argument('--msg', default='hello via rosbridge')
    ap.add_argument('--joint', action='store_true', help='publish JointState instead of String')
    ap.add_argument('--names', default='j1,j2', help='joint names when --joint')
    args = ap.parse_args()

    ros = roslibpy.Ros(host=args.host, port=args.port); ros.run()
    assert ros.is_connected, 'cannot connect to rosbridge'
    print(f'[INFO] connected to ws://{args.host}:{args.port}')

    period = 1.0 / max(args.rate, 0.001)

    if args.joint:
        topic = roslibpy.Topic(ros, args.topic, 'sensor_msgs/JointState')
        names = [n.strip() for n in args.names.split(',') if n.strip()]
        t0 = time.time()
        i = 0
        try:
            while ros.is_connected:
                t = time.time() - t0
                pos = [math.sin(t + k) for k in range(len(names))] or [math.sin(t)]
                msg = {'name': names, 'position': pos, 'velocity': [], 'effort': []}
                topic.publish(roslibpy.Message(msg))
                print(f'sent joints {i}:', list(zip(names, pos)))
                i += 1; time.sleep(period)
        except KeyboardInterrupt:
            pass
        finally:
            topic.unadvertise(); ros.terminate()
    else:
        topic = roslibpy.Topic(ros, args.topic, 'std_msgs/String')
        i = 0
        try:
            while ros.is_connected:
                payload = f'{args.msg} #{i}'
                topic.publish(roslibpy.Message({'data': payload}))
                print('sent:', payload)
                i += 1; time.sleep(period)
        except KeyboardInterrupt:
            pass
        finally:
            topic.unadvertise(); ros.terminate()

if __name__ == '__main__':
    main()
