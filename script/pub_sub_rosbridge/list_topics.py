import roslibpy

client = roslibpy.Ros(host='192.168.0.168', port=9090)
client.run()

print("Connected. Getting topics...")
topics = client.get_topics()
print("Topics found:")
for t in topics:
    print(t)

client.terminate()
