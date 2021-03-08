import yaml

GYM_DICT={
    'car':{
    'car_name': 'TritonRacer',
    'font_size': 50,
    'racer_name': 'Triton AI',
    'bio': 'Something',
    'country': 'US',
    'body_style': 'car01',
    'body_rgb': [24, 43, 73],
    'guid': 'some_random_string'},

  'default_connection': 'local', # Which is the default connection profile? "local" or "remote"?
  # default_connection: 'remote'

  'local_connection':{
    'scene_name': 'generated_track', # roboracingleague_1 | generated_track | generated_road | warehouse | sparkfun_avc | waveshare
    'host': '127.0.0.1', # Use "127.0.0.1" for simulator running on local host.
    'port': 9091,
    'artificial_latency': 0}, # Ping the remote simulator whose latency you would like to match with, and put the ping in millisecond here.

  'remote_connection':{
    'scene_name': 'generated_track',
    'host': '127.0.0.1', # Use the actual host name for remote simulator.
    'port': 9091,
    'artificial_latency': 0}, # Besides the ping to the remote simulator, how many MORE delay would you like to add?

  'lidar':{
    'enabled': False,
    'deg_inc': 2, # Degree increment between each ray of the lidar
    'max_range': 50.0}, # Max range of the lidar laser
}

with open('src\\donkey_gym_wrapper\\src\\config.yaml', 'w') as outfile:
    yaml.dump(GYM_DICT, outfile, default_flow_style=False)