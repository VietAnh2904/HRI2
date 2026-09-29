"""Generate the Gazebo scene from the same coordinates used by the skills."""
from pathlib import Path
import yaml

root = Path(__file__).resolve().parents[1] / 'src/ur3_llm_control'
scene = yaml.safe_load((root / 'config/scene.yaml').read_text())
def box(name, xyz, size, color, static=True, collide=True):
    geometry = '<geometry><box><size>' + ' '.join(map(str, size)) + '</size></box></geometry>'
    collision = f'<collision name="collision">{geometry}<surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision>' if collide else ''
    return f'''<model name="{name}"><static>{str(static).lower()}</static>
      <pose>{' '.join(map(str, xyz))} 0 0 0</pose><link name="link">
      <inertial><mass>0.05</mass><inertia><ixx>0.00001333</ixx><iyy>0.00001333</iyy><izz>0.00001333</izz></inertia></inertial>
      {collision}<visual name="visual">{geometry}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link></model>'''
parts = ['''<?xml version="1.0"?><sdf version="1.6"><world name="default">
  <gravity>0 0 -9.81</gravity>
  <physics name="physics" type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
  <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.85 0.89 0.93 1</background><shadows>true</shadows></scene>
  <light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 -0.3 -1</direction><cast_shadows>true</cast_shadows></light>
  <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><ros><namespace>/gazebo</namespace></ros><update_rate>20</update_rate></plugin>
  <gui><camera name="user_camera"><pose>1.15 -1.25 1.1 0 0.5 2.35</pose></camera></gui>''']
parts.append(box('floor', [0, 0, -0.035], [3, 3, 0.05], '0.65 0.68 0.70 1'))
parts.append(box('work_table', scene['table']['center'], scene['table']['size'], '0.57 0.42 0.27 1'))
colors = {'red_cube':'0.85 0.05 0.04 1', 'yellow_cube':'0.95 0.75 0.02 1', 'blue_cube':'0.03 0.20 0.88 1'}
for name, pos in scene['objects'].items():
    parts.append(box(name, pos, [scene['cube_size']]*3, colors[name], static=False))
for name, pos in scene['zones'].items():
    parts.append(box(name, [pos[0], pos[1], 0.1005], [0.08, 0.08, 0.001], '0.20 0.70 0.48 1', collide=False))
parts.append('''<model name="overview_camera"><static>true</static><pose>1.30 -1.40 1.30 0 0.45 2.25</pose><link name="link">
  <sensor name="overview" type="camera"><always_on>true</always_on><update_rate>6</update_rate>
    <camera><horizontal_fov>1.05</horizontal_fov><image><width>1280</width><height>720</height><format>R8G8B8</format></image><clip><near>0.05</near><far>10</far></clip></camera>
    <plugin name="overview_camera" filename="libgazebo_ros_camera.so"><ros><namespace>/overview</namespace></ros><camera_name>overview</camera_name><frame_name>world</frame_name></plugin>
  </sensor></link></model></world></sdf>''')
(root / 'worlds/table.world').write_text('\n'.join(parts))
