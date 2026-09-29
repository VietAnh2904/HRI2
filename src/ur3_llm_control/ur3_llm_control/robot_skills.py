"""All arm motion uses MoveIt's MoveGroup action (plan + execute)."""
import math
import time
from copy import deepcopy
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState, Image
from std_srvs.srv import SetBool
from gazebo_msgs.srv import GetEntityState
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity, GetPositionIK, GetCartesianPath
from moveit_msgs.msg import (CollisionObject, AttachedCollisionObject, PlanningScene,
                            Constraints, JointConstraint, AllowedCollisionEntry, PlanningSceneComponents)
from shape_msgs.msg import SolidPrimitive

JOINTS = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']


class SkillFailure(RuntimeError):
    pass


class RobotSkills(Node):
    def __init__(self):
        super().__init__('skill_executor', parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.scene = yaml.safe_load((Path(get_package_share_directory('ur3_llm_control'))/'config/scene.yaml').read_text())
        self.joints = None
        self.image = None
        self.held = None
        self.motion_log = []
        self.create_subscription(JointState, '/joint_states', self._joints, qos_profile_sensor_data)
        self.create_subscription(Image, '/overview/overview/image_raw', self._image, qos_profile_sensor_data)
        self.entity = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.cart = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.grasp = {o:self.create_client(SetBool, '/grasp/'+o) for o in self.scene['objects']}

    def _joints(self, value): self.joints = value
    def _image(self, value): self.image = value

    @staticmethod
    def wait(future, timeout=30):
        end = time.monotonic()+timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        if not future.done():
            raise SkillFailure('TIMEOUT')
        return future.result()

    def call(self, client, request, timeout=30):
        if not client.wait_for_service(timeout_sec=10):
            raise SkillFailure('SERVICE_UNAVAILABLE: '+client.srv_name)
        return self.wait(client.call_async(request), timeout)

    def ready(self):
        if not self.move.wait_for_server(timeout_sec=60):
            raise SkillFailure('MOVEIT_UNAVAILABLE')
        end = time.monotonic()+30
        while self.joints is None and time.monotonic()<end:
            time.sleep(0.1)
        if self.joints is None:
            raise SkillFailure('NO_JOINT_STATE')
        req=GetPlanningScene.Request(); req.components.components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        attached=self.call(self.get_scene,req).scene.robot_state.attached_collision_objects
        if attached:
            self.held=attached[0].object.id
            raise SkillFailure('RECOVERY_REQUIRED: ROBOT_STILL_HOLDING_'+self.held)
        self.sync_scene()

    def pose(self, name):
        req = GetEntityState.Request(); req.name = name; req.reference_frame = 'world'
        result = self.call(self.entity, req)
        if not result.success:
            raise SkillFailure('INVALID_OBJECT: '+name)
        return result.state.pose

    def positions(self):
        return {o:[p.position.x,p.position.y,p.position.z]
                for o in self.scene['objects'] for p in [self.pose(o)]}

    def locations(self):
        positions = self.positions()
        return {o:next((z for z,t in self.scene['zones'].items()
                        if math.dist(xyz,t)<self.scene['zone_tolerance']),None)
                for o,xyz in positions.items()}

    @staticmethod
    def collision(name, xyz, size):
        obj = CollisionObject(); obj.id = name; obj.header.frame_id = 'world'; obj.operation = CollisionObject.ADD
        shape = SolidPrimitive(); shape.type = SolidPrimitive.BOX; shape.dimensions = list(size)
        pose = Pose(); pose.position.x,pose.position.y,pose.position.z = map(float, xyz); pose.orientation.w=1.0
        obj.primitives=[shape]; obj.primitive_poses=[pose]
        return obj

    def apply(self, scene):
        scene.is_diff = True
        req = ApplyPlanningScene.Request(); req.scene=scene
        if not self.call(self.apply_scene, req).success:
            raise SkillFailure('PLANNING_SCENE_FAILED')

    def sync_scene(self):
        s = PlanningScene(); table=self.scene['table']
        s.world.collision_objects = [self.collision('work_table', table['center'], table['size']),
                                     self.collision('floor',[0.,0.,-0.035],[3.,3.,0.05])]
        for o,xyz in self.positions().items():
            if o != self.held:
                s.world.collision_objects.append(self.collision(o, xyz, [self.scene['cube_size']]*3))
        self.apply(s)

    def allow_touch(self, obj, enabled, links=None):
        req=GetPlanningScene.Request(); req.components.components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        acm=self.call(self.get_scene,req).scene.allowed_collision_matrix
        names=[obj]+(links if links is not None else ['suction_tip','wrist_3_link','tool0','flange'])
        for name in names:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values: row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        i=acm.entry_names.index(obj)
        for link in names[1:]:
            j=acm.entry_names.index(link)
            acm.entry_values[i].enabled[j]=enabled; acm.entry_values[j].enabled[i]=enabled
        s=PlanningScene(); s.allowed_collision_matrix=acm; self.apply(s)

    def motion(self, target, label):
        goal=MoveGroup.Goal(); req=goal.request
        req.group_name='ur_manipulator'; req.num_planning_attempts=5; req.allowed_planning_time=8.0
        req.max_velocity_scaling_factor=0.18; req.max_acceleration_scaling_factor=0.15
        req.start_state.is_diff=True
        constraints=Constraints()
        if isinstance(target, list):
            constraints.joint_constraints=[JointConstraint(joint_name=n,position=float(v),tolerance_above=0.003,tolerance_below=0.003,weight=1.0) for n,v in zip(JOINTS,target)]
        else:
            ik_req=GetPositionIK.Request(); ik=ik_req.ik_request
            ik.group_name='ur_manipulator'; ik.ik_link_name='tool0'
            ik.pose_stamped.header.frame_id='world'; ik.pose_stamped.pose=target
            ik.robot_state.joint_state=deepcopy(self.joints); ik.robot_state.is_diff=True
            if label.endswith(':approach') and self.held is None:
                # Seed an elbow-up grasp posture. This is only an IK initial
                # guess; MoveIt still solves, bounds-checks and plans all motion.
                azimuth=math.atan2(target.position.y,target.position.x)-0.4
                seed=dict(zip(JOINTS,[azimuth,-1.5,1.5,-1.57,-1.57,azimuth-math.pi/2]))
                ik.robot_state.joint_state.position=[seed.get(n,v) for n,v in zip(
                    ik.robot_state.joint_state.name,ik.robot_state.joint_state.position)]
            ik.avoid_collisions=True; ik.timeout.sec=2
            solution=self.call(self.ik,ik_req)
            if solution.error_code.val != 1:
                raise SkillFailure('PLANNING_FAILED: NO_COLLISION_FREE_IK')
            angles=dict(zip(solution.solution.joint_state.name,solution.solution.joint_state.position))
            current=dict(zip(self.joints.name,self.joints.position))
            for n in JOINTS:
                options=[angles[n]+k*2*math.pi for k in (-1,0,1)]
                options=[q for q in options if -6.10<q<6.10]
                if options: angles[n]=min(options,key=lambda q:abs(q-current[n]))
            print('IK '+label+': '+str([round(angles[n],4) for n in JOINTS]),flush=True)
            constraints.joint_constraints=[JointConstraint(joint_name=n,position=angles[n],tolerance_above=0.001,tolerance_below=0.001,weight=1.0) for n in JOINTS]
        req.goal_constraints=[constraints]
        goal.planning_options.plan_only=False; goal.planning_options.replan=False
        started=time.monotonic()
        handle=self.wait(self.move.send_goal_async(goal),10)
        if not handle.accepted: raise SkillFailure('GOAL_REJECTED')
        try:
            result=self.wait(handle.get_result_async(),120).result
        except SkillFailure:
            self.wait(handle.cancel_goal_async(),10)
            raise
        code=result.error_code.val
        entry={'motion':label,'moveit_error_code':code,'seconds':round(time.monotonic()-started,3),
               'trajectory_points':len(result.planned_trajectory.joint_trajectory.points)}
        self.motion_log.append(entry)
        print('MOVEIT '+str(entry),flush=True)
        if code != 1: raise SkillFailure('PLANNING_FAILED' if code in (-1,-2,-10,-12,-31) else 'EXECUTION_FAILED')
        check=GetStateValidity.Request(); check.group_name='ur_manipulator'; check.robot_state.joint_state=deepcopy(self.joints)
        check.robot_state.is_diff=True
        state=self.call(self.validity,check)
        entry['state_valid_after']=state.valid
        if not state.valid:
            entry['contacts']=[f'{c.contact_body_1}/{c.contact_body_2}' for c in state.contacts]
            raise SkillFailure('STATE_INVALID_AFTER_MOTION')

    def target(self, xyz, above=False):
        p=Pose(); p.position.x,p.position.y=map(float,xyz[:2])
        p.position.z=float(xyz[2]+self.scene['cube_size']/2+self.scene['tool_offset']+0.006)
        if above: p.position.z+=self.scene['approach_height']
        p.orientation.x=1.0; p.orientation.w=0.0
        return p

    def cartesian(self, target, label):
        started=time.monotonic()
        req=GetCartesianPath.Request(); req.header.frame_id='world'
        req.group_name='ur_manipulator'; req.link_name='tool0'
        req.start_state.joint_state=deepcopy(self.joints); req.start_state.is_diff=True
        req.waypoints=[target]; req.max_step=0.005; req.jump_threshold=2.0; req.avoid_collisions=True
        req.max_velocity_scaling_factor=0.15; req.max_acceleration_scaling_factor=0.10
        path=self.call(self.cart,req)
        entry={'motion':label,'planner':'MoveIt Cartesian','fraction':path.fraction,
               'trajectory_points':len(path.solution.joint_trajectory.points)}
        self.motion_log.append(entry)
        if path.error_code.val!=1 or path.fraction<0.999:
            raise SkillFailure('PLANNING_FAILED: INCOMPLETE_CARTESIAN_PATH')
        # Recheck every timed sample after MoveIt's time parameterization.
        for point in path.solution.joint_trajectory.points:
            check=GetStateValidity.Request(); check.group_name='ur_manipulator'
            check.robot_state.joint_state.name=path.solution.joint_trajectory.joint_names
            check.robot_state.joint_state.position=point.positions; check.robot_state.is_diff=True
            if not self.call(self.validity,check).valid:
                raise SkillFailure('PLANNING_FAILED: CARTESIAN_POSTPROCESS_COLLISION')
        if not self.execute.wait_for_server(timeout_sec=10): raise SkillFailure('MOVEIT_UNAVAILABLE')
        goal=ExecuteTrajectory.Goal(); goal.trajectory=path.solution
        handle=self.wait(self.execute.send_goal_async(goal),10)
        if not handle.accepted: raise SkillFailure('GOAL_REJECTED')
        try: result=self.wait(handle.get_result_async(),90).result
        except SkillFailure:
            self.wait(handle.cancel_goal_async(),10); raise
        entry['moveit_error_code']=result.error_code.val
        entry['seconds']=round(time.monotonic()-started,3)
        check=GetStateValidity.Request(); check.group_name='ur_manipulator'
        check.robot_state.joint_state=deepcopy(self.joints); check.robot_state.is_diff=True
        entry['state_valid_after']=self.call(self.validity,check).valid
        print('MOVEIT '+str(entry),flush=True)
        if result.error_code.val!=1 or not entry['state_valid_after']:
            raise SkillFailure('EXECUTION_FAILED')

    def attach_scene(self, obj, attaching):
        s=PlanningScene(); s.robot_state.is_diff=True
        attached=AttachedCollisionObject(); attached.link_name='tool0'
        attached.touch_links=['tool0','flange','wrist_3_link','suction_tip']
        p=self.pose(obj)
        attached.object=self.collision(obj,[p.position.x,p.position.y,p.position.z],[self.scene['cube_size']]*3)
        attached.object.primitive_poses[0]=p
        # MoveIt moves the object from world to attached state automatically;
        # removing it a second time makes ApplyPlanningScene report failure.
        if not attaching:
            attached.object.operation=CollisionObject.REMOVE
        s.robot_state.attached_collision_objects=[attached]; self.apply(s)

    def switch(self, obj, value):
        req=SetBool.Request(); req.data=value
        response=self.call(self.grasp[obj],req)
        if not response.success: raise SkillFailure('GRASP_FAILED: '+response.message)

    def home(self):
        if self.held: raise SkillFailure('PRECONDITION_FAILED')
        self.motion(self.scene['home'],'home')

    def pick(self, obj):
        if obj not in self.scene['objects']: raise SkillFailure('INVALID_OBJECT')
        if self.held: raise SkillFailure('PRECONDITION_FAILED')
        self.sync_scene()
        p=self.pose(obj); xyz=[p.position.x,p.position.y,p.position.z]
        self.motion(self.target(xyz,True),obj+':approach')
        self.allow_touch(obj,True)
        self.cartesian(self.target(xyz),obj+':descend')
        self.switch(obj,True); self.held=obj; self.attach_scene(obj,True)
        # Contact between the cube and its supporting table is intentional at
        # lift-off only. Arm/table and all other collision checks stay enabled.
        self.allow_touch(obj,True,['work_table'])
        try: self.cartesian(self.target(xyz,True),obj+':lift')
        finally: self.allow_touch(obj,False,['work_table'])
        actual=self.pose(obj)
        if actual.position.z < xyz[2]+0.06: raise SkillFailure('OBJECT_NOT_LIFTED')

    def place(self, obj, zone):
        if obj != self.held: raise SkillFailure('PRECONDITION_FAILED')
        if zone not in self.scene['zones']: raise SkillFailure('INVALID_ZONE')
        if any(z==zone for o,z in self.locations().items() if o!=obj): raise SkillFailure('ZONE_OCCUPIED')
        xyz=self.scene['zones'][zone]
        self.motion(self.target(xyz,True),zone+':approach')
        # Release 3 mm above the table to avoid penetrations in the contact solver.
        release=self.target(xyz); release.position.z+=0.003
        self.cartesian(release,zone+':descend')
        self.switch(obj,False); self.attach_scene(obj,False); self.held=None
        time.sleep(0.6); self.sync_scene()
        self.cartesian(self.target(xyz,True),zone+':retreat')
        self.allow_touch(obj,False)
        actual=self.positions()[obj]
        if math.dist(actual,xyz)>self.scene['zone_tolerance']: raise SkillFailure('PLACE_VERIFICATION_FAILED')

    def save_image(self,path):
        from PIL import Image as PILImage
        end=time.monotonic()+10
        while self.image is None and time.monotonic()<end: time.sleep(0.1)
        msg=self.image
        if msg is None: return False
        if msg.encoding not in ('rgb8','bgr8'): return False
        PILImage.frombytes('RGB',(msg.width,msg.height),bytes(msg.data),'raw','RGB' if msg.encoding=='rgb8' else 'BGR',msg.step).save(path)
        return True
