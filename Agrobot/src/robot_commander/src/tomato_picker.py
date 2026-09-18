#!/usr/bin/env python3
import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String, Bool

# tf2 imports for coordinate frame transform
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

#Beginning Date: 03/22/2026
#Updated: 4/26
#Completed: 4/27?
#Beginning Date: 03/22/2026

#pre-grasp position --> target point - offset 10 cm to the tomato
#grasp position --> actual target point of tomato
#safe retraction --> target point + some kind of offset 

#from kaedin's own words "you're going to get a list of JSON objects --> closest neighbor search / traveling salesmen problem & generate list of closest tomatoes
#--> for each tomato in list generate gripper pose,generate target interpolation points, & complete list--> pick tomato (emily's job) 
#--> move to drop tomato in basket--> go to safe position to pick next tomato"
#notes: moving on linear stage, think about optimizations
#Use TSP/greedy sort 
#what this code is NOT doing: planning the robot dynamics (that's Emily's job with moveit)
#ordered coordinate list is location of tomato centroid relative to AgroBot base

#----HOW THIS SCRIPT WORKS----
#Setup:when the node starts, it initializes a TF2 listener (for coordinate frame transforms), two subscribers, and one publisher
#It also keeps track of safe_to_pick state and a set of already-picked tomato IDs so it doesn't re-queue the same tomato across detection batches

#Safety gate — it listens to /agrobot/safe_to_pick. If that's False, any incoming detection batch is immediately ignored. 
#---> This is a persistent flag, NOT a one-shot check!!!!4

#Main picking logic — when a detection batch arrives on /agrobot/tomato_spatial, it runs through four steps:
        #1. Filters out tomatoes below the confidence threshold and ones already picked
        #2. Transforms each surviving centroid from the camera's coordinate frame into the robot arm's base frame using TF2, and drops any that fall outside the arm's reach
        #3. Sorts the remaining tomatoes greedily along the Y axis, which corresponds to the linear stage direction
        #4. For each tomato, computes three waypoints:
            #an approach point backed off from the tomato surface by radius + 5cm,
            #the grasp point at the centroid itself, 
            #and a retract point 7cm above +++++++++++++

#Output — all of those poses are packed into a single PoseArray and published to /agrobot/pick_target, where Emily's MoveIt node takes it for motion planning
#----------------------------------------------------------
CONFIDENCE_THRESHOLD = 0.6   # ignore low-confidence detections
APPROACH_STANDOFF    = 0.05  # 5 cm before tomato surface on approach
RETRACT_STANDOFF     = 0.15  # 15 cm back along approach vector after grasp

#------------------------------------------
class TomatoPicker(Node):
    def __init__(self):
        super().__init__('tomato_picker')

        # --- TF2 setup for camera -> base frame transform ---
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Safety states ---
        self.safe_to_pick = True
        self.picked_ids   = set()

        # --- Subscribers ---
        self.create_subscription(
            Bool,
            '/agrobot/safe_to_pick',
            self.safety_callback,
            10
        )
        self.create_subscription(
            String,
            '/agrobot/tomato_spatial',
            self.spatial_callback,
            10
        )

        # --- Publisher ---
        # PoseArray to MoveIt; each tomato contributes 3 poses:
        # [approach, grasp, retract] then a basket drop pose
        self.publisher_ = self.create_publisher(PoseArray, '/agrobot/pick_targets', 10)

        self.get_logger().info('TomatoPicker ready, waiting for detections...')

    # ------------------------------------------------------------------
    # Safety gate
    # ------------------------------------------------------------------
    def safety_callback(self, msg: Bool):
        self.safe_to_pick = msg.data
        if not self.safe_to_pick:
            self.get_logger().warn('safe_to_pick is False — picking paused')

    # ------------------------------------------------------------------
    # Main callback: fires every time Node 2 publishes a detection batch
    # ------------------------------------------------------------------
    def spatial_callback(self, msg: String):
        if not self.safe_to_pick:
            self.get_logger().warn('Received detections but safe_to_pick = False, skipping')
            return

        try:
            tomatoes = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse tomato_spatial JSON: {e}')
            return

        # 1. Filter: confidence threshold + already picked
        candidates = [
            t for t in tomatoes
            if t['confidence'] >= CONFIDENCE_THRESHOLD
            and t['tomato_id'] not in self.picked_ids
        ]

        if not candidates:
            self.get_logger().info('No new pickable tomatoes in this batch')
            return

        # 2. Transform centroids from camera frame -> robot base frame via TF2
        base_frame_candidates = []
        for t in candidates:
            base_pos = self.transform_to_base(t['centroid'])
            if base_pos is None:
                continue  # TF lookup failed for this tomato, skip it
            if not self.is_reachable(base_pos):
                self.get_logger().info(
                    f'Tomato {t["tomato_id"]} out of reach at '
                    f'({base_pos["x"]:.3f}, {base_pos["y"]:.3f}, {base_pos["z"]:.3f}), skipping'
                )
                continue
            base_frame_candidates.append({
                'id':     t['tomato_id'],
                'pos':    base_pos,
                'radius': t['sphere']['radius'],
            })

        if not base_frame_candidates:
            return

        # 3. Greedy sort along Y 
        base_frame_candidates.sort(key=lambda t: t['pos']['y'])

        # 4. Build PoseArray
        pick_msg = PoseArray()
        pick_msg.header.frame_id  = 'linear_rail_link'
        pick_msg.header.stamp     = self.get_clock().now().to_msg()

        for t in base_frame_candidates:
            approach_vec = self.approach_vector(t['pos'])

            waypoints = self.get_waypoints(t['pos'], approach_vec, t['radius'])
            quat      = self.get_quaternion(approach_vec)

            for wp in waypoints:
                pose = Pose()
                pose.position.x    = wp['x']
                pose.position.y    = wp['y']
                pose.position.z    = wp['z']
                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]
                pick_msg.poses.append(pose)

            self.picked_ids.add(t['id'])
            self.get_logger().info(f'Queued tomato {t["id"]} for picking')

        self.publisher_.publish(pick_msg)
        self.get_logger().info(
            f'Published {len(base_frame_candidates)} tomatoes to /agrobot/pick_targets'
        )

    # ------------------------------------------------------------------
    # Coordinate transform: camera frame -> robot base frame via TF2
    # ------------------------------------------------------------------
    def transform_to_base(self, centroid: dict) -> dict | None:
        point = PointStamped()
        point.header.frame_id = 'camera_color_optical_frame'  # verify against your URDF
        point.header.stamp    = self.get_clock().now().to_msg()
        point.point.x = centroid['x']
        point.point.y = centroid['y']
        point.point.z = centroid['z']

        try:
            transformed = self.tf_buffer.transform(
                point,
                'linear_rail_link',
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return {
                'x': transformed.point.x,
                'y': transformed.point.y,  # flip Y axis if needed based on TF results
                'z': transformed.point.z,
            }
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF2 transform failed: {e}')
            return None

    # ------------------------------------------------------------------
    # Reachability check (flat 3D distance from base origin)
    # ------------------------------------------------------------------
    MAX_REACH = 1.5  # meters, adjust based on the robot's actual reach

    def is_reachable(self, p: dict) -> bool:
        dist = math.sqrt(p['x']**2 + p['y']**2 + p['z']**2)
        return dist <= self.MAX_REACH

    # ------------------------------------------------------------------
    # Approach vector: arm base origin -> tomato centroid (unit vector)
    # ------------------------------------------------------------------
    def approach_vector(self, base_pos: dict) -> dict:
        mag = math.sqrt(base_pos['x']**2 + base_pos['y']**2 + base_pos['z']**2)
        return {
            'dx': base_pos['x'] / mag,
            'dy': base_pos['y'] / mag,
            'dz': base_pos['z'] / mag,
        }

    # ------------------------------------------------------------------
    # Waypoints: approach surface, grasp centroid, retract upward
    # Approach is offset by (radius + appoach standoff) along approach vector
    # so it scales with actual tomato size
    # Retract is offset by (radius + retract standoff) along the retract vector
    # ------------------------------------------------------------------
    def get_waypoints(self, centroid: dict, vec: dict, radius: float) -> list:

        approach = {
            'x': centroid['x'] - vec['dx'] * (radius + APPROACH_STANDOFF),
            'y': centroid['y'] - vec['dy'] * (radius + APPROACH_STANDOFF),
            'z': centroid['z'] - vec['dz'] * (radius + APPROACH_STANDOFF),
        }
        retract = {
            'x': centroid['x'] - vec['dx'] * (radius + RETRACT_STANDOFF),
            'y': centroid['y'] - vec['dy'] * (radius + RETRACT_STANDOFF),
            'z': centroid['z'] - vec['dz'] * (radius + RETRACT_STANDOFF),
        }
        return [approach, centroid, retract]

    # ------------------------------------------------------------------
    # Gripper orientation: rotate gripper's +Z axis to face approach vec
    # ------------------------------------------------------------------
    def get_quaternion(self, vec: dict) -> np.ndarray:
        target  = np.array([vec['dx'], vec['dy'], vec['dz']])
        current = np.array([0.0, 0.0, 1.0])  # gripper forward axis — verify in URDF
        rotation, _ = R.align_vectors([target], [current])
        return rotation.as_quat()  # [x, y, z, w]


# ----------------------------------------------------------------------

def main():
    rclpy.init()
    node = TomatoPicker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

