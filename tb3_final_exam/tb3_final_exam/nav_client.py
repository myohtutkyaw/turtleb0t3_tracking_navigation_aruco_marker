import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import math

def q_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2.0), w=math.cos(yaw/2.0))

class NavClient:
    def __init__(self, node: Node):
        self.node = node
        self.client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

    async def goto_xyyaw(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = q_from_yaw(yaw)
        await self.client.wait_for_server()
        send = await self.client.send_goal_async(goal)
        result = await send.get_result_async()
        return result.status
