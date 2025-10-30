from geometry_msgs.msg import Twist

class Docker:
    def __init__(self, node):
        self.node = node
        self.cmd = node.create_publisher(Twist, '/cmd_vel', 10)

    def approach(self, distance_m, heading_err=0.0):
        """Simple PD: stop at 0.10 m"""
        t = Twist()
        # linear scaling
        t.linear.x = max(min((distance_m - 0.10) * 0.8, 0.18), -0.05)
        t.angular.z = max(min(-heading_err * 1.5, 1.0), -1.0)
        self.cmd.publish(t)

    def stop(self):
        self.cmd.publish(Twist())
