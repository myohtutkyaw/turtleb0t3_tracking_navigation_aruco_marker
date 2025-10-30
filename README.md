# turtleb0t3_tracking_navigation_aruco_marker

| Phase             | Description                                                                                                                                       |
| ----------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| **A**             | Follows a yellow line, detects a QR code saying “STOP”, performs a 90° turn (via odometry or timed fallback), and waits 1 second.                 |
| **B1 (optional)** | Waits for SLAM Toolbox `/map` topic updates (lifecycle-aware if enabled).                                                                         |
| **B**             | Detects a **front-facing ArUco marker**, retrieves its corresponding goal (x, y, yaw) from YAML parameters, and sends a **Nav2 navigation goal**. |
| **C**             | Uses odometry and vision to **dock precisely 10 cm** from the same ArUco marker.                                                                  |
