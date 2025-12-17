---
id: book-06-algorithms-04-robot-kinematics-forward-and-inverse
title: Function for Inverse Kinematics of a 2-DOF planar arm
sidebar_position: 4
---

--- 
sidebar_position: 4
title: Robot Kinematics (Forward and Inverse)
---

## 04-Robot Kinematics (Forward and Inverse)

**Kinematics** is the study of motion without considering the forces that cause it. In robotics, kinematics is crucial for understanding and controlling the movement of robotic manipulators (arms) and mobile robots. It answers questions about where a robot's parts are in space given its joint positions, or what joint positions are needed to reach a desired target. This chapter focuses on two main branches: Forward Kinematics and Inverse Kinematics.

### 4.1 Coordinate Systems

Before diving into kinematics, it's essential to understand the coordinate systems used:
* **World Frame:** A fixed reference frame defining the robot's environment.
* **Base Frame:** A frame fixed to the robot's base.
* **Joint Frames:** A coordinate frame associated with each joint, simplifying the description of links.
* **End-Effector Frame:** A frame fixed to the robot's end-effector (gripper, tool).

Transformations (translation and rotation) are used to relate these frames to each other.

### 4.2 Forward Kinematics (FK)

**Forward Kinematics** is the process of calculating the position and orientation of the robot's end-effector (or any point on the robot) in the world frame, given the known lengths of its links and the angles/displacements of all its joints.

#### 4.2.1 Principle

For a serial manipulator (robot arm):
1.  Start at the base of the robot.
2.  Apply a transformation (rotation and translation) for the first joint and link.
3.  Then, apply the transformation for the second joint and link, relative to the first.
4.  Continue this process until the end-effector.

Each joint-link pair can be represented by a **homogeneous transformation matrix** that combines rotation and translation. Multiplying these matrices sequentially from the base to the end-effector yields the final pose of the end-effector relative to the base.

#### 4.2.2 Denavit-Hartenberg (DH) Parameters

The **Denavit-Hartenberg (DH) convention** is a widely used method to define a coordinate system for each joint in a serial kinematic chain, simplifying the process of deriving the homogeneous transformation matrices and thus the forward kinematics equations. Each transformation between two consecutive link frames is described by four parameters:
*   `a`: Link length
*   `α` (alpha): Link twist
*   `d`: Link offset
*   `θ` (theta): Joint angle

#### 4.2.3 Application

*   Simulating robot movement and visualizing its reach.
*   Determining if a given joint configuration results in a collision.
*   Calculating sensor positions on the end-effector.

**Diagram 4.1: Two-Link Robot Arm (Forward Kinematics)**

```mermaid
graph TD
    A[Base Frame] --> J1(Joint 1: Theta1)
J1 --> L1(Link 1: Length L1)
L1 --> J2(Joint 2: Theta2)
J2 --> L2(Link 2: Length L2)
L2 --> EE[End-Effector Position (x,y)]
```

*Description: A simple 2-link robot arm, illustrating how knowing joint angles (Theta1, Theta2) and link lengths (L1, L2) allows calculation of the end-effector's position.*

### 4.3 Inverse Kinematics (IK)

**Inverse Kinematics** is the process of calculating the required joint angles or displacements of a robot manipulator to achieve a desired position and orientation of its end-effector in space.

#### 4.3.1 Principle

This is generally a much harder problem than forward kinematics because:
*   **Multiple Solutions:** For a given end-effector pose, there might be multiple possible joint configurations (e.g., "elbow up" vs. "elbow down").
*   **No Solution:** The desired pose might be outside the robot's workspace.
*   **Singularities:** Certain joint configurations can lead to a loss of degrees of freedom.

#### 4.3.2 Methods for Solving IK

1.  **Analytical Solutions:**
    *   **Principle:** Involves solving a set of algebraic (trigonometric) equations derived from the FK equations.
    *   **Advantages:** Fast, precise, guaranteed to find all solutions.
    *   **Disadvantages:** Only possible for robots with a simple kinematic structure (e.g., 2-3 DOF planar arms, 6-DOF PUMA-type industrial arms). Not feasible for complex or redundant manipulators.
2.  **Numerical/Iterative Solutions:**
    *   **Principle:** Start with an initial guess for joint angles and iteratively adjust them to minimize the error between the current end-effector pose and the desired pose. Uses techniques like Jacobian-based methods (e.g., Newton-Raphson, pseudo-inverse Jacobian).
    *   **Advantages:** Can solve IK for any kinematic structure, can incorporate joint limits and obstacle avoidance.
    *   **Disadvantages:** Slower (computationally intensive), may get stuck in local minima, not guaranteed to find a solution, accuracy depends on initial guess.
3.  **Geometric Solutions:**
    *   **Principle:** Uses geometric relationships (e.g., law of cosines, law of sines) for simple arms (e.g., 2R planar arm).
    *   **Advantages:** Intuitive, fast for specific structures.
    *   **Disadvantages:** Limited to very simple kinematics.

#### 4.3.3 Application

*   **Trajectory Generation:** Planning a sequence of joint movements to follow a desired end-effector path.
*   **Grasping:** Positioning the gripper at an object.
*   **Human-Robot Interaction:** Allowing users to specify target positions without worrying about joint angles.

### 4.4 Kinematics for Mobile Robots

For mobile robots, kinematics focuses on the relationship between wheel velocities and the robot's overall linear and angular velocity (odometry).

*   **Differential Drive Robots:** Two independent drive wheels.
    *   **Forward Kinematics:** Given left and right wheel speeds, calculate robot's linear velocity and angular velocity.
    *   **Inverse Kinematics:** Given desired linear and angular velocity, calculate required left and right wheel speeds.
*   **Omnidirectional Robots:** Wheels designed for movement in any direction (e.g., Mecanum wheels).
    *   More complex kinematics involving multiple wheel speeds to achieve desired body velocity.

Kinematics is a foundational aspect of robotics, enabling the precise control and planning of robot movement, whether it's the tip of a robotic arm or the center of a mobile base.

---

### C++ Example: Forward Kinematics for a 2-DOF Planar Arm

This C++ example implements forward kinematics for a simple 2-Degree-of-Freedom (DOF) planar robotic arm.

```cpp
#include <iostream>
#include <cmath> // For sin, cos, M_PI
#include <iomanip> // For std::fixed, std::setprecision

// Define PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Structure to hold end-effector position
struct EndEffectorPose {
    float x;
    float y;
};

// Function for Forward Kinematics of a 2-DOF planar arm
// L1: length of first link
// L2: length of second link
// theta1: angle of first joint (radians) relative to base X-axis
// theta2: angle of second joint (radians) relative to first link
EndEffectorPose forwardKinematics(float L1, float L2, float theta1, float theta2) {
    EndEffectorPose pose;

    // X-coordinate
    pose.x = L1 * std::cos(theta1) + L2 * std::cos(theta1 + theta2);

    // Y-coordinate
    pose.y = L1 * std::sin(theta1) + L2 * std::sin(theta1 + theta2);

    return pose;
}

int main() {
    std::cout << "--- Forward Kinematics for 2-DOF Planar Arm ---" << std::endl;

    const float link1_length = 10.0f; // cm
    const float link2_length = 8.0f;  // cm

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Link 1 Length: " << link1_length << " cm" << std::endl;
    std::cout << "Link 2 Length: " << link2_length << " cm" << std::endl;

    // Test Case 1: Arm fully extended horizontally
    float joint1_angle_deg1 = 0.0f;
    float joint2_angle_deg1 = 0.0f;
    float theta1_rad1 = joint1_angle_deg1 * M_PI / 180.0f;
    float theta2_rad1 = joint2_angle_deg1 * M_PI / 180.0f;
    EndEffectorPose pose1 = forwardKinematics(link1_length, link2_length, theta1_rad1, theta2_rad1);
    std::cout << "\nJoint Angles: Theta1 = " << joint1_angle_deg1 << " deg, Theta2 = " << joint2_angle_deg1 << " deg" << std::endl;
    std::cout << "End-Effector Pose: (x=" << pose1.x << ", y=" << pose1.y << ") cm" << std::endl;
    // Expected: (18.00, 0.00)

    // Test Case 2: Arm bent upwards
    float joint1_angle_deg2 = 45.0f;
    float joint2_angle_deg2 = 45.0f;
    float theta1_rad2 = joint1_angle_deg2 * M_PI / 180.0f;
    float theta2_rad2 = joint2_angle_deg2 * M_PI / 180.0f;
    EndEffectorPose pose2 = forwardKinematics(link1_length, link2_length, theta1_rad2, theta2_rad2);
    std::cout << "\nJoint Angles: Theta1 = " << joint1_angle_deg2 << " deg, Theta2 = " << joint2_angle_deg2 << " deg" << std::endl;
    std::cout << "End-Effector Pose: (x=" << pose2.x << ", y=" << pose2.y << ") cm" << std::endl;
    // Expected: (6.06, 12.06)

    // Test Case 3: Arm reaching straight up
    float joint1_angle_deg3 = 90.0f;
    float joint2_angle_deg3 = 0.0f;
    float theta1_rad3 = joint1_angle_deg3 * M_PI / 180.0f;
    float theta2_rad3 = joint2_angle_deg3 * M_PI / 180.0f;
    EndEffectorPose pose3 = forwardKinematics(link1_length, link2_length, theta1_rad3, theta2_rad3);
    std::cout << "\nJoint Angles: Theta1 = " << joint1_angle_deg3 << " deg, Theta2 = " << joint2_angle_deg3 << " deg" << std::endl;
    std::cout << "End-Effector Pose: (x=" << pose3.x << ", y=" << pose3.y << ") cm" << std::endl;
    // Expected: (0.00, 18.00)


    std::cout << "\nForward kinematics simulation finished." << std::endl;
    return 0;
}
```

---

### Python Example: Inverse Kinematics for a 2-DOF Planar Arm (Geometric Solution)

This Python example implements a geometric solution for Inverse Kinematics for a simple 2-DOF planar robotic arm.

```python
import math

# Function for Inverse Kinematics of a 2-DOF planar arm
# L1: length of first link
# L2: length of second link
# x, y: desired end-effector position
# Returns: (theta1_deg, theta2_deg) in degrees, or None if unreachable
def inverseKinematics(L1, L2, x, y):
    """
    Solves inverse kinematics for a 2-DOF planar arm using a geometric approach.
    Returns joint angles (theta1, theta2) in degrees.
    """
    # Calculate square of distance from base to target (D^2)
    D_squared = x**2 + y**2
    D = math.sqrt(D_squared)

    # Check if target is out of reach
    if D > (L1 + L2) or D < abs(L1 - L2):
        print(f"Target ({x:.2f}, {y:.2f}) unreachable. Max reach: {L1+L2:.2f}, Min reach: {abs(L1-L2):.2f}.")
        return None, None

    # Calculate theta2 using the Law of Cosines
    # D^2 = L1^2 + L2^2 - 2*L1*L2*cos(pi - theta2)
    # cos(pi - theta2) = -cos(theta2)
    # D^2 = L1^2 + L2^2 + 2*L1*L2*cos(theta2)
    # cos(theta2) = (D^2 - L1^2 - L2^2) / (2 * L1 * L2)
    cos_theta2 = (D_squared - L1**2 - L2**2) / (2 * L1 * L2)
    
    # Due to floating point inaccuracies, cos_theta2 might be slightly outside [-1, 1]
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))

    theta2_rad = math.acos(cos_theta2) # Elbow up solution (positive theta2)
    # For elbow down solution: theta2_rad = -math.acos(cos_theta2)

    # Calculate theta1
    # Using atan2(y,x) to handle quadrants correctly
    # Alpha is the angle from base to end-effector
    alpha = math.atan2(y, x)

    # Beta is the angle in the triangle formed by L1, L2, and D, opposite L2
    # cos(beta) = (L1^2 + D_squared - L2^2) / (2 * L1 * D)
    cos_beta = (L1**2 + D_squared - L2**2) / (2 * L1 * D)
    cos_beta = max(-1.0, min(1.0, cos_beta)) # Clamp for numerical stability
    beta = math.acos(cos_beta)

    theta1_rad = alpha - beta # Elbow up solution (based on geometry)
    # For elbow down solution: theta1_rad = alpha + beta

    theta1_deg = math.degrees(theta1_rad)
    theta2_deg = math.degrees(theta2_rad)

    # Normalize angles to [0, 360) or (-180, 180] as preferred
    theta1_deg = (theta1_deg + 360) % 360
    theta2_deg = (theta2_deg + 360) % 360

    return theta1_deg, theta2_deg

if __name__ == "__main__":
    link1_length = 10.0 # cm
    link2_length = 8.0  # cm

    print("--- Inverse Kinematics for 2-DOF Planar Arm ---")
    print(f"Link 1 Length: {link1_length:.2f} cm")
    print(f"Link 2 Length: {link2_length:.2f} cm")

    # Test Case 1: Target straight ahead
    target_x1, target_y1 = 18.0, 0.0
    theta1_1, theta2_1 = inverseKinematics(link1_length, link2_length, target_x1, target_y1)
    if theta1_1 is not None:
        print(f"\nTarget: ({target_x1:.2f}, {target_y1:.2f}) -> Joint Angles: (Theta1={theta1_1:.2f} deg, Theta2={theta2_1:.2f} deg)")
    
    # Test Case 2: Target straight up
    target_x2, target_y2 = 0.0, 18.0
    theta1_2, theta2_2 = inverseKinematics(link1_length, link2_length, target_x2, target_y2)
    if theta1_2 is not None:
        print(f"\nTarget: ({target_x2:.2f}, {target_y2:.2f}) -> Joint Angles: (Theta1={theta1_2:.2f} deg, Theta2={theta2_2:.2f} deg)")

    # Test Case 3: Target unreachable
    target_x3, target_y3 = 20.0, 20.0
    theta1_3, theta2_3 = inverseKinematics(link1_length, link2_length, target_x3, target_y3)
    if theta1_3 is not None:
        print(f"\nTarget: ({target_x3:.2f}, {target_y3:.2f}) -> Joint Angles: (Theta1={theta1_3:.2f} deg, Theta2={theta2_3:.2f} deg)")

    # Test Case 4: Target within reach, bent arm
    target_x4, target_y4 = 10.0, 10.0
    theta1_4, theta2_4 = inverseKinematics(link1_length, link2_length, target_x4, target_y4)
    if theta1_4 is not None:
        print(f"\nTarget: ({target_x4:.2f}, {target_y4:.2f}) -> Joint Angles: (Theta1={theta1_4:.2f} deg, Theta2={theta2_4:.2f} deg)")

    print("\nInverse kinematics simulation finished.")
```

---

### Arduino Example: Mobile Robot Differential Drive Kinematics

This Arduino sketch demonstrates the **inverse kinematics** for a differential drive mobile robot, calculating required wheel speeds from desired linear and angular velocities.

```arduino
// Mobile Robot Differential Drive Inverse Kinematics

// Robot parameters
const float WHEEL_RADIUS = 0.03; // meters (3 cm)
const float WHEEL_BASE = 0.15;   // meters (15 cm - distance between wheels)

// Desired robot velocities
float desiredLinearVelocity = 0.0; // meters/second
float desiredAngularVelocity = 0.0; // radians/second

// Calculated wheel velocities
float leftWheelVelocity = 0.0;   // radians/second
float rightWheelVelocity = 0.0;  // radians/second

// --- Inverse Kinematics Function ---
// Given desired linear (v) and angular (omega) velocities, calculate wheel velocities.
void calculateWheelVelocities(float v, float omega) {
  // v_R = v + (omega * L / 2)
  // v_L = v - (omega * L / 2)
  // Where v_R, v_L are linear speeds of right/left wheels
  // And v_wheel_angular = v_wheel_linear / WHEEL_RADIUS

  float v_right_linear = v + (omega * WHEEL_BASE / 2.0);
  float v_left_linear = v - (omega * WHEEL_BASE / 2.0);

  rightWheelVelocity = v_right_linear / WHEEL_RADIUS;
  leftWheelVelocity = v_left_linear / WHEEL_RADIUS;
}

// --- Forward Kinematics Function (for verification/odometry) ---
// Given wheel velocities, calculate robot's linear and angular velocities.
void calculateRobotVelocities(float v_left_wheel, float v_right_wheel) {
  float v_left_linear = v_left_wheel * WHEEL_RADIUS;
  float v_right_linear = v_right_wheel * WHEEL_RADIUS;

  desiredLinearVelocity = (v_right_linear + v_left_linear) / 2.0;
  desiredAngularVelocity = (v_right_linear - v_left_linear) / WHEEL_BASE;
}


void setup() {
  Serial.begin(9600);
  Serial.println("Arduino Differential Drive Kinematics Demo Ready.");
  Serial.print("Wheel Radius: "); Serial.print(WHEEL_RADIUS); Serial.println(" m");
  Serial.print("Wheel Base:   "); Serial.print(WHEEL_BASE); Serial.println(" m");
}

void loop() {
  Serial.println("\n--- New Command ---");

  // Test Case 1: Move Straight Forward
  desiredLinearVelocity = 0.1; // 0.1 m/s
  desiredAngularVelocity = 0.0; // No turn
  calculateWheelVelocities(desiredLinearVelocity, desiredAngularVelocity);
  Serial.print("Desired: Linear V="); Serial.print(desiredLinearVelocity); Serial.print(" m/s, Angular Omega="); Serial.print(desiredAngularVelocity); Serial.println(" rad/s");
  Serial.print("  Calculated Wheel Velocities: Left="); Serial.print(leftWheelVelocity); Serial.print(" rad/s, Right="); Serial.print(rightWheelVelocity); Serial.println(" rad/s");
  
  // Verify with FK
  calculateRobotVelocities(leftWheelVelocity, rightWheelVelocity);
  Serial.print("  FK Verified: Linear V="); Serial.print(desiredLinearVelocity); Serial.print(" m/s, Angular Omega="); Serial.print(desiredAngularVelocity); Serial.println(" rad/s");

  delay(3000);

  // Test Case 2: Turn Left
  desiredLinearVelocity = 0.05; // 0.05 m/s (slower forward while turning)
  desiredAngularVelocity = 0.5; // 0.5 rad/s (turning left)
  calculateWheelVelocities(desiredLinearVelocity, desiredAngularVelocity);
  Serial.print("Desired: Linear V="); Serial.print(desiredLinearVelocity); Serial.print(" m/s, Angular Omega="); Serial.print(desiredAngularVelocity); Serial.println(" rad/s");
  Serial.print("  Calculated Wheel Velocities: Left="); Serial.print(leftWheelVelocity); Serial.print(" rad/s, Right="); Serial.print(rightWheelVelocity); Serial.println(" rad/s");
  delay(3000);

  // Test Case 3: Pure Rotation (Spin in place)
  desiredLinearVelocity = 0.0; // No forward motion
  desiredAngularVelocity = 1.0; // 1 rad/s (spinning left)
  calculateWheelVelocities(desiredLinearVelocity, desiredAngularVelocity);
  Serial.print("Desired: Linear V="); Serial.print(desiredLinearVelocity); Serial.print(" m/s, Angular Omega="); Serial.print(desiredAngularVelocity); Serial.println(" rad/s");
  Serial.print("  Calculated Wheel Velocities: Left="); Serial.print(leftWheelVelocity); Serial.print(" rad/s, Right="); Serial.print(rightWheelVelocity); Serial.println(" rad/s");
  delay(3000);
}
```

---

### Equations in LaTeX: Homogeneous Transformation Matrix

A **homogeneous transformation matrix** `T` combines both rotation `R` and translation `t` into a single matrix. This is fundamental for Forward Kinematics.

```latex
T = begin{bmatrix} R_{3 times 3} & t_{3 times 1}  0_{1 times 3} & 1 end{bmatrix}
```

Where `R` is the `3 times 3` rotation matrix and `t` is the `3 	imes 1` translation vector.

For 2D planar kinematics, a 3x3 matrix is used:
```latex
T = begin{bmatrix} cos theta & -sin theta & x  sin theta & cos theta & y  0 & 0 & 1 end{bmatrix}
```

---

### MCQs with Answers

1.  Which branch of robot kinematics calculates the position and orientation of the end-effector given the joint angles and link lengths?
    a) Inverse Kinematics
    b) Forward Kinematics
    c) Differential Kinematics
    d) Velocity Kinematics
    *Answer: b) Forward Kinematics*

2.  What is a key challenge that often makes Inverse Kinematics more difficult to solve than Forward Kinematics?
    a) Inverse Kinematics problems always have only one unique solution.
    b) Inverse Kinematics can involve multiple possible joint configurations for a single end-effector pose.
    c) Forward Kinematics requires more complex mathematical tools.
    d) Inverse Kinematics does not account for joint limits.
    *Answer: b) Inverse Kinematics can involve multiple possible joint configurations for a single end-effector pose.*

3.  The Denavit-Hartenberg (DH) convention is a standardized method used to:
    a) Calculate the shortest path in a robot's environment.
    b) Define coordinate systems for each joint in a serial kinematic chain to simplify forward kinematics.
    c) Determine the required motor torques for a robot's movement.
    d) Control the speed and direction of a differential drive robot.
    *Answer: b) Define coordinate systems for each joint in a serial kinematic chain to simplify forward kinematics.*

---

### Practice Tasks

1.  **Forward Kinematics for a 3-DOF Arm (Conceptual):** Imagine a 3-DOF planar arm (like the 2-DOF arm with an additional joint and link). Describe how you would extend the `forwardKinematics` function to calculate the end-effector position (x, y) given three joint angles (`theta_1, theta_2, theta_3`) and three link lengths (`L_1, L_2, L_3`).
2.  **Inverse Kinematics Workspace Analysis:** For the 2-DOF planar arm from the examples (L1=10cm, L2=8cm), what is the maximum reach (largest distance from the base) and minimum reach (smallest distance from the base for all reachable points)? Sketch the workspace.
3.  **Differential Drive FK/IK Application:** You have a differential drive robot that needs to:
    *   Move straight forward at 0.2 m/s.
    *   Rotate in place (spin) clockwise at 0.8 rad/s.
    *   Turn right while moving forward (right wheel slower than left wheel).
    For each scenario, describe (conceptually, or with pseudo-calculations) the linear and angular velocities (`v, omega`) or the required left and right wheel speeds (`v_L, v_R`) involved.

---

### Notes for Teachers

*   **Visual Aids:** Use physical robot arm models or interactive simulations (e.g., in CoppeliaSim, ROS Rviz) to demonstrate FK and IK.
*   **Geometric Intuition:** Emphasize the geometric intuition behind the equations, especially for 2D arms.
*   **IK Complexity:** Clearly explain why IK is generally harder than FK, highlighting multiple solutions and singularities.

### Notes for Students

*   **Master Transformations:** A solid understanding of coordinate transformations (rotation and translation) is crucial for kinematics.
*   **Practice with Angles:** Be careful with units (degrees vs. radians) in trigonometric functions. Most math libraries use radians.
*   **Analytical vs. Numerical:** Understand when analytical solutions for IK are possible and when numerical methods are necessary.
*   **Workspace:** Familiarize yourself with the concept of a robot's workspace, the set of all reachable points for its end-effector.
