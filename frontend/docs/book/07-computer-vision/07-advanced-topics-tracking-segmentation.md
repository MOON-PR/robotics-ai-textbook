---
id: book-07-computer-vision-07-advanced-topics-tracking-segmentation
title: 'import cv2 # Uncomment to run with actual OpenCV'
sidebar_position: 7
---

--- 
sidebar_position: 7
title: Advanced Topics (Tracking, Segmentation)
---

## 07-Advanced Topics (Tracking, Segmentation)

Beyond simply detecting objects, robots often need a more nuanced understanding of their visual world. **Object tracking** enables a robot to follow the motion of detected objects over time, while **image segmentation** allows for a precise, pixel-level understanding of which parts of an image belong to which objects or categories. These advanced computer vision topics are crucial for sophisticated robotic interactions and scene analysis.

### 7.1 Object Tracking

**Object tracking** is the process of locating a moving object (or multiple objects) in successive frames of a video. It allows a robot to monitor the behavior of dynamic elements in its environment, predict their future positions, and react accordingly.

#### 7.1.1 Why Track?

*   **Prediction:** Predict where an object will be next.
*   **Behavior Analysis:** Understand patterns of movement (e.g., human gait).
*   **Interaction:** Manipulating moving objects, following a person.
*   **Data Association:** Maintaining a consistent identity for an object across frames.
*   **Efficiency:** Once an object is detected, tracking can be more efficient than re-detecting in every frame.

#### 7.1.2 Tracking-by-Detection

The most common approach:
1.  **Detect:** An object detector (e.g., YOLO, Faster R-CNN) identifies objects in each frame.
2.  **Associate:** A data association algorithm links current detections to existing tracks. This is often done using proximity (bounding box overlap), motion prediction (e.g., Kalman filter to predict next position), and feature matching.
3.  **Track Update:** The state of the track (position, velocity, identity) is updated based on the associated detection.
4.  **Track Management:** New tracks are created for new detections; old tracks are terminated if they are not detected for several frames.

#### 7.1.3 Common Tracking Algorithms

*   **Correlation Filters (e.g., KCF, MOSSE):** Learn a model of the object's appearance and then efficiently search for it in subsequent frames using correlation. Fast but can struggle with appearance changes.
*   **Kalman Filters (for state estimation):** Used within tracking-by-detection to predict the object's next position and filter noisy detections, improving association.
*   **Deep Learning Trackers:** Use CNNs to learn object appearance models or perform end-to-end tracking (e.g., Siamese networks).

#### 7.1.4 Metrics for Tracking

*   **MOTA (Multiple Object Tracking Accuracy):** Combines false positives, false negatives, and ID switches.
*   **MOTP (Multiple Object Tracking Precision):** Measures the average overlap between ground truth and predicted bounding boxes.

**Diagram 7.1: Tracking-by-Detection Pipeline**

```mermaid
graph TD
    A[Video Stream] --> B(Object Detector)
    B --> C(Detections (Frame t))
    C --> D(Data Association)
    D -- Links to --> E(Existing Tracks)
    E --> F(Track Management)
    F -- Updates --> E
    F -- Outputs --> G[Tracked Objects with IDs]
```

*Description: A flow diagram illustrating the Tracking-by-Detection pipeline, where an object detector generates detections, which are then associated with and used to update existing object tracks.*

### 7.2 Image Segmentation

**Image segmentation** is the process of partitioning an image into multiple segments (sets of pixels), typically to identify objects, boundaries, or regions with similar properties. It provides a more fine-grained understanding of the image than bounding box detection.

#### 7.2.1 Types of Segmentation

*   **Semantic Segmentation:** Classifies every pixel in an image into a predefined category (e.g., "road," "person," "sky," "car"). All instances of the same class are treated as one (e.g., all pixels belonging to "car" are labeled "car").
*   **Instance Segmentation:** Identifies and delineates each individual object instance in the image. This means if there are two cars, they are segmented as "car_1" and "car_2."
*   **Panoptic Segmentation:** Combines semantic and instance segmentation. It assigns a class label to every pixel and also distinguishes between individual object instances for "things" (e.g., cars, people) and assigns category labels for "stuff" (e.g., sky, road, grass).

#### 7.2.2 Segmentation Algorithms

*   **Traditional Methods:**
    *   **Thresholding:** Simple binarization based on pixel intensity.
    *   **Watershed Algorithm:** Treats image as a topographic map, finding "watershed lines" to segment objects.
    *   **Graph-based Segmentation:** Models image as a graph, partitioning it based on edge weights.
*   **Deep Learning Methods:** Have revolutionized segmentation, achieving state-of-the-art results.
    *   **Fully Convolutional Networks (FCNs):** Adapt CNNs for pixel-wise classification.
    *   **U-Net:** A popular FCN architecture, especially for biomedical image segmentation.
    *   **Mask R-CNN:** Extends Faster R-CNN by adding a branch for predicting an object mask in parallel with the bounding box and class prediction. (For instance segmentation).

### 7.3 Applications in Robotics

#### 7.3.1 Object Tracking

*   **Autonomous Driving:** Tracking pedestrians, cyclists, other vehicles for collision prediction.
*   **Human-Robot Collaboration:** Tracking human limbs or tools for safe and efficient interaction.
*   **Drone Surveillance:** Tracking targets or areas of interest.

#### 7.3.2 Image Segmentation

*   **Semantic SLAM:** Building maps that understand the "meaning" of different regions (e.g., traversable ground, walls, furniture).
*   **Grasping & Manipulation:** Precisely identifying the boundaries of objects to be grasped, even complex or deformable ones.
*   **Path Planning in Complex Environments:** Differentiating free space from obstacles at a pixel level, understanding "driveable" surfaces.
*   **Robotic Surgery:** Segmenting organs and tissues for precise intervention.
*   **Agriculture:** Segmenting crops from weeds, identifying ripe fruits.

Tracking and segmentation provide robots with an advanced level of visual intelligence, enabling them to move beyond simple object recognition to a dynamic, pixel-accurate understanding of their environment, which is critical for complex and adaptive robotic behaviors.

--- 

### C++ Example: Conceptual Object Tracking (Kalman Filter for Bounding Box)

This C++ example conceptually simulates using a Kalman Filter to track the center of an object's bounding box, fusing noisy detections.

```cpp
#include <iostream>
#include <vector>
#include <string>
#include <random> // For random number generation
#include <chrono> // For std::chrono::milliseconds
#include <thread> // For std::this_thread::sleep_for
#include <iomanip> // For std::fixed, std::setprecision

// --- Simple 2D Kalman Filter for Position and Velocity ---
// State: [x, y, vx, vy]^T
// Measurement: [x_meas, y_meas]^T (from detector)
class KalmanFilter2D {
private:
    // State vector [x, y, vx, vy]
    Eigen::VectorXd x_hat;
    // Covariance matrix
    Eigen::MatrixXd P;
    // State transition matrix
    Eigen::MatrixXd F;
    // Measurement matrix
    Eigen::MatrixXd H;
    // Process noise covariance
    Eigen::MatrixXd Q;
    // Measurement noise covariance
    Eigen::MatrixXd R;

public:
    KalmanFilter2D(double dt, double proc_noise_pos, double proc_noise_vel, double meas_noise) {
        // Initial state (assume stationary initially)
        x_hat = Eigen::VectorXd(4);
        x_hat << 0, 0, 0, 0;

        // Initial covariance (large uncertainty)
        P = Eigen::MatrixXd(4, 4);
        P << 1000, 0, 0, 0,
             0, 1000, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

        // State transition matrix (constant velocity model)
        F = Eigen::MatrixXd(4, 4);
        F << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

        // Measurement matrix (we measure x and y position)
        H = Eigen::MatrixXd(2, 4);
        H << 1, 0, 0, 0,
             0, 1, 0, 0;

        // Process noise covariance (tune these)
        Q = Eigen::MatrixXd(4, 4);
        Q << pow(proc_noise_pos, 2), 0, 0, 0,
             0, pow(proc_noise_pos, 2), 0, 0,
             0, 0, pow(proc_noise_vel, 2), 0,
             0, 0, 0, pow(proc_noise_vel, 2);

        // Measurement noise covariance (tune this)
        R = Eigen::MatrixXd(2, 2);
        R << pow(meas_noise, 2), 0,
             0, pow(meas_noise, 2);
    }

    void predict() {
        x_hat = F * x_hat;
        P = F * P * F.transpose() + Q;
    }

    void update(const Eigen::VectorXd& z) {
        Eigen::VectorXd y = z - H * x_hat; // Innovation
        Eigen::MatrixXd S = H * P * H.transpose() + R; // Innovation covariance
        Eigen::MatrixXd K = P * H.transpose() * S.inverse(); // Kalman Gain

        x_hat = x_hat + K * y; // Update state
        P = (Eigen::MatrixXd::Identity(4, 4) - K * H) * P; // Update covariance
    }

    Eigen::VectorXd getState() const {
        return x_hat;
    }
};

// --- Mockup for Eigen Library (if not installed) ---
// If Eigen is not installed, this provides a minimal set of classes/functions
// to allow the conceptual code to compile and run for demonstration.
// For real use, install Eigen (e.g., apt install libeigen3-dev)
namespace Eigen {
    // Forward declarations (simplified)
    class VectorXd {
    public:
        std::vector<double> data;
        VectorXd(int size) : data(size) {}
        double operator()(int i) const { return data[i]; }
        double& operator()(int i) { return data[i]; }
        // Minimal operations for this example
        VectorXd operator*(const MatrixXd& other) const; // Placeholder
        VectorXd operator+(const VectorXd& other) const;
        VectorXd operator-(const VectorXd& other) const;
        VectorXd operator*(const VectorXd& other) const; // Element-wise for simpler cases
        int size() const { return data.size(); }
        // Stream output
        friend std::ostream& operator<<(std::ostream& os, const VectorXd& v) {
            os << "[";
            for(size_t i=0; i<v.data.size(); ++i) os << v.data[i] << (i == v.data.size()-1 ? "" : ", ");
            os << "]"; return os;
        }
    };
    class MatrixXd {
    public:
        std::vector<std::vector<double>> data;
        MatrixXd(int rows, int cols) : data(rows, std::vector<double>(cols)) {}
        double operator()(int r, int c) const { return data[r][c]; }
        double& operator()(int r, int c) { return data[r][c]; }
        int rows() const { return data.size(); }
        int cols() const { return data.empty() ? 0 : data[0].size(); }
        MatrixXd transpose() const;
        MatrixXd inverse() const; // Placeholder
        MatrixXd operator*(const MatrixXd& other) const;
        MatrixXd operator+(const MatrixXd& other) const;
        MatrixXd operator-(const MatrixXd& other) const;
        static MatrixXd Identity(int size, int size2); // Simplified
        friend std::ostream& operator<<(std::ostream& os, const MatrixXd& m) {
            os << "\n";
            for(size_t i=0; i<m.data.size(); ++i) {
                for(size_t j=0; j<m.data[0].size(); ++j) os << std::setw(8) << std::fixed << std::setprecision(2) << m.data[i][j];
                os << "\n";
            } return os;
        }
    };
    // Implementations to satisfy the example (very minimal)
    VectorXd VectorXd::operator+(const VectorXd& other) const { VectorXd res(size()); for(int i=0; i<size(); ++i) res.data[i] = data[i] + other.data[i]; return res; }
    VectorXd VectorXd::operator-(const VectorXd& other) const { VectorXd res(size()); for(int i=0; i<size(); ++i) res.data[i] = data[i] - other.data[i]; return res; }
    MatrixXd MatrixXd::transpose() const { MatrixXd res(cols(), rows()); for(int r=0; r<rows(); ++r) for(int c=0; c<cols(); ++c) res.data[c][r] = data[r][c]; return res; }
    MatrixXd MatrixXd::operator*(const MatrixXd& other) const { MatrixXd res(rows(), other.cols()); for(int i=0; i<rows(); ++i) for(int j=0; j<other.cols(); ++j) for(int k=0; k<cols(); ++k) res.data[i][j] += data[i][k] * other.data[k][j]; return res; }
    MatrixXd MatrixXd::operator+(const MatrixXd& other) const { MatrixXd res(rows(), cols()); for(int i=0; i<rows(); ++i) for(int j=0; j<cols(); ++j) res.data[i][j] = data[i][j] = data[i][j] + other.data[i][j]; return res; }
    MatrixXd MatrixXd::operator-(const MatrixXd& other) const { MatrixXd res(rows(), cols()); for(int i=0; i<rows(); ++i) for(int j=0; j<cols(); ++j) res.data[i][j] = data[i][j] = data[i][j] - other.data[i][j]; return res; }
    MatrixXd MatrixXd::Identity(int rows, int cols) { MatrixXd res(rows, cols); for(int i=0; i<rows && i<cols; ++i) res.data[i][i] = 1.0; return res; }
    MatrixXd MatrixXd::inverse() const { /* Simplistic for 1x1 or 2x2. For larger, need more complex math. For 1D/2D measurement, S is 1x1/2x2. Assume invertible.*/ MatrixXd res(rows(), cols()); res.data[0][0] = 1.0 / data[0][0]; return res; } // Very basic inverse for 1x1 S
    VectorXd operator*(const MatrixXd& m, const VectorXd& v) { VectorXd res(m.rows()); for(int i=0; i<m.rows(); ++i) for(int j=0; j<m.cols(); ++j) res.data[i] += m(i,j) * v(j); return res; }
    VectorXd operator*(const VectorXd& v, const MatrixXd& m) { return m.transpose() * v; } // Simplified for this context
}
// --- End Mockup ---

// Simulate object detection (noisy measurement)
Eigen::VectorXd getNoisyDetection(double true_x, double true_y, double noise_std) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::normal_distribution<> dist_x(0, noise_std);
    static std::normal_distribution<> dist_y(0, noise_std);
    Eigen::VectorXd measurement(2);
    measurement << true_x + dist_x(gen), true_y + dist_y(gen);
    return measurement;
}

int main() {
    std::cout << "--- Conceptual Object Tracking with Kalman Filter (C++) ---" << std::endl;
    std::cout << "Requires Eigen library for proper matrix operations. Using simple mockups for demo." << std::endl;

    double dt = 0.1; // Time step (seconds)
    double process_noise_pos_std = 0.1; // Std dev for position part of process noise
    double process_noise_vel_std = 0.1; // Std dev for velocity part of process noise
    double measurement_noise_std = 2.0; // Std dev for detection noise (pixels)

    KalmanFilter2D kf(dt, process_noise_pos_std, process_noise_vel_std, measurement_noise_std);

    // Simulated true object motion
    double true_obj_x = 0;
    double true_obj_y = 0;
    double true_obj_vx = 5; // moving right at 5 units/sec
    double true_obj_vy = 1; // moving up at 1 unit/sec

    std::cout << "\nTime\tTrueX\tTrueY\tMeasX\tMeasY\tEstX\tEstY\tEstVx\tEstVy" << std::endl;
    std::cout << std::fixed << std::setprecision(2);

    for (int i = 0; i < 50; ++i) { // Simulate 5 seconds
        // Update true object position
        true_obj_x += true_obj_vx * dt;
        true_obj_y += true_obj_vy * dt;

        // Get a noisy detection
        Eigen::VectorXd measurement = getNoisyDetection(true_obj_x, true_obj_y, measurement_noise_std);

        kf.predict();
        kf.update(measurement);

        Eigen::VectorXd state = kf.getState();

        std::cout << i * dt << "\t" << true_obj_x << "\t" << true_obj_y << "\t"
                  << measurement(0) << "\t" << measurement(1) << "\t"
                  << state(0) << "\t" << state(1) << "\t"
                  << state(2) << "\t" << state(3) << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(dt * 1000)));
    }

    std::cout << "\nConceptual object tracking demo finished." << std::endl;
    return 0;
}
```

--- 

### Python Example: Conceptual Semantic Segmentation with FCN-like Output

This Python example conceptually simulates the output of a Semantic Segmentation model (like an FCN), where each pixel is classified into a category.

```python
import numpy as np
# import cv2 # Uncomment to run with actual OpenCV
import matplotlib.pyplot as plt

def conceptual_semantic_segmentation():
    print("--- Conceptual Semantic Segmentation ---")

    # Step 1: Simulate Input Image
    # Imagine a simplified 20x20 image representing a scene
    simulated_img_height, simulated_img_width = 20, 20
    # In reality: img = cv2.imread("robot_scene.jpg")
    print(f"1. Simulated input image ({simulated_img_width}x{simulated_img_height}) acquired.")

    # Step 2: Define Classes
    class_labels = ["background", "road", "person", "car"]
    num_classes = len(class_labels)
    
    # Step 3: Simulate CNN Output (Pixel-wise Class Probabilities)
    # A tensor of shape (height, width, num_classes)
    # Each (h,w) pixel has a probability distribution over classes
    
    # Initialize with background probabilities
    simulated_prob_map = np.zeros((simulated_img_height, simulated_img_width, num_classes), dtype=np.float32)
    simulated_prob_map[:, :, 0] = 0.8 # 80% chance of background everywhere
    simulated_prob_map[:, :, 1:] = 0.1 / (num_classes - 1) # Distribute remaining to other classes

    # Simulate a "road" section
    simulated_prob_map[10:20, :, 0] = 0.1 # Less background probability in road area
    simulated_prob_map[10:20, :, 1] = 0.8 # High road probability
    simulated_prob_map[10:20, :, 2:] = 0.1 / (num_classes - 2)

    # Simulate a "person" object
    simulated_prob_map[5:10, 10:12, 0] = 0.1 # Less background
    simulated_prob_map[5:10, 10:12, 2] = 0.8 # High person probability

    # Simulate a "car" object
    simulated_prob_map[12:15, 5:8, 0] = 0.1
    simulated_prob_map[12:15, 5:8, 3] = 0.8 # High car probability

    print(f"2. Simulated CNN output (pixel-wise class probabilities) generated.")

    # Step 4: Generate Segmentation Map
    # For each pixel, assign the class with the highest probability
    segmentation_map = np.argmax(simulated_prob_map, axis=2) # Get index of max prob

    print("3. Segmentation map generated (each pixel assigned a class ID).")

    # Step 5: Visualize Segmentation (Conceptual)
    # Create a color map for visualization
    colors = np.array([
        [0, 0, 0],       # background (black)
        [128, 64, 128],  # road (purple)
        [0, 255, 0],     # person (green)
        [0, 0, 255]      # car (blue)
    ], dtype=np.uint8)

    # Create an RGB image from the segmentation map
    segmented_image_rgb = colors[segmentation_map]

    # Plotting (requires matplotlib)
    fig, axes = plt.subplots(1, 2, figsize=(10, 5))
    
    axes[0].imshow(np.zeros((simulated_img_height, simulated_img_width, 3), dtype=np.uint8)) # Just a black image for original
    axes[0].set_title('Original Image (Conceptual)')
    axes[0].axis('off')

    axes[1].imshow(segmented_image_rgb)
    axes[1].set_title('Semantic Segmentation Map')
    axes[1].axis('off')
    
    # Add a legend for classes
    handles = [plt.Rectangle((0,0),1,1, color=c/255.0) for c in colors]
    axes[1].legend(handles, class_labels, bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

    plt.tight_layout()
    plt.show()
    print("\nConceptual semantic segmentation visualization finished. Close plot to continue.")

if __name__ == "__main__":
    conceptual_semantic_segmentation()
```

--- 

### Arduino Example: Background Subtraction (Conceptual)

This Arduino sketch conceptually shows the logic of **background subtraction**, a technique often used in tracking to detect moving objects. It operates on simulated pixel values rather than actual images.

```arduino
// Conceptual Background Subtraction for Motion Detection
// This sketch simulates a basic background subtraction algorithm
// on single pixel values to detect changes (motion).
// Real-time image processing on basic Arduinos is NOT practical.

const byte THRESHOLD_DIFF = 20; // Threshold for pixel difference to detect change
const int motionLedPin = 13;    // Onboard LED

// Store a conceptual "background" pixel value
byte backgroundPixelValue = 128; // Mid-gray

void setup() {
  Serial.begin(9600);
  pinMode(motionLedPin, OUTPUT);
  digitalWrite(motionLedPin, LOW); // Off initially
  randomSeed(analogRead(A0)); // Seed random generator

  Serial.println("Arduino Conceptual Background Subtraction Demo Ready.");
  Serial.print("Initial Background Pixel: "); Serial.println(backgroundPixelValue);
}

void loop() {
  // Simulate a current pixel value from the camera feed
  byte currentPixelValue = random(100, 150); // Default around background
  
  // Introduce a "moving object" by making pixel darker/brighter occasionally
  if (random(0, 100) < 15) { // 15% chance of an object appearing
    currentPixelValue = random(0, 50); // Simulate a dark object
  } else if (random(0, 100) < 5) {
    currentPixelValue = random(200, 255); // Simulate a bright object
  }

  // Calculate the absolute difference from the background
  int diff = abs(currentPixelValue - backgroundPixelValue);

  Serial.print("Current Pixel: "); Serial.print(currentPixelValue);
  Serial.print(", Diff from BG: "); Serial.print(diff);

  if (diff > THRESHOLD_DIFF) {
    Serial.println(" -> MOTION DETECTED!");
    digitalWrite(motionLedPin, HIGH); // Turn on LED
    // Optionally update background model slowly
    backgroundPixelValue = (backgroundPixelValue * 0.9 + currentPixelValue * 0.1); // Slow update
  } else {
    Serial.println(" -> No motion.");
    digitalWrite(motionLedPin, LOW); // Turn off LED
  }
  
  delay(1000); // Simulate processing time (1 second per frame)
}
```

--- 

### Equations in LaTeX: IoU (Intersection over Union) for Segmentation

For segmentation, IoU (also known as the Jaccard index) can be used to compare a predicted segmentation mask `M_{p}` with a ground truth mask `M_{g}`:

```latex
text{IoU}(M_{p}, M_{g}) = frac{|M_{p} cap M_{g}|}{|M_{p} cup M_{g}|}
```

Where `| cdot |` denotes the number of pixels (area) in the region. This metric is computed per class and then averaged to get mean IoU (mIoU) for semantic segmentation.

--- 

### MCQs with Answers

1.  What is the primary goal of **object tracking** in computer vision for robotics?
    a) To classify every pixel in an image.
    b) To locate a moving object in successive frames of a video.
    c) To build a 3D map of the environment.
    d) To detect new, previously unseen objects.
    *Answer: b) To locate a moving object in successive frames of a video.*

2.  Which type of image segmentation identifies and delineates each individual object instance in an image (e.g., "car_1" and "car_2" if there are two cars)?
    a) Semantic Segmentation
    b) Panoptic Segmentation
    c) Instance Segmentation
    d) Binary Segmentation
    *Answer: c) Instance Segmentation*

3.  In a "tracking-by-detection" approach, what role does a **Kalman Filter** often play?
    a) To detect objects in each frame.
    b) To perform image segmentation.
    c) To predict the object's next position and filter noisy detections, improving association.
    d) To capture the video stream from the camera.
    *Answer: c) To predict the object's next position and filter noisy detections, improving association.*

--- 

### Practice Tasks

1.  **Tracking Dynamic Obstacles:** A mobile robot is navigating a factory floor. It uses object detection to identify moving forklifts. Explain how object tracking would enable the robot to safely avoid these dynamic obstacles, going beyond just a single-frame detection.
2.  **Semantic Segmentation for Navigation:** Describe how semantic segmentation could be used by an autonomous vehicle to understand its driveable area and identify non-driveable obstacles (e.g., distinguishing between "road," "sidewalk," "building," "tree").
3.  **Background Subtraction Limitations:** Research two common limitations or challenges of simple background subtraction algorithms for motion detection (e.g., changing illumination, moving background elements). How might these limitations be addressed?

--- 

### Notes for Teachers

*   **Real-time Demos:** Show videos of advanced computer vision applications (e.g., real-time instance segmentation, robust object tracking) to inspire students.
*   **Computational Cost:** Emphasize that these advanced techniques are computationally very intensive and often require significant processing power (GPUs) for real-time performance.
*   **Deep Learning Dominance:** Highlight that deep learning methods now dominate state-of-the-art results in both tracking and segmentation.

### Notes for Students

*   **Context is King:** Tracking and segmentation add crucial context to object detection, allowing robots to make more informed decisions.
*   **Data Association is Hard:** A key challenge in tracking is correctly associating detections across frames, especially in crowded scenes or with occlusions.
*   **Segmentation Accuracy:** Pixel-level accuracy provided by segmentation is invaluable for tasks requiring precise interaction.
*   **Combine Techniques:** Advanced robotic systems often combine detection, tracking, and segmentation to build a comprehensive understanding of their dynamic environment.
