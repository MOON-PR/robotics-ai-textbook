---
id: chapters-getting-started
title: Getting Started Guide
sidebar_position: 1
---
# Getting Started with Physical AI & Robotics

This guide will help you set up your learning environment and get started with the course.

## System Requirements

### Minimum Requirements
- **OS**: Windows 10+, macOS 10.14+, Ubuntu 18.04+
- **RAM**: 4GB minimum (8GB recommended)
- **Disk Space**: 2GB for tools, 5GB for Docker images
- **Internet**: Required for initial setup and accessing resources

### Recommended Setup
- **OS**: Ubuntu 20.04 LTS or Windows with WSL2
- **RAM**: 16GB for smooth simulation
- **Disk Space**: 10GB+ SSD for faster operation
- **GPU**: NVIDIA GPU recommended for AI components (not required)

## Installation Steps

### Step 1: Install Python

**Windows & macOS**:
1. Visit [python.org](https://www.python.org/downloads/)
2. Download Python 3.10 or later
3. Run installer (check "Add to PATH")

**Ubuntu/Debian**:
```bash
sudo apt-get update
sudo apt-get install python3.10 python3-pip python3-venv
```

Verify installation:
```bash
python3 --version
pip3 --version
```

### Step 2: Install Git

**Windows**: Download from [git-scm.com](https://git-scm.com/)

**macOS**:
```bash
brew install git
```

**Ubuntu/Debian**:
```bash
sudo apt-get install git
```

### Step 3: Clone the Repository

```bash
git clone https://github.com/robotics-textbook/ai-native-robotics.git
cd ai-native-robotics
```

### Step 4: Install Dependencies

#### For Frontend (Web Interface)

```bash
cd frontend
npm install
# or
yarn install
```

**Requirements**:
- Node.js 18+ (download from [nodejs.org](https://nodejs.org/))
- npm or yarn package manager

#### For Backend (Optional, Advanced Users)

```bash
cd backend
python3 -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install packages
pip install -r requirements.txt
```

### Step 5: Configure Docker (Optional)

For running AI backend and vector database:

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop)
2. Start the services:

```bash
docker compose up -d
```

This starts:
- Qdrant Vector Database (port 6333)
- FastAPI Backend (port 8000)
- Docusaurus Frontend (port 3000)

Check status:
```bash
docker compose ps
```

## Starting Your Learning

### Option 1: Web Interface (Easiest)

```bash
cd frontend
npm start
```

Then open your browser to `http://localhost:3000`

You can now:
- Browse all chapters
- Read content
- Ask the AI chatbot questions
- Select your learning level (Beginner/Intermediate/Advanced)

### Option 2: Full Stack (With AI Features)

```bash
# In root directory
docker compose up -d

# Wait for services to start (1-2 minutes)

# In frontend directory
cd frontend
npm start
```

Access the full platform at `http://localhost:3000` with:
- RAG chatbot for Q&A
- Personalized learning
- Urdu translation
- All interactive features

## Learning Path

### Week 1-2: Foundations
- Read Chapter 1: Introduction to Physical AI
- Understand robotics basics
- Set up your development environment

### Week 3-4: Hardware Fundamentals
- Study Chapter 2: Electronics & Circuits
- Learn about microcontrollers (Chapter 5)
- Set up your first Arduino project (optional)

### Week 5-6: Programming & Control
- Master Chapter 3: Programming Basics
- Study Chapter 6: Algorithms (especially PID control)
- Complete programming exercises

### Week 7-8: Perception Systems
- Study Chapter 4: Sensors & Perception
- Learn image processing basics (Chapter 7)
- Build a simple sensor integration project

### Week 9-10: AI Integration
- Study Chapter 8: AI & Robotics Integration
- Integrate machine learning models
- Work on AI-based decision making

### Week 11-12: Capstone
- Complete Chapter 9: Capstone Projects
- Build a comprehensive robot system
- Present your work

## Setting Up Your First Project

### Project 1: Blinking LED (Beginner)

**Goal**: Learn microcontroller basics

**Hardware**:
- Arduino Uno
- LED
- 220Ω Resistor
- Breadboard
- Jumper wires
- USB cable

**Circuit**:
```
Arduino Pin 13 → Resistor → LED Anode
LED Cathode → GND
```

**Code** (in Arduino IDE):
```cpp
void setup() {
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);   // LED on
  delay(1000);              // Wait 1 second
  digitalWrite(13, LOW);    // LED off
  delay(1000);              // Wait 1 second
}
```

### Project 2: Temperature Monitoring (Intermediate)

Build a system that reads temperature and sends it to your computer.

**Hardware**:
- Arduino
- Temperature sensor (DHT11 or DS18B20)
- LCD display (optional)
- USB cable

**Setup**: Follow the full tutorial in Chapter 4

### Project 3: Obstacle Avoidance Robot (Advanced)

Build a robot that detects and avoids obstacles.

**Components**: Motors, ultrasonic sensor, Arduino, chassis

**Follow**: Detailed instructions in Chapter 9

## Troubleshooting

### Port Already in Use

If `npm start` fails with "port 3000 already in use":

**Windows**:
```cmd
netstat -ano | findstr :3000
taskkill /PID <PID> /F
```

**macOS/Linux**:
```bash
lsof -ti:3000 | xargs kill -9
```

### Docker Issues

Check logs:
```bash
docker compose logs
```

Rebuild containers:
```bash
docker compose down
docker compose up -d --build
```

### Python Virtual Environment Issues

Recreate virtual environment:
```bash
rm -rf venv
python3 -m venv venv
source venv/bin/activate  # macOS/Linux
pip install -r requirements.txt
```

### Browser Not Loading

- Clear browser cache (Ctrl+Shift+Delete)
- Try incognito/private window
- Check firewall settings
- Try different browser

## Next Steps

1. **Read the Introduction**: Start with Chapter 1
2. **Set Your Level**: Choose Beginner/Intermediate/Advanced
3. **Ask Questions**: Use the chatbot for clarification
4. **Code Along**: Run all examples on your machine
5. **Complete Exercises**: Each chapter has practice problems
6. **Join Community**: Connect with other learners on Discord

## Additional Resources

### Official Documentation
- [ROS 2 Documentation](https://docs.ros.org/)
- [Gazebo Simulation Guide](https://gazebosim.org/)
- [Arduino Reference](https://www.arduino.cc/reference/)
- [Python Documentation](https://docs.python.org/)

### Learning Resources
- [YouTube: ROS 2 Tutorials](https://www.youtube.com/results?search_query=ROS+2+tutorial)
- [GitHub: Community Projects](https://github.com/topics/robotics)
- [Stack Overflow: Robotics Tag](https://stackoverflow.com/questions/tagged/robotics)

### Tools & Software
- [Arduino IDE](https://www.arduino.cc/en/software)
- [Visual Studio Code](https://code.visualstudio.com/)
- [Gazebo](https://gazebosim.org/)
- [RViz Visualization](http://wiki.ros.org/rviz)

## Getting Help

### Before You Ask
1. **Search**: Use Ctrl+F to search in the textbook
2. **Check FAQ**: Review common issues
3. **Read Error Messages**: They often tell you exactly what's wrong
4. **Debug**: Add print statements to understand code flow

### How to Ask Questions
1. **Be Specific**: Include exact error messages
2. **Show Code**: Paste relevant code snippets
3. **Share Environment**: Python version, OS, dependencies
4. **Describe Attempts**: What have you already tried?

### Support Channels
- **AI Chatbot** (Fastest): Ask directly in the platform
- **Discord Community**: Chat with other learners
- **GitHub Issues**: Report bugs or suggest improvements
- **Email**: Contact instructors (if applicable)

## Tips for Success

### Do ✅
- **Code regularly**: Even 30 minutes daily is better than marathon sessions
- **Experiment**: Modify examples to understand them better
- **Ask questions**: The best learners ask the most questions
- **Share learning**: Teaching others reinforces your understanding
- **Keep a journal**: Document what you learn each day

### Don't ❌
- **Rush through chapters**: Understanding > speed
- **Copy without reading**: Always understand before implementing
- **Ignore error messages**: They're your friends
- **Skip prerequisites**: Foundations matter
- **Give up**: Every programmer gets stuck—it's normal

## Self-Assessment Checklist

By the end of this course, you should be able to:

- [ ] Explain what Physical AI is and its applications
- [ ] Set up and program an Arduino or similar microcontroller
- [ ] Work with sensors and interpret sensor data
- [ ] Understand control systems (PID) and implement them
- [ ] Process images and perform basic computer vision tasks
- [ ] Integrate AI models into robotic systems
- [ ] Simulate robots in Gazebo or similar environments
- [ ] Build a complete robot system from scratch
- [ ] Troubleshoot hardware and software issues
- [ ] Communicate your work clearly to others

---

**Ready to start?** Head to [Chapter 1: Introduction to Physical AI](/docs/book/01-introduction/01-introduction-to-physical-ai)

Have fun learning! 🚀🤖
