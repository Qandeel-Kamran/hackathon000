const express = require('express');
const path = require('path');
const app = express();
const PORT = process.env.PORT || 3000;

// Serve static files from the 'public' directory
app.use(express.static(path.join(__dirname, 'public')));

// Serve static files from the 'docs' directory to access book content
app.use('/docs', express.static(path.join(__dirname, 'docs')));

// Route for the main page
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Route for individual chapters
app.get('/chapter/:id', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'chapter.html'));
});

// Route for feature details
app.get('/feature/:id', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'feature-details.html'));
});

// API endpoint to get chapter content
app.get('/api/chapter/:id', (req, res) => {
    const chapterId = req.params.id;
    const fs = require('fs');

    // Validate chapter ID to prevent directory traversal
    if (!/^[0-9]+$/.test(chapterId) || parseInt(chapterId) < 1 || parseInt(chapterId) > 18) {
        return res.status(404).json({ error: 'Chapter not found' });
    }

    const chapterPath = path.join(__dirname, 'docs', `chapter-${chapterId}`);

    // Read all markdown files in the chapter directory
    fs.readdir(chapterPath, (err, files) => {
        if (err) {
            return res.status(404).json({ error: 'Chapter not found' });
        }

        const mdFile = files.find(file => file.endsWith('.md'));
        if (!mdFile) {
            return res.status(404).json({ error: 'Chapter content not found' });
        }

        const filePath = path.join(chapterPath, mdFile);
        fs.readFile(filePath, 'utf8', (err, data) => {
            if (err) {
                return res.status(500).json({ error: 'Error reading chapter' });
            }

            // Extract frontmatter and content
            const frontmatterMatch = data.match(/^---\n([\s\S]*?)\n---/);
            let frontmatter = {};
            let content = data;

            if (frontmatterMatch) {
                const frontmatterStr = frontmatterMatch[1];
                content = data.slice(frontmatterMatch[0].length).trim();

                // Simple frontmatter parsing (in a real app, use a library like gray-matter)
                const lines = frontmatterStr.split('\n');
                lines.forEach(line => {
                    const [key, ...valueParts] = line.split(': ');
                    if (key && valueParts.length > 0) {
                        const value = valueParts.join(': ').trim();
                        // Remove quotes if present
                        frontmatter[key.trim()] = value.replace(/^"|"$/g, '').replace(/^'|'$/g, '');
                    }
                });
            }

            res.json({
                frontmatter: frontmatter,
                content: content,
                raw: data
            });
        });
    });
});

// API endpoint to get all chapters
app.get('/api/chapters', (req, res) => {
    const fs = require('fs');

    // Read all chapter directories
    fs.readdir(path.join(__dirname, 'docs'), (err, files) => {
        if (err) {
            return res.status(500).json({ error: 'Error reading chapters' });
        }

        const chapters = files
            .filter(file => file.startsWith('chapter-') && !isNaN(file.split('-')[1]))
            .map(file => {
                const chapterNum = file.split('-')[1];
                return {
                    id: chapterNum,
                    path: `/chapter/${chapterNum}`,
                    apiPath: `/api/chapter/${chapterNum}`
                };
            })
            .sort((a, b) => parseInt(a.id) - parseInt(b.id));

        res.json({ chapters });
    });
});

// API endpoint for chat functionality
app.post('/api/chat', express.json(), (req, res) => {
    const { message } = req.body;

    if (!message) {
        return res.status(400).json({ error: 'Message is required' });
    }

    // In a real implementation, this would connect to the ROS 2 conversational AI node
    // For now, we'll return a simulated response based on the message content
    const lowerMsg = message.toLowerCase();
    let response = '';

    // Enhanced knowledge base for robotics and AI concepts
    if (lowerMsg.includes('hello') || lowerMsg.includes('hi') || lowerMsg.includes('hey')) {
        response = "Hello! I'm your AI assistant for Physical AI and Humanoid Robotics education. I can help explain concepts from the textbook, provide examples, or answer questions about robotics. What would you like to learn about?";
    } else if (lowerMsg.includes('kinematics') || lowerMsg.includes('forward') || lowerMsg.includes('inverse')) {
        if (lowerMsg.includes('forward')) {
            response = "Forward kinematics is the process of determining the position and orientation of a robot's end-effector based on the known joint angles. It involves using transformation matrices to map from joint space to Cartesian space. For example, in a robotic arm, if you know all the joint angles, you can calculate exactly where the gripper will be in 3D space.";
        } else if (lowerMsg.includes('inverse')) {
            response = "Inverse kinematics is the reverse process of forward kinematics, where you determine the required joint angles to achieve a desired end-effector position and orientation. This is more complex than forward kinematics and often has multiple solutions or no solution. Common methods include analytical solutions for simple robots and numerical methods like the Jacobian transpose or pseudoinverse for more complex robots.";
        } else {
            response = "Kinematics is the study of motion without considering the forces that cause it. Forward kinematics calculates the end-effector position from joint angles, while inverse kinematics determines the joint angles needed to achieve a desired end-effector position. This is fundamental for robot control and manipulation tasks. The Jacobian matrix relates joint velocities to end-effector velocities and is crucial for motion planning.";
        }
    } else if (lowerMsg.includes('pid') || lowerMsg.includes('control')) {
        if (lowerMsg.includes('tuning') || lowerMsg.includes('parameters')) {
            response = "PID controller tuning involves adjusting the proportional (Kp), integral (Ki), and derivative (Kd) gains to achieve optimal system response. Common methods include Ziegler-Nichols tuning, where you first find the ultimate gain that causes sustained oscillations, then use formulas to set the PID parameters. The P term affects response speed, I term eliminates steady-state error, and D term reduces overshoot and improves stability.";
        } else if (lowerMsg.includes('proportional') || lowerMsg.includes('integral') || lowerMsg.includes('derivative')) {
            response = "In PID control, each component has a specific role: Proportional (P) provides immediate response proportional to the error; Integral (I) accumulates past errors to eliminate steady-state error; Derivative (D) predicts future error based on the rate of change. The combination allows for precise control of robot movements, balancing speed, accuracy, and stability.";
        } else {
            response = "PID (Proportional-Integral-Derivative) control is a fundamental control technique in robotics. It uses three terms: Proportional (P) for immediate response to error, Integral (I) to eliminate steady-state error, and Derivative (D) to predict future error. PID controllers are essential for precise robot motion control, from simple motor position control to complex trajectory following.";
        }
    } else if (lowerMsg.includes('zmp') || lowerMsg.includes('balance') || lowerMsg.includes('humanoid')) {
        if (lowerMsg.includes('walking') || lowerMsg.includes('gait')) {
            response = "In humanoid walking, the ZMP (Zero Moment Point) trajectory is planned to ensure stable locomotion. The ZMP must stay within the support polygon defined by the feet. During walking, the ZMP typically moves from the heel to the toe of the stance foot, then quickly to the heel of the swing foot as it becomes the new stance foot. This creates the characteristic 'flat-footed' walking pattern in most humanoid robots.";
        } else if (lowerMsg.includes('stability')) {
            response = "ZMP stability is achieved when the ZMP remains within the support polygon. For a humanoid robot, this means the robot's center of mass projection must be controlled such that the net moment around the contact points with the ground is zero. This is accomplished through careful control of the robot's joint torques and sometimes active ankle control to maintain balance.";
        } else {
            response = "The Zero Moment Point (ZMP) is a crucial concept in humanoid robotics for maintaining balance. It's the point on the ground where the net moment of the ground reaction force is zero. For stable walking, the ZMP must remain within the support polygon defined by the feet. This concept is fundamental to humanoid robot control and is used in trajectory planning algorithms like the Linear Inverted Pendulum Model (LIPM).";
        }
    } else if (lowerMsg.includes('ros') || lowerMsg.includes('node') || lowerMsg.includes('topic') || lowerMsg.includes('service') || lowerMsg.includes('action')) {
        if (lowerMsg.includes('node')) {
            response = "In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS program. Each node is typically responsible for a specific task, such as sensor data processing, control algorithm execution, or user interface. Nodes communicate with each other through topics, services, or actions using a publish-subscribe model or request-response pattern.";
        } else if (lowerMsg.includes('topic')) {
            response = "Topics in ROS 2 are used for asynchronous communication between nodes using a publish-subscribe pattern. A node publishes messages to a topic, and other nodes can subscribe to that topic to receive the messages. Topics are ideal for streaming data like sensor readings, robot states, or commands. Examples include '/joint_states' for robot joint positions and '/cmd_vel' for velocity commands.";
        } else if (lowerMsg.includes('service')) {
            response = "Services in ROS 2 provide synchronous request-response communication between nodes. A service client sends a request to a service server and waits for a response. Services are appropriate for tasks that have a clear request-response pattern, such as changing robot parameters, triggering specific behaviors, or requesting computation results. Unlike topics, services are synchronous and block until a response is received.";
        } else if (lowerMsg.includes('action')) {
            response = "Actions in ROS 2 are used for long-running tasks that may take a significant amount of time to complete. They provide feedback during execution, can be canceled, and return a result when completed. Actions are ideal for navigation, trajectory execution, or any task where you need to know the progress, not just the final result. Examples include moving to a goal position or executing a complex manipulation task.";
        } else {
            response = "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides services such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. Key concepts include nodes (processes), topics (publish-subscribe communication), services (request-response communication), and actions (long-running tasks with feedback).";
        }
    } else if (lowerMsg.includes('gazebo') || lowerMsg.includes('simulation')) {
        response = "Gazebo is a 3D simulation environment that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It's commonly used with ROS 2 for robot simulation before deploying to real hardware. Gazebo allows you to test robot algorithms, sensor configurations, and control strategies in a safe, repeatable environment. It supports various physics engines, sensors (cameras, LIDAR, IMU), and provides plugins for custom robot behaviors.";
    } else if (lowerMsg.includes('isaac') || lowerMsg.includes('perception')) {
        response = "Isaac Sim (from NVIDIA) is a robotics simulation platform that provides high-fidelity physics simulation and synthetic data generation for AI training. It's particularly powerful for perception tasks, offering realistic sensor simulation including cameras, LIDAR, and IMU sensors. Isaac Sim integrates with ROS 2 and provides tools for developing perception algorithms, SLAM (Simultaneous Localization and Mapping), and computer vision applications for robotics.";
    } else if (lowerMsg.includes('slam') || lowerMsg.includes('localization') || lowerMsg.includes('mapping')) {
        if (lowerMsg.includes('simultaneous')) {
            response = "SLAM (Simultaneous Localization and Mapping) is a technique that allows a robot to build a map of an unknown environment while simultaneously keeping track of its location within that map. This is a fundamental capability for autonomous robots operating in unstructured environments. Common approaches include EKF SLAM, FastSLAM, and graph-based SLAM methods. SLAM typically uses sensors like LIDAR, cameras, or IMUs to perceive the environment.";
        } else if (lowerMsg.includes('lidar')) {
            response = "LIDAR-based SLAM uses laser range finders to create accurate 2D or 3D maps of the environment. Popular algorithms include Hector SLAM, Gmapping, and LOAM (Lidar Odometry and Mapping). LIDAR provides precise distance measurements and works well in various lighting conditions, making it ideal for indoor mapping and navigation tasks. The data is typically processed to extract features like walls, obstacles, and landmarks.";
        } else {
            response = "Localization and mapping are fundamental capabilities for autonomous robots. Localization determines the robot's position and orientation in a known or unknown environment, while mapping creates a representation of the environment. SLAM (Simultaneous Localization and Mapping) combines both capabilities, allowing robots to operate in previously unexplored environments. Common sensors include LIDAR, cameras, and IMUs, with algorithms like particle filters, EKF, or graph optimization.";
        }
    } else if (lowerMsg.includes('chapter') || lowerMsg.includes('topic') || lowerMsg.includes('learn')) {
        response = "I can help explain concepts from any chapter in the Physical AI & Humanoid Robotics textbook. The book covers topics from basic robotics principles to advanced humanoid control systems. Which specific topic would you like to explore? I can provide detailed explanations about forward and inverse kinematics, dynamics, control systems, sensor integration, perception algorithms, or humanoid-specific concepts like balance control and walking patterns.";
    } else if (lowerMsg.includes('trajectory') || lowerMsg.includes('motion') || lowerMsg.includes('path')) {
        if (lowerMsg.includes('planning')) {
            response = "Motion planning involves finding a valid path from a start configuration to a goal configuration while avoiding obstacles. Common approaches include sampling-based methods like RRT (Rapidly-exploring Random Trees) and PRM (Probabilistic Roadmap), as well as optimization-based methods. For humanoid robots, motion planning must consider the robot's complex kinematics, balance constraints, and dynamic capabilities.";
        } else if (lowerMsg.includes('interpolation')) {
            response = "Trajectory interpolation creates smooth, continuous paths between waypoints by specifying position, velocity, and acceleration over time. Common methods include linear interpolation, polynomial interpolation (cubic, quintic), and splines. For robot control, trajectories must be smooth to avoid jerky movements that could damage the robot or cause instability in dynamic systems like humanoid robots.";
        } else {
            response = "Trajectory planning involves creating smooth, collision-free paths for robots to follow. This includes both path planning (finding a geometric path) and trajectory generation (adding timing information). For humanoid robots, trajectories must consider balance, joint limits, and dynamic constraints. Common representations include joint-space trajectories and Cartesian-space trajectories.";
        }
    } else if (lowerMsg.includes('dynamics') || lowerMsg.includes('force') || lowerMsg.includes('torque')) {
        if (lowerMsg.includes('newton') || lowerMsg.includes('euler')) {
            response = "Newton-Euler dynamics is a recursive method for computing the forward dynamics of a robot, determining the joint accelerations given the joint torques and external forces. The inverse dynamics problem computes the required joint torques given the joint positions, velocities, and accelerations. These are fundamental for robot control, simulation, and trajectory optimization in both fixed-base and mobile robots.";
        } else {
            response = "Robot dynamics deals with the relationship between forces acting on a robot and the resulting motion. This includes forward dynamics (computing motion from forces) and inverse dynamics (computing forces from motion). For humanoid robots, dynamics is crucial for balance control, walking, and manipulation tasks. The equations of motion are typically derived using Lagrangian mechanics or Newton-Euler methods.";
        }
    } else if (lowerMsg.includes('machine learning') || lowerMsg.includes('ai') || lowerMsg.includes('neural')) {
        response = "Machine learning in robotics encompasses various techniques for enabling robots to learn from experience and adapt to new situations. This includes reinforcement learning for control tasks, deep learning for perception (computer vision, sensor processing), and imitation learning for skill acquisition. In humanoid robotics, ML techniques are used for gait learning, manipulation skill acquisition, and adaptive control strategies.";
    } else if (lowerMsg.includes('control') && (lowerMsg.includes('system') || lowerMsg.includes('theory'))) {
        response = "Control systems in robotics ensure that robots behave as desired by regulating their behavior through feedback. Classical control includes PID controllers, while modern control encompasses state-space methods, optimal control, and robust control. For humanoid robots, advanced control techniques like model predictive control (MPC) and whole-body control are often used to coordinate multiple tasks while maintaining balance.";
    } else {
        response = `I understand you're asking about "${message}". In the context of Physical AI and Humanoid Robotics, this likely relates to concepts in the textbook. I can provide detailed explanations about robotics principles, ROS 2 implementation, control systems, perception, or simulation. For example, I can explain kinematics, dynamics, control theory, sensor fusion, SLAM, or humanoid-specific topics like balance control and walking patterns. What specific aspect would you like me to elaborate on?`;
    }

    // Simulate processing delay
    setTimeout(() => {
        res.json({
            success: true,
            response: response,
            timestamp: new Date().toISOString()
        });
    }, 500);
});

// API endpoint for search functionality
app.get('/api/search', (req, res) => {
    const query = req.query.q ? req.query.q.toLowerCase() : '';

    if (!query) {
        return res.json({ results: [] });
    }

    const fs = require('fs');
    const path = require('path');

    // Read all chapter directories
    fs.readdir(path.join(__dirname, 'docs'), (err, files) => {
        if (err) {
            return res.status(500).json({ error: 'Error reading chapters' });
        }

        const chapterDirs = files.filter(file =>
            file.startsWith('chapter-') &&
            fs.statSync(path.join(__dirname, 'docs', file)).isDirectory()
        );

        const results = [];

        chapterDirs.forEach(chapterDir => {
            const chapterPath = path.join(__dirname, 'docs', chapterDir);
            const chapterFiles = fs.readdirSync(chapterPath);

            chapterFiles.forEach(file => {
                if (file.endsWith('.md')) {
                    const filePath = path.join(chapterPath, file);
                    const content = fs.readFileSync(filePath, 'utf8');
                    const chapterNum = chapterDir.split('-')[1];

                    // Search in content (case insensitive)
                    if (content.toLowerCase().includes(query)) {
                        // Extract a snippet around the search term
                        const contentLower = content.toLowerCase();
                        const queryIndex = contentLower.indexOf(query);
                        const start = Math.max(0, queryIndex - 100);
                        const end = Math.min(content.length, queryIndex + query.length + 100);
                        const snippet = content.substring(start, end);

                        results.push({
                            chapter: chapterNum,
                            title: file.replace('.md', '').replace(/-/g, ' '),
                            path: `/chapter/${chapterNum}`,
                            snippet: `...${snippet}...`,
                            apiPath: `/api/chapter/${chapterNum}`
                        });
                    }
                }
            });
        });

        res.json({ results: results });
    });
});

// API endpoint for dynamic features
app.get('/api/features', (req, res) => {
    const features = [
        {
            id: "interactive-code",
            icon: "fas fa-code",
            title: "Interactive Code Examples",
            description: "Run and modify Python and C++ code examples directly in your browser with real-time feedback.",
            details: "This feature allows students to experiment with real robotics code examples without needing to set up complex development environments. You can modify parameters, see immediate results, and understand how code changes affect robot behavior. Examples include forward kinematics, PID controllers, path planning algorithms, and more."
        },
        {
            id: "smart-search",
            icon: "fas fa-search",
            title: "Smart Content Search",
            description: "Search across all chapters to find specific topics, concepts, or code examples instantly.",
            details: "Quickly locate information across the entire textbook using our intelligent search system. Search for concepts like 'inverse kinematics', 'SLAM', 'ZMP', or 'trajectory planning' and find relevant sections, code examples, and exercises instantly."
        },
        {
            id: "smart-bookmarks",
            icon: "fas fa-bookmark",
            title: "Smart Bookmarks",
            description: "Save your favorite sections and return to them later with personalized notes and highlights.",
            details: "Bookmark important sections, add your own notes and annotations, and create a personalized learning path. Your bookmarks sync across sessions, so you can continue learning where you left off."
        },
        {
            id: "math-visualization",
            icon: "fas fa-calculator",
            title: "Mathematical Visualization",
            description: "Interactive tools to visualize complex mathematical concepts in robotics and AI.",
            details: "Understand complex mathematical concepts through interactive visualizations. See how rotation matrices work, visualize Jacobian transformations, explore coordinate transformations, and understand the geometry behind robot movements."
        },
        {
            id: "system-architecture",
            icon: "fas fa-project-diagram",
            title: "System Architecture",
            description: "Explore ROS 2 architecture diagrams and understand how different components interact.",
            details: "Dive deep into ROS 2 architecture with interactive diagrams showing how nodes, topics, services, and actions work together. Understand the communication patterns used in robotics applications and learn best practices for system design."
        }
    ];

    res.json({ features });
});

// API endpoint for learning resources (removed)
// app.get('/api/resources', (req, res) => {
//     const resources = [
//         {
//             icon: "fas fa-book",
//             title: "Textbook Chapters",
//             description: "Complete textbook content with examples, exercises, and solutions."
//         },
//         {
//             icon: "fas fa-code",
//             title: "Code Repository",
//             description: "Complete code examples and exercises available for download and experimentation."
//         },
//         {
//             icon: "fas fa-chalkboard-teacher",
//             title: "Lecture Slides",
//             description: "Presentation slides for each topic to support classroom learning."
//         },
//         {
//             icon: "fas fa-tasks",
//             title: "Exercise Sets",
//             description: "Practice problems with solutions to reinforce learning concepts."
//         },
//         {
//             icon: "fas fa-graduation-cap",
//             title: "Certification Path",
//             description: "Structured learning paths with assessments and certificates of completion."
//         }
//     ];

//     res.json({ resources });
// });

app.listen(PORT, () => {
    console.log(`Server is running on http://localhost:${PORT}`);
});