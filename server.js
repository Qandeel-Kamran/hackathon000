const express = require('express');
const path = require('path');
const app = express();
const PORT = process.env.PORT || 8080;

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

    if (lowerMsg.includes('hello') || lowerMsg.includes('hi') || lowerMsg.includes('hey')) {
        response = "Hello! I'm your AI assistant for Physical AI and Humanoid Robotics education. I can help explain concepts from the textbook, provide examples, or answer questions about robotics. What would you like to learn about?";
    } else if (lowerMsg.includes('kinematics') || lowerMsg.includes('forward') || lowerMsg.includes('inverse')) {
        response = "Kinematics is the study of motion without considering the forces that cause it. Forward kinematics calculates the end-effector position from joint angles, while inverse kinematics determines the joint angles needed to achieve a desired end-effector position. This is fundamental for robot control and manipulation tasks.";
    } else if (lowerMsg.includes('pid') || lowerMsg.includes('control')) {
        response = "PID (Proportional-Integral-Derivative) control is a fundamental control technique in robotics. It uses three terms: Proportional (P) for immediate response to error, Integral (I) to eliminate steady-state error, and Derivative (D) to predict future error. PID controllers are essential for precise robot motion control.";
    } else if (lowerMsg.includes('zmp') || lowerMsg.includes('balance') || lowerMsg.includes('humanoid')) {
        response = "The Zero Moment Point (ZMP) is a crucial concept in humanoid robotics for maintaining balance. It's the point on the ground where the net moment of the ground reaction force is zero. For stable walking, the ZMP must remain within the support polygon defined by the feet.";
    } else if (lowerMsg.includes('music') || lowerMsg.includes('dance') || lowerMsg.includes('rhythm')) {
        response = "Music-based robot control uses audio analysis to drive robot movements. By detecting beats, tempo, and rhythm patterns, robots can synchronize their movements to music. This provides an engaging way to learn about timing, coordination, and control systems in robotics.";
    } else if (lowerMsg.includes('chapter') || lowerMsg.includes('topic') || lowerMsg.includes('learn')) {
        response = "I can help explain concepts from any chapter in the Physical AI & Humanoid Robotics textbook. The book covers topics from basic robotics principles to advanced humanoid control systems. Which specific topic would you like to explore?";
    } else {
        response = `I understand you're asking about "${message}". In the context of Physical AI and Humanoid Robotics, this likely relates to concepts in the textbook. I can provide detailed explanations about robotics principles, ROS 2 implementation, control systems, perception, or simulation. Would you like me to elaborate on any specific aspect?`;
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

// API endpoint for dynamic features
app.get('/api/features', (req, res) => {
    const features = [
        {
            icon: "fas fa-code",
            title: "Interactive Code Examples",
            description: "Run and modify Python and C++ code examples directly in your browser with real-time feedback."
        },
        {
            icon: "fas fa-search",
            title: "Smart Content Search",
            description: "Search across all chapters to find specific topics, concepts, or code examples instantly."
        },
        {
            icon: "fas fa-bookmark",
            title: "Smart Bookmarks",
            description: "Save your favorite sections and return to them later with personalized notes and highlights."
        },
        {
            icon: "fas fa-calculator",
            title: "Mathematical Visualization",
            description: "Interactive tools to visualize complex mathematical concepts in robotics and AI."
        },
        {
            icon: "fas fa-project-diagram",
            title: "System Architecture",
            description: "Explore ROS 2 architecture diagrams and understand how different components interact."
        },
        {
            icon: "fas fa-music",
            title: "Music-Based Learning",
            description: "Learn robot movement concepts through music and rhythm-based interactive examples."
        }
    ];

    res.json({ features });
});

// API endpoint for learning resources
app.get('/api/resources', (req, res) => {
    const resources = [
        {
            icon: "fas fa-book",
            title: "Textbook Chapters",
            description: "Complete textbook content with examples, exercises, and solutions."
        },
        {
            icon: "fas fa-code",
            title: "Code Repository",
            description: "Complete code examples and exercises available for download and experimentation."
        },
        {
            icon: "fas fa-chalkboard-teacher",
            title: "Lecture Slides",
            description: "Presentation slides for each topic to support classroom learning."
        },
        {
            icon: "fas fa-tasks",
            title: "Exercise Sets",
            description: "Practice problems with solutions to reinforce learning concepts."
        },
        {
            icon: "fas fa-graduation-cap",
            title: "Certification Path",
            description: "Structured learning paths with assessments and certificates of completion."
        }
    ];

    res.json({ resources });
});

app.listen(PORT, () => {
    console.log(`Server is running on http://localhost:${PORT}`);
});