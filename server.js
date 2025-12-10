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

app.listen(PORT, () => {
    console.log(`Server is running on http://localhost:${PORT}`);
});