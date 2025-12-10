# Data Model: Physical AI & Humanoid Robotics Book

## Content Entities

### Chapter
- **Description**: A major section of the book covering a specific aspect of Physical AI or humanoid robotics
- **Attributes**:
  - id: Unique identifier (e.g., "ch01", "ch02")
  - title: Full chapter title
  - number: Chapter sequence number
  - subtopics: Array of 3-5 subtopic strings
  - wordCount: Estimated word count range (object with min/max)
  - prerequisites: Array of prerequisite knowledge areas
  - learningOutcomes: Array of 3 measurable learning outcomes
  - status: Draft, Review, Complete
  - author: Author identifier
  - reviewers: Array of reviewer identifiers

### Subtopic
- **Description**: A specific area within a chapter that addresses a particular concept or technique
- **Attributes**:
  - id: Unique identifier within parent chapter
  - title: Subtopic title
  - content: Detailed content for the subtopic
  - relatedConcepts: Array of related concepts from other chapters
  - codeExamples: Array of associated code example IDs

### Code Example
- **Description**: Practical implementation demonstrating concepts from the chapters
- **Attributes**:
  - id: Unique identifier
  - title: Example title
  - description: Brief description of what the example demonstrates
  - language: Programming language used
  - chapterId: Reference to parent chapter
  - files: Array of file paths for the example
  - dependencies: List of required libraries or frameworks
  - complexity: Beginner, Intermediate, Advanced

### Visual Asset
- **Description**: Diagram, chart, illustration, or 3D render supporting chapter content
- **Attributes**:
  - id: Unique identifier
  - type: Diagram, Chart, Illustration, 3D Render, Photo
  - title: Asset title
  - description: Brief description of the asset
  - chapterId: Reference to parent chapter
  - filePath: Path to the asset file
  - specifications: Technical specifications for the asset
  - status: Concept, In Design, Complete

### Learning Outcome
- **Description**: A measurable skill or knowledge that readers should acquire from each chapter
- **Attributes**:
  - id: Unique identifier
  - text: The measurable outcome statement
  - chapterId: Reference to parent chapter
  - type: Knowledge, Skill, Understanding
  - measurementCriteria: How to measure achievement of this outcome

## Relationship Model

```
[Chapter] 1--* [Subtopic]
[Chapter] 1--* [Code Example]
[Chapter] 1--* [Visual Asset]
[Chapter] 1--* [Learning Outcome]
[Subtopic] --* [Code Example] (via relatedConcepts)
```

## Validation Rules

### Chapter Validation
- MUST have 12-18 chapters total (as per specification)
- MUST include 3-5 subtopics per chapter
- MUST specify word count range between 500-2000 words
- MUST define prerequisite knowledge
- MUST include 3 measurable learning outcomes
- MUST align with Physical AI and humanoid robotics focus

### Content Quality Validation
- All content MUST follow safety-first design principles
- All content MUST integrate interdisciplinary perspectives
- All content MUST emphasize real-world validation
- All content MUST include ethical considerations
- All content MUST be technically accurate and validated

### Learning Outcome Validation
- MUST be measurable and specific
- MUST align with chapter content
- MUST be achievable through chapter material
- MUST follow SMART criteria (Specific, Measurable, Achievable, Relevant, Time-bound)

## State Transitions

### Chapter State Model
```
Draft → Technical Review → Editorial Review → Complete
   ↑                                        ↓
   └-------- Revision Required ←--------------┘
```

### Asset State Model
```
Concept → Design → Review → Approved → Integrated
                    ↑           ↓
                    └--- Rejected ---┘
```

## Content Flow Validation

### Cross-Reference Validation
- All chapters MUST link to relevant prerequisite chapters
- All technical concepts MUST link to foundational chapters
- All code examples MUST reference relevant chapters
- All visual assets MUST enhance understanding of concepts

### Quality Assurance Validation
- All content MUST pass technical review
- All content MUST pass editorial review
- All safety considerations MUST be addressed
- All ethical frameworks MUST be included