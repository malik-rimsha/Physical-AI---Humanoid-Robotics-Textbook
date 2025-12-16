---
title: Documentation Standards and Frontmatter Guide
sidebar_label: Documentation Standards
---

# Documentation Standards and Frontmatter Guide

This document outlines the standards and frontmatter conventions for all documentation in the Physical AI - Humanoid Robotics Textbook.

## Frontmatter Standards

Each document should include the following frontmatter at the top:

```yaml
---
title: Your Document Title
sidebar_label: Short Label for Sidebar (optional, defaults to title)
description: Brief description of the content
keywords: [keyword1, keyword2, keyword3]
tags: [tag1, tag2]
---

```

## Content Standards

- Use H1 only for the document title (in frontmatter)
- Start content with H2 or lower headings
- Include learning objectives at the beginning of educational content
- Use code blocks with appropriate language identifiers
- Include practical examples and exercises where applicable

## Code Block Standards

- Use appropriate language identifiers: `python`, `xml`, `bash`, `text`, etc.
- For ROS 2 code examples, use `python` for rclpy code
- For URDF files, use `xml`
- For command line examples, use `bash`

## Learning Content Structure

1. Learning objectives
2. Prerequisites
3. Main content with examples
4. Practical exercises
5. Summary