# Quickstart Guide: ROS 2 Module Development

## Prerequisites

1. **System Requirements**:
   - Ubuntu 22.04 LTS or equivalent
   - At least 8GB RAM
   - Node.js 16+ for Docusaurus
   - ROS 2 Humble Hawksbill or later

2. **Install ROS 2**:
   ```bash
   # Add ROS 2 repository
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install python3-colcon-common-extensions
   sudo apt install python3-rosdep
   ```

3. **Setup ROS 2 Environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

4. **Install Docusaurus Prerequisites**:
   ```bash
   # Install Node.js if not already installed
   curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
   sudo apt install -y nodejs
   npm install -g @docusaurus/cli
   ```

## Setting Up the Documentation Project

1. **Initialize Docusaurus**:
   ```bash
   # Navigate to your project root
   npx create-docusaurus@latest website classic
   cd website
   ```

2. **Install Additional Dependencies**:
   ```bash
   npm install @docusaurus/module-type-aliases @docusaurus/types
   ```

3. **Create ROS 2 Module Directory Structure**:
   ```bash
   mkdir -p docs/modules/ros2
   ```

## Creating the Three Chapters

1. **Chapter 1: The Core of ROS 2**:
   ```bash
   # Create the first chapter file
   touch docs/modules/ros2/chapter-1-core-of-ros2.md
   ```

2. **Chapter 2: Programming Embodiment**:
   ```bash
   # Create the second chapter file
   touch docs/modules/ros2/chapter-2-programming-embodiment.md
   ```

3. **Chapter 3: Defining the Humanoid**:
   ```bash
   # Create the third chapter file
   touch docs/modules/ros2/chapter-3-defining-the-humanoid.md
   ```

## Configuring Docusaurus Sidebar

1. **Update sidebar.js**:
   ```javascript
   // In your docusaurus.config.js, add the sidebar configuration
   module.exports = {
     // ... other config
     presets: [
       [
         'classic',
         /** @type {import('@docusaurus/preset-classic').Options} */
         ({
           docs: {
             sidebarPath: require.resolve('./sidebars.js'),
             // ... other options
           },
           // ... other presets
         }),
       ],
     ],
   };
   ```

2. **Create sidebar configuration**:
   ```javascript
   // sidebars.js
   /** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
   const sidebars = {
     tutorialSidebar: [
       'intro',
       {
         type: 'category',
         label: 'ROS 2 Module',
         items: [
           'modules/ros2/chapter-1-core-of-ros2',
           'modules/ros2/chapter-2-programming-embodiment',
           'modules/ros2/chapter-3-defining-the-humanoid',
         ],
       },
       // ... other items
     ],
   };

   module.exports = sidebars;
   ```

## Running the Documentation Site

1. **Start Development Server**:
   ```bash
   cd website
   npm start
   ```

2. **Build for Production**:
   ```bash
   npm run build
   ```

3. **Deploy to GitHub Pages**:
   ```bash
   GIT_USER=<your-github-username> npm run deploy
   ```

## Testing Code Examples

1. **ROS 2 Publisher/Subscriber Example**:
   ```bash
   # Create a ROS 2 workspace for testing
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

2. **Run the examples from the documentation**:
   ```bash
   # After creating your publisher/subscriber nodes from the docs
   ros2 run your_package_name publisher_node
   ros2 run your_package_name subscriber_node
   ```

## Next Steps

1. Complete the content for each chapter following the specification
2. Add code examples with proper explanations
3. Include exercises and practical applications
4. Test all examples in a ROS 2 environment
5. Review and validate the documentation structure