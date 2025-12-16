// @ts-check

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
    // Add other categories as needed
  ],
};

module.exports = sidebars;