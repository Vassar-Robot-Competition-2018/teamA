# Week 8: 3D Printing
This week we installed our first prototype of the sliding chamber, 3D printed our wheel mounts, and developed some of the software for using the camera. 

## 3D Printed Wheel Mounts
On Tuesday, Eliana worked on 3D printing the wheel mounts for our robot. She designed a block with cutouts for the servo and color sensor. The color sensor should slide into a mount on the bottom of the main block, but the 3D printer wasn't able to successfully print the lip on one side of the mount. Another issue is that the right servo mount needs to be flipped so that the servo mount is facing the right direction.

On Monday, Gabe worked on preparing our body to install the mounts by removing the previous mounts. This necessitated scraping off some of the old foamcore mount since it was hot-glued on.

## Sliding Chamber
On Tuesday, Gabe worked on getting the sliding chamber installed. He glued the gear rack on to the top of the sliding mount. There might be some issues with the torque of the servo shredding the gear or the gear rack. We plan on mitigating this issue by making sure that the gear and the rack are mounted tightly together, as well as trying to slow the servo if necessary.

## Camera Testing
George worked on preparing some of the software for using the camera on the robot. We will be using an algorithm that slows the left or right wheel as necessary to try and center a block in the camera. It will drastically slow the wheel when the block is close to either side, and then as either block gets closer to the center it will increase the speed until the block is entirely in the center. Once the block is entirely in the center both wheels will be on at full speed.
