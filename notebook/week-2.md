# Week 2 Notebook

This week we mostly discussed and began prototyping the structure of our robot. Our idea is to have our robot be a type of vacuum where it just drives all over the arena until it finds blocks and then funnel the blocks towards a flap. There is a sensor near the flap which determines what color the block is and whether we want to store or ignore the block. If we decide to store the block we will keep it in a storage container in the back of the underside of the robot. Then before the time is up we will have the robot navigate back into our quandrant and switch into a mode that tries to remove blocks from our quadrant.

## Funnel

We takled about multiple issues with the design of the funnel. The first is that we don't want blocks to get stuck in the funnel. In order to try and resolve this problem we decided to make the funnel deep enough so that the friction of the material of the funnel will be less than the friction of the arena, allowing the blocks to slide towards the flap without getting stuck. We also decided after building our initial prototype that curving the funnel will keep the block smoothly going towards the flap, which will also reduce the times blocks get stuck in the funnel.

## Flappy McDoodle

We also discussed a bit about the design of our flap (nicknamed Flappy McDoodle). The flap will be controlled by a servo attached to either a piece of foam core or a 3d printed part. The blocks will stop before the flap, a sensor will sense what color the block is, and then either open the flap if we want to store the block, or let the block fall through and out of our robot.

## Acknowledgments

We all worked on coming up with the ideas for the structure. Gabe and Eliana worked on cutting and gluing the foam core for our initial prototype of the funnel. George set up the Arduino and Ardunio code to get the robot moving in order to test the funnel. Then we all tested the funnel with different block arrangements on the arena.