Check `SpecimenCycleAutoBase` in the `opmode` package as an example. It's a working 5+0 autonomous.

This is not based on the latest Pedro Pathing code, because it doesn't work for Pinpoint localizer. Our code works for Pinpoint, it's confirmed by team 21229 Quality Control and others.

If you don't use Pinpoint localizer and want to use the latest Pedo code, you just need to copy `pedroPathing.follower.FollowPathAction` and `build.gradle`.

There are `ActionSchedulers` in the `utils.software` package, one for Auto and one for TeleOps that you can use to queue up actions.

Please mail us at ftcteam12611@gmail.com if you have any questions!


